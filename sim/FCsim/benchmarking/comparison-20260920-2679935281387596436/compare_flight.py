#!/usr/bin/env python3
"""Compare OR and GS1 using barometric height; --show-3d opens a rotatable Matplotlib view.

Requires numpy and matplotlib. Inputs are read only. Coordinates are east/north relative
to each flight's own prelaunch GPS position, with height above its prelaunch barometer.
GPS altitude is used only in the separately labeled low-confidence diagnostic.
"""
import argparse
import ast
import csv
import hashlib
import json
from pathlib import Path
import re

import numpy as np
import matplotlib

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
DEFAULT_RUN = ROOT / 'clone/openrocket-release-24.12.RC.01/build/libs/fc-telemetry/zephyrus-2679935281387596436'
COLORS = {'OpenRocket': '#2166ac', 'GS1': '#c75b13'}
MAX_GAP_S = 0.5


def read_log(path):
    physics, events, baro, pwm = [], [], [], []
    fc_start = None
    with path.open() as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.startswith('ZEPHYRUS '):
                continue
            values = dict(re.findall(r'(\w+)=([^\s]+)', line))
            action = values.get('action')
            if action == 'fc.transition' and values.get('to') == 'FLIGHT':
                fc_start = int(values['flight_begin_ms'])
            if action == 'physics.accept':
                physics.append([float(values['simulation_s']), float(values['truth_altitude_m']), float(values['truth_velocity_mps'])])
            if action == 'baro.update':
                baro.append([int(values['boot_us'])/1e6, float(values['temperature_C'])])
            if action == 'pwm.latch':
                pwm.append([int(values['boot_us'])/1e6, float(values['pulse_us']), float(values['exposed'])])
            if action in ['fc.transition', 'airbrakes.transition', 'pyro.fire', 'simulation.start', 'simulation.end']:
                values['log_line'] = line_number
                events.append(values)
    if fc_start is None:
        raise ValueError('No FC FLIGHT transition found in the console log')
    return {'start_ms': fc_start, 'physics': np.asarray(physics), 'baro': np.asarray(baro), 'pwm': np.asarray(pwm), 'events': events}


def read_flight(path, name, start_ms=None):
    with path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    keys = ['flight_time', 'gpsalt', 'gps_fix', 'lat', 'lon', 'barofilteredalt', 'temp', 'pktnum', 'accel_integrated_velo']
    d = {key: np.asarray([float(row[key]) for row in rows]) for key in keys}
    for key in ['servos', 'gyro', 'accelerometer']:
        d[key] = np.asarray([ast.literal_eval(row[key]) for row in rows], dtype=float)
    d['state'] = np.asarray([row['state'] for row in rows])
    d['name'], d['path'], d['rows'] = name, path, rows
    if start_ms is None:
        start_ms = d['flight_time'][np.flatnonzero(d['state'] == 'state.FLIGHT')[0]]
    d['start_ms'] = int(start_ms)
    d['t'] = (d['flight_time']-start_ms)/1000
    assert np.all(np.diff(d['t']) > 0), 'Expected increasing firmware time'
    pad = (d['t'] >= -3) & (d['t'] < -0.2) & (d['state'] == 'state.PRE_FLIGHT')
    if not np.any(pad):
        raise ValueError(f'{name}: no prelaunch baseline samples')
    d['baseline'] = {key: float(np.median(d[key][pad])) for key in ['barofilteredalt', 'gpsalt', 'lat', 'lon', 'temp']}
    d['baseline']['samples'] = int(pad.sum())
    # Broad, explicit plausibility bounds for this supplied ~5 km flight, not a generic filter.
    d['bad'] = ((np.abs(d['gpsalt']) > 10000) | (np.abs(d['accel_integrated_velo']) > 2000)
                | (np.abs(d['lat']-d['baseline']['lat']) > 0.1)
                | (np.abs(d['lon']-d['baseline']['lon']) > 0.1))
    d['height'] = d['barofilteredalt']-d['baseline']['barofilteredalt']
    d['gps_height'] = d['gpsalt']-d['baseline']['gpsalt']
    d['height'][d['bad']] = np.nan
    # WGS84 local tangent-plane linearization; adequate for these few-kilometer tracks.
    lat0 = np.radians(d['baseline']['lat'])
    a, e2 = 6378137.0, 6.69437999014e-3
    normal = a/np.sqrt(1-e2*np.sin(lat0)**2)
    meridian = a*(1-e2)/(1-e2*np.sin(lat0)**2)**1.5
    d['east'] = np.radians(d['lon']-d['baseline']['lon'])*normal*np.cos(lat0)
    d['north'] = np.radians(d['lat']-d['baseline']['lat'])*meridian
    d['position_ok'] = (d['gps_fix'] == 3) & ~d['bad'] & (d['t'] >= 0)
    # Vertical GPS is visibly unsettled early in GS1; the last flight fix recovery is the cutoff.
    invalid = np.flatnonzero((d['t'] >= 0) & (d['gps_fix'] != 3))
    cutoff = float(d['t'][invalid[-1]+1]) if len(invalid) else 0.0
    d['gps_stable_from_s'] = cutoff
    d['gps_height'][~(d['position_ok'] & (d['t'] >= cutoff))] = np.nan
    return d


def interpolate(t, y, grid):
    """Interpolate only short gaps; never extrapolate across a missing flight interval."""
    good = np.isfinite(y)
    t, y = t[good], y[good]
    out = np.full(grid.shape, np.nan)
    for segment in np.split(np.arange(len(t)), np.flatnonzero(np.diff(t) > MAX_GAP_S)+1):
        if len(segment) < 2:
            continue
        inside = (grid >= t[segment[0]]) & (grid <= t[segment[-1]])
        out[inside] = np.interp(grid[inside], t[segment], y[segment])
    return out


def plot_series(ax, d, values, **style):
    y = np.asarray(values, dtype=float).copy()
    y[np.r_[False, np.diff(d['t']) > MAX_GAP_S]] = np.nan
    ax.plot(d['t'], y, **style)


def derivative(d, grid):
    """Local least-squares slope of barometric height over a one-second window."""
    result = np.full(grid.shape, np.nan)
    for i, center in enumerate(grid):
        select = (np.abs(d['t']-center) <= 0.5) & np.isfinite(d['height'])
        t, y = d['t'][select], d['height'][select]
        if len(t) >= 8 and t[-1]-t[0] >= 0.6 and np.max(np.diff(t)) <= MAX_GAP_S:
            x = t-t.mean()
            result[i] = np.dot(x, y-y.mean())/np.dot(x, x)
    return result


def metrics(d):
    flight = (d['t'] >= 0) & ~d['bad']
    peak = np.nanargmax(np.where(flight, d['height'], np.nan))
    gps_peak = np.nanargmax(d['gps_height'])
    v = np.where(flight, d['accel_integrated_velo'], np.nan)
    opened = np.flatnonzero(flight & (d['servos'][:, 0] < 939))
    close = np.flatnonzero(flight & (d['t'] > d['t'][opened[0]]) & (d['servos'][:, 0] >= 939))
    state_events = [{'state': str(d['state'][i]), 't_s': float(d['t'][i]), 'csv_line': int(i+2)}
                    for i in np.flatnonzero(np.r_[True, d['state'][1:] != d['state'][:-1]])]
    last = np.flatnonzero(flight)[-1]
    boost = np.flatnonzero(flight & (d['t'] <= 4))
    acc_peak = boost[np.argmax(d['accelerometer'][boost, 0])]
    descent_crossings = {}
    for height in [4000, 1000, 500]:
        indices = np.flatnonzero(flight & (d['t'] > 30) & (d['height'] <= height))
        descent_crossings[str(height)] = float(d['t'][indices[0]]) if len(indices) else None
    return {'rows': len(d['t']), 'baseline': d['baseline'], 'start_boot_ms': d['start_ms'],
            'baro_peak_rise_m': float(d['height'][peak]), 'baro_peak_reported_m': float(d['barofilteredalt'][peak]),
            'baro_peak_time_s': float(d['t'][peak]), 'baro_peak_csv_line': int(peak+2),
            'gps_peak_low_confidence_rise_m': float(d['gps_height'][gps_peak]), 'gps_peak_time_s': float(d['t'][gps_peak]),
            'gps_stable_from_s': d['gps_stable_from_s'],
            'fc_velocity_peak_mps': float(np.nanmax(v)), 'fc_velocity_peak_time_s': float(d['t'][np.nanargmax(v)]),
            'boost_accelerometer_x_peak_mps2': float(d['accelerometer'][acc_peak, 0]), 'boost_accelerometer_x_peak_time_s': float(d['t'][acc_peak]),
            'baro_velocity_at_100s_mps': float(derivative(d, np.array([100.]))[0]),
            'descending_first_sample_below_height_s': descent_crossings,
            'airbrake_first_open_s': float(d['t'][opened[0]]), 'airbrake_close_s': float(d['t'][close[0]]),
            'airbrake_min_pulse_us': float(np.min(d['servos'][flight, 0])),
            'state_events': state_events, 'last_t_s': float(d['t'][last]), 'last_baro_rise_m': float(d['height'][last]),
            'last_horizontal_distance_m': float(np.hypot(d['east'][last], d['north'][last])),
            'last_east_m': float(d['east'][last]), 'last_north_m': float(d['north'][last]),
            'gyro_saturated_sample_counts_xyz': [int(v) for v in ((np.abs(d['gyro']) > 999) & flight[:, None]).sum(axis=0)],
            'maximum_flight_packet_gap_s': float(np.max(np.diff(d['t'][d['t'] >= 0]))),
            'excluded_csv_lines': [int(i+2) for i in np.flatnonzero(d['bad'])]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim', type=Path, default=DEFAULT_RUN/'telemetry.csv')
    parser.add_argument('--gs', type=Path, default=HERE.parent/'telem/ZEPH_TEST_FLIGHT_GS1.csv')
    parser.add_argument('--log', type=Path, default=DEFAULT_RUN/'OR.log')
    parser.add_argument('--output', type=Path, default=HERE)
    parser.add_argument('--show-3d', action='store_true', help='Open the 3D figure for rotation, panning and zooming')
    args = parser.parse_args()
    if not args.show_3d:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11, 'axes.spines.top': False,
                         'axes.spines.right': False, 'axes.titleweight': 'bold', 'axes.grid': True,
                         'grid.alpha': 0.18, 'savefig.dpi': 180})
    args.output.mkdir(parents=True, exist_ok=True)
    log = read_log(args.log)
    sim = read_flight(args.sim, 'OpenRocket', log['start_ms'])
    gs = read_flight(args.gs, 'GS1')
    series = [sim, gs]
    summary = {d['name']: metrics(d) for d in series}
    grid = np.arange(0, min(d['t'][-1] for d in series), 0.1)
    sh, gh = (interpolate(d['t'], d['height'], grid) for d in series)
    velocity_grid = np.arange(0, max(d['t'][-1] for d in series), 0.1)
    sv, gv = (derivative(d, velocity_grid) for d in series)
    summary['altitude_errors_sim_minus_GS1'] = {}
    for label, lo, hi in [('ascent_0_28s', 0, 28), ('descent_35_130s', 35, 130)]:
        select = (grid >= lo) & (grid <= hi) & np.isfinite(sh) & np.isfinite(gh)
        error = sh[select]-gh[select]
        summary['altitude_errors_sim_minus_GS1'][label] = {'samples': int(select.sum()), 'bias_m': float(error.mean()), 'rmse_m': float(np.sqrt(np.mean(error**2)))}
    summary['sampled_height_rise_m'] = {str(t): {d['name']: float(interpolate(d['t'], d['height'], np.array([t], dtype=float))[0])
                                                       for d in series} for t in [2, 5, 10, 14, 20, 28, 60, 100, 120]}
    truth = log['physics'].copy()
    truth[:, 0] += 1-log['start_ms']/1000  # physical t=0 corresponds to boot 1 s
    summary['simulated_ground_hit'] = {'t_s': float(truth[-1, 0]), 'velocity_mps': float(truth[-1, 2]),
        'GS1_baro_rise_at_same_time_m': float(interpolate(gs['t'], gs['height'], np.array([truth[-1, 0]]))[0])}
    summary['log_events'] = log['events']
    summary['sources_sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in [args.sim, args.gs, args.log]}
    summary['software'] = {'numpy': np.__version__, 'matplotlib': matplotlib.__version__}
    summary['method'] = {'primary_height': 'barofilteredalt minus median PRE_FLIGHT baseline in [-3,-0.2) s',
        'alignment': 'GS1 first FLIGHT telemetry row; OR exact FLIGHT transition in log; no curve fitting or time scaling',
        'GPS_altitude': 'low-confidence diagnostic only; excluded from primary altitude/3D height',
        'position': 'GPS lat/lon, fix=3, WGS84 local east/north relative to separate prelaunch origins',
        'packet_exclusion': 'whole rows with |gpsalt|>10000 m, |FC velocity|>2000 m/s, or lat/lon >0.1 degree from launch',
        'maximum_interpolated_gap_s': MAX_GAP_S, 'baro_velocity': 'one-second local linear fit, at least eight points'}
    (args.output/'metrics.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    with (args.output/'excluded_packets.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['source', 'csv_line', 't_s', 'packet_number', 'barometric_altitude_m', 'gps_altitude_m', 'latitude_deg', 'FC_integrated_velocity_mps'])
        for d in series:
            for i in np.flatnonzero(d['bad']):
                writer.writerow([d['name'], i+2, d['t'][i], d['pktnum'][i], d['barofilteredalt'][i], d['gpsalt'][i], d['lat'][i], d['accel_integrated_velo'][i]])

    def save(fig, name):
        fig.savefig(args.output/f'{name}.png', bbox_inches='tight')
        svg = args.output/f'{name}.svg'
        fig.savefig(svg, bbox_inches='tight')
        svg.write_text('\n'.join(line.rstrip() for line in svg.read_text().splitlines())+'\n')

    fig, axes = plt.subplots(2, 2, figsize=(14, 9), layout='constrained')
    fig.suptitle('Zephyrus flight comparison: barometric altitude is the primary reference', fontsize=17, fontweight='bold')
    for d in series:
        style = {'color': COLORS[d['name']], 'label': d['name'], 'linewidth': 2}
        for ax in axes[0]:
            plot_series(ax, d, d['height']/1000, **style)
        velocity = d['accel_integrated_velo'].copy(); velocity[d['bad']] = np.nan
        plot_series(axes[1, 0], d, velocity, **style)
        pulse = d['servos'][:, 0].copy(); pulse[d['bad']] = np.nan
        plot_series(axes[1, 1], d, pulse, drawstyle='steps-post', **style)
    axes[0, 0].set(title='Full vertical trajectory', xlim=(-2, 161), ylabel='Height above prelaunch barometer (km)', xlabel='Time from FC flight detection (s)')
    axes[0, 1].set(title='Ascent and apogee', xlim=(-1, 36), ylabel='Height above prelaunch barometer (km)', xlabel='Time from FC flight detection (s)')
    for d in series:
        m = summary[d['name']]
        axes[0, 1].plot(m['baro_peak_time_s'], m['baro_peak_rise_m']/1000, 'o', color=COLORS[d['name']])
        axes[0, 1].annotate(f"{d['name']}: {m['baro_peak_rise_m']/1000:.3f} km at {m['baro_peak_time_s']:.2f} s",
            xy=(m['baro_peak_time_s'], m['baro_peak_rise_m']/1000), xytext=(6, -28 if d is sim else 7), textcoords='offset points', ha='right', fontsize=10, color=COLORS[d['name']])
    axes[1, 0].set(title='FC integrated velocity (estimator output)', xlim=(0, 40), ylim=(-120, 470), ylabel='Integrated velocity (m/s)', xlabel='Time from FC flight detection (s)')
    axes[1, 1].set(title='Airbrake servo command', xlim=(10, 32), ylim=(490, 980), ylabel='PWM pulse width (µs)', xlabel='Time from FC flight detection (s)')
    axes[1, 1].axhline(525, color='0.5', ls=':', lw=1); axes[1, 1].text(10.3, 533, '525 µs = fully open', fontsize=9, color='0.4')
    axes[1, 1].text(10.3, 948, '941 µs = closed', fontsize=9, color='0.4')
    axes[0, 0].legend(loc='upper right')
    axes[0, 0].text(0.02, 0.03, 'GS1 pad baseline: 145.26 m\nOR pad baseline: 0.00 m\nGS1 recording ends before landing', transform=axes[0, 0].transAxes, fontsize=9)
    save(fig, 'trajectory-comparison')

    diagnostic, da = plt.subplots(2, 2, figsize=(14, 9), layout='constrained')
    diagnostic.suptitle('Sensor and model discrepancies', fontsize=17, fontweight='bold')
    for d in series:
        color = COLORS[d['name']]
        plot_series(da[0, 0], d, (d['height']-d['gps_height'])/1000, color=color, label=d['name'], lw=1.8)
        v = d['accel_integrated_velo'].copy(); v[d['bad']] = np.nan
        plot_series(da[0, 1], d, v, color=color, lw=1.6, label=f"{d['name']} FC estimator")
        da[0, 1].plot(velocity_grid, sv if d is sim else gv, color=color, ls='--', lw=1.4, label=f"{d['name']} barometer slope")
        acc = d['accelerometer'][:, 0].copy(); acc[d['bad']] = np.nan
        plot_series(da[1, 0], d, acc, color=color, label=d['name'], lw=1.8)
        temp = d['temp'].copy(); temp[d['bad']] = np.nan
        plot_series(da[1, 1], d, temp, color=color, label=f"{d['name']} CSV", lw=1.8)
    da[1, 1].plot(log['baro'][:, 0]-log['start_ms']/1000, log['baro'][:, 1], color=COLORS['OpenRocket'], ls='--', label='OR FC engineering value')
    da[0, 0].set(title='Barometer minus low-confidence GPS altitude', xlim=(0, 160), xlabel='Time (s)', ylabel='Difference after pad zeroing (km)')
    da[0, 0].axvspan(0, gs['gps_stable_from_s'], color='0.9'); da[0, 0].text(.03, .9, 'GS1 GPS height unsettled through 14.15 s', transform=da[0, 0].transAxes, fontsize=9)
    da[0, 1].set(title='Estimator velocity versus barometric slope', xlim=(0, 160), xlabel='Time (s)', ylabel='Velocity (m/s)')
    da[1, 0].set(title='Boost: transmitted accelerometer X', xlim=(-.2, 4), ylim=(-50, 380), xlabel='Time (s)', ylabel='Specific force, GS decode (m/s²)')
    da[1, 1].set(title='Temperature: known CSV calibration mismatch', xlim=(0, 160), xlabel='Time (s)', ylabel='Temperature (°C)')
    for ax in da.flat:
        ax.legend(fontsize=9)
    save(diagnostic, 'sensor-diagnostics')

    spatial = plt.figure(figsize=(14, 9))
    layout = spatial.add_gridspec(1, 2, width_ratios=[1.7, 1], left=.03, right=.96, bottom=.18, top=.90, wspace=.24)
    ax3d = spatial.add_subplot(layout[0, 0], projection='3d')
    ground = spatial.add_subplot(layout[0, 1])
    spatial.suptitle('3D trajectories: GPS horizontal position and barometric height', fontsize=17, fontweight='bold')
    spatial.text(.06, .045, 'Each launch is translated to its own origin. Height uses prelaunch barometer zeroing; GPS altitude is not used.\nGaps indicate invalid GPS fixes, excluded packets, or telemetry gaps over 0.5 s. Neither track is extrapolated.\nCircle = barometric apogee; square = last recorded sample.', fontsize=10, linespacing=1.6)
    positions = []
    for d in series:
        ok = d['position_ok'] & np.isfinite(d['height'])
        xyz = np.c_[d['east'], d['north'], d['height']]/1000
        xyz[~ok] = np.nan
        xyz[np.r_[False, np.diff(d['t']) > MAX_GAP_S]] = np.nan
        ax3d.plot(*xyz.T, color=COLORS[d['name']], lw=2.2, label=d['name'])
        ground.plot(xyz[:, 0], xyz[:, 1], color=COLORS[d['name']], lw=2.2, label=d['name'])
        valid = np.flatnonzero(ok)
        last, peak = valid[-1], valid[np.argmax(d['height'][valid])]
        for index, marker in [(peak, 'o'), (last, 's')]:
            ax3d.scatter(*xyz[index], color=COLORS[d['name']], s=45, marker=marker)
            ground.scatter(*xyz[index, :2], color=COLORS[d['name']], s=45, marker=marker)
        ground.annotate(f"{d['name']} last sample", xy=xyz[last, :2], xytext=(5, 6), textcoords='offset points', fontsize=9, color=COLORS[d['name']])
        positions.append(xyz)
    all_positions = np.vstack(positions)
    low, high = np.nanmin(all_positions, axis=0), np.nanmax(all_positions, axis=0)
    extents = np.maximum(high-low, 0.2)
    ax3d.set_box_aspect(extents)
    ax3d.view_init(elev=23, azim=-62)
    from matplotlib.ticker import MaxNLocator
    ax3d.xaxis.set_major_locator(MaxNLocator(5)); ax3d.yaxis.set_major_locator(MaxNLocator(4))
    ax3d.set(xlabel='East (km)', ylabel='North (km)', zlabel='Barometric height (km)')
    ground.set(title='Horizontal projection', xlabel='East from launch (km)', ylabel='North from launch (km)')
    ground.set_aspect('equal', adjustable='box')
    ground.margins(.22)
    ground.plot(0, 0, 'k^', ms=7, label='Launch origin')
    ground.legend(loc='upper right', fontsize=10)
    ax3d.legend(loc='upper left')
    save(spatial, 'trajectories-3d')
    print(json.dumps({d['name']: summary[d['name']] for d in series}, indent=2))
    print(f'Figures and metrics written to {args.output}')
    if args.show_3d:
        plt.close(fig); plt.close(diagnostic)
        plt.show()


if __name__ == '__main__':
    main()
