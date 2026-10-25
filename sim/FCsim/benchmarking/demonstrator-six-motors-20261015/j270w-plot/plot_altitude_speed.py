"""Plot OpenRocket's own ensemble time-series means and sample deviations."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, StrMethodFormatter

root = Path(__file__).resolve().parent
series = json.loads((root / 'iteration3/J270W-trajectory.json').read_text())
summary = json.loads((root / 'iteration3/J270W-ensemble.json').read_text())
previous = json.loads((root.parent / 'neutral-tabs-no-fc/iteration3/J270W-ensemble.json').read_text())
assert series['iteration'] == 3 and series['motor'] == 'J270W'
assert series['fc_enabled'] is False and series['roll_tab_angle_deg'] == 0
assert len(series['individual_runs']) == 20
assert summary['runs'] == previous['runs'], 'Rerun must reproduce the original ensemble exactly'

t = np.asarray(series['time_s'])
assert np.all(np.diff(t) > 0)
verification = {}
for raw, mean_key, sd_key in [
    ('altitude_m', 'mean_altitude_m', 'stdev_altitude_m'),
    ('speed_m_s', 'mean_speed_m_s', 'stdev_speed_m_s'),
]:
    members = np.array([np.interp(t, r['time_s'], r[raw]) for r in series['individual_runs']])
    expected_mean = np.asarray(series[mean_key])
    expected_sd = np.asarray(series[sd_key])
    assert expected_mean.shape == expected_sd.shape == t.shape
    assert np.all(np.isfinite(expected_mean)) and np.all(expected_sd >= 0)
    mean_error = float(np.max(np.abs(members.mean(axis=0) - expected_mean)))
    sd_error = float(np.max(np.abs(members.std(axis=0, ddof=1) - expected_sd)))
    # Individual histories are read back from OpenRocket's serialized archive.
    np.testing.assert_allclose(members.mean(axis=0), expected_mean, atol=2e-5, rtol=2e-6)
    np.testing.assert_allclose(members.std(axis=0, ddof=1), expected_sd, atol=2e-5, rtol=2e-6)
    verification[raw] = {'maximum_mean_difference': mean_error, 'maximum_sd_difference': sd_error}
earliest_landing = min(r['ground_hit_s'] for r in series['individual_runs'])
assert t[-1] <= earliest_landing + 1e-8

height = np.asarray(series['mean_altitude_m']) / .3048
height_sd = np.asarray(series['stdev_altitude_m']) / .3048
speed = np.asarray(series['mean_speed_m_s'])
speed_sd = np.asarray(series['stdev_speed_m_s'])
blue, orange, ink = '#1765A3', '#CF681C', '#263445'
plt.rcParams.update({
    'font.family': 'DejaVu Sans', 'font.size': 11,
    'axes.labelsize': 12, 'axes.titleweight': 'bold',
    'svg.fonttype': 'none', 'savefig.facecolor': 'white',
})
fig, altitude_ax = plt.subplots(figsize=(12.6, 7.6))
fig.subplots_adjust(left=.095, right=.90, top=.79, bottom=.17)
speed_ax = altitude_ax.twinx()
fig.text(.095, .94, 'Iteration3 demonstrator: J270W', fontsize=22, fontweight='bold', color=ink)
fig.text(.095, .89, '20 runs, mass and Cd standard deviations of 5%; roll tabs 0°, flight computer off',
         fontsize=11.5, color='#536171')

altitude_ax.fill_between(t, height-height_sd, height+height_sd, color=blue, alpha=.19, linewidth=0)
speed_ax.fill_between(t, speed-speed_sd, speed+speed_sd, color=orange, alpha=.22, linewidth=0)
altitude_line, = altitude_ax.plot(t, height, color=blue, lw=2.15, label='Altitude: mean ±1σ')
speed_line, = speed_ax.plot(t, speed, color=orange, lw=2.0, label='Flight speed: mean ±1σ')

apogee_time = summary['mean_time_to_apogee_s']
altitude_ax.axvline(apogee_time, color='#A3ACB5', lw=1.05, linestyle=(0, (4, 4)), zorder=0)
altitude_ax.annotate(f'Mean apogee time\n{apogee_time:.2f} s',
    xy=(apogee_time, .96), xycoords=('data', 'axes fraction'),
    xytext=(7, 0), textcoords='offset points', ha='left', va='top',
    fontsize=10, color='#596573')

altitude_ax.set_xlabel('Time after launch (s)', labelpad=10, color=ink)
altitude_ax.set_ylabel('Altitude above launch level (ft)', color=blue, labelpad=11)
speed_ax.set_ylabel('Flight speed (m/s)', color=orange, labelpad=12)
altitude_ax.tick_params(axis='y', colors=blue)
speed_ax.tick_params(axis='y', colors=orange)
altitude_ax.tick_params(axis='x', colors=ink)
altitude_ax.set_xlim(0, float(t[-1]))
altitude_ax.set_ylim(min(0, float(np.min(height-height_sd))*1.08),
                     np.ceil(np.max(height+height_sd)/250)*250)
speed_ax.set_ylim(min(0, float(np.min(speed-speed_sd))*1.08),
                  np.ceil(np.max(speed+speed_sd)/20)*20)
altitude_ax.xaxis.set_major_locator(MultipleLocator(10))
altitude_ax.xaxis.set_minor_locator(MultipleLocator(5))
altitude_ax.yaxis.set_major_locator(MultipleLocator(250))
altitude_ax.yaxis.set_major_formatter(StrMethodFormatter('{x:,.0f}'))
speed_ax.yaxis.set_major_locator(MultipleLocator(20))
altitude_ax.grid(True, axis='both', color='#E1E6EB', linewidth=.8)
altitude_ax.set_axisbelow(True)
for ax in [altitude_ax, speed_ax]:
    ax.spines['top'].set_visible(False)
    for edge in ['left', 'right', 'bottom']:
        ax.spines[edge].set_color('#B8C2CC')

fig.legend(handles=[altitude_line, speed_line], loc='upper right',
           bbox_to_anchor=(.89, .855), ncol=2, frameon=False, fontsize=11)
fig.text(.095, .075, 'Shaded bands show pointwise sample standard deviation across the 20 flights.',
         fontsize=10, color='#536171')
fig.text(.095, .045, f'OpenRocket common time interval: 0–{t[-1]:.2f} s, ending before the first touchdown. Speed is total velocity magnitude.',
         fontsize=9.5, color='#536171')
png = root / 'iteration3-J270W-altitude-speed.png'
svg = root / 'iteration3-J270W-altitude-speed.svg'
fig.savefig(png, dpi=180)
fig.savefig(svg)
plt.close(fig)
verification.update({'same_20_runs_reproduced': True, 'samples': len(t),
                     'common_time_end_s': float(t[-1]), 'earliest_landing_s': earliest_landing,
                     'altitude_unit': 'ft AGL', 'speed_unit': 'm/s', 'band': 'pointwise mean ± sample SD (N-1)'})
(root / 'plot-verification.json').write_text(json.dumps(verification, indent=2)+'\n')
print(json.dumps(verification, indent=2))
print(png)
print(svg)
