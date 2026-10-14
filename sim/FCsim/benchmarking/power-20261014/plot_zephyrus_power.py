"""Read-only launch-power analysis; export a LaTeX/PGF diagnostic figure."""
import ast
import json
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use("pgf")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

SOURCE = Path('/Users/mdn/Developer/MIT_Rkt_Team/Avionics2025/RT_Python_Lib/ground_station/telemetry')
OUTPUT = Path('/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/benchmarking/power-20261014')
df = pd.read_csv(SOURCE / 'ZEPH_TEST_FLIGHT_GS2.csv')
voltage = np.array([ast.literal_eval(x) for x in df.converter_voltages])
current = np.array([ast.literal_eval(x) for x in df.converter_currents])
servo = np.array([ast.literal_eval(x) for x in df.servos_deg])
power = voltage * current
launch = int(df.loc[df.state.eq('state.FLIGHT'), 'flight_time'].iloc[0])
t = (df.flight_time.to_numpy() - launch) / 1000.0
apogee = float(t[df.state.eq('state.POST_APOGEE').to_numpy()][0])
main = float(t[df.state.eq('state.MAIN').to_numpy()][0])
drop = 2.514
assert launch == 747766 and apogee == 28.956
assert np.all(np.diff(t) > 0)
assert current[np.searchsorted(t, drop), 4] == 0.000625

def weights(lo, hi):
    dt = np.maximum(0, np.minimum(t[1:], hi) - np.maximum(t[:-1], lo))
    dt[np.diff(t) > 0.5] = 0
    return dt

def summary(channel, lo, hi):
    dt = weights(lo, hi)
    span = float(dt.sum())
    m = (t >= lo) & (t < hi)
    energy = float(dt @ power[:-1, channel])
    return dict(start_s=lo, end_s=hi, covered_s=span,
                mean_voltage_V=float(dt @ voltage[:-1, channel] / span),
                mean_current_A=float(dt @ current[:-1, channel] / span),
                mean_power_W=energy / span, peak_power_W=float(power[m, channel].max()),
                peak_current_A=float(current[m, channel].max()), energy_J=energy,
                energy_Wh=energy / 3600, samples=int(m.sum()))

results = {
    'VTX_pad': summary(5, -50, 0),
    'FC_pad': summary(1, -50, 0),
    'roll_initial_draw': summary(4, 0, drop),
    'roll_ascent': summary(4, 0, apogee),
    'roll_after_drop': summary(4, drop, apogee),
    'FC_ascent': summary(1, 0, apogee),
    'FC_recorded_flight': summary(1, 0, float(t[-1])),
    'roll_pad': summary(4, -50, 0),
}
Path('/tmp/zephyrus-power-summary.json').write_text(json.dumps(results, indent=2))

plt.rcParams.update({'text.usetex': True, 'pgf.texsystem': 'pdflatex',
                     'pgf.rcfonts': False, 'font.family': 'serif',
                     'font.serif': ['Computer Modern Roman'], 'font.size': 10,
                     'axes.titlesize': 13, 'axes.titleweight': 'bold',
                     'axes.spines.top': False, 'axes.spines.right': False,
                     'axes.grid': True, 'grid.alpha': .16,
                     'savefig.facecolor': 'white'})
fig, axs = plt.subplots(2, 2, figsize=(13, 8.4))
fig.subplots_adjust(left=.075, right=.91, top=.86, bottom=.12, hspace=.52, wspace=.29)
fig.suptitle(r'\textbf{Zephyrus launch: measured electrical power}', x=.075, ha='left', y=.97,
             fontsize=20, fontweight='bold')
fig.text(.075, .92, r'GS2 telemetry $\cdot$ Time zero = first FLIGHT packet $\cdot$ Power = measured rail voltage $\times$ current',
         fontsize=11, color='#495867')
blue, red, green = '#2463A3', '#B13E41', '#268068'

for ax, channel, title in [(axs[0, 0], 5, r'\textbf{VTX $\cdot$ 28 V rail}'),
                            (axs[0, 1], 1, r'\textbf{FC $\cdot$ 3.3 V rail}')]:
    m = (t >= -50)
    ax.axvspan(-50, 0, color=green, alpha=.08)
    ax.axvspan(0, apogee, color=blue, alpha=.06)
    ax.step(t[m], power[m, channel], where='post', color=blue,
            lw=.65, alpha=.38 if channel == 1 else .85, label='Telemetry')
    if channel == 1:
        edges = np.arange(-50, int(t[-1]))
        means = [summary(channel, float(x), float(x+1))['mean_power_W'] for x in edges]
        ax.plot(edges+.5, means, color=blue, lw=1.5, label='1 s time-weighted mean')
        ax.legend(loc='upper right', fontsize=8, framealpha=.95)
    for event in [0, apogee, main]:
        ax.axvline(event, color='#8493A0', ls='--', lw=.8)
    ax.set(title=title, xlim=(-50, 160), xlabel='Time since FC flight detection (s)', ylabel='Power (W)')
    ax.set_ylim(0, 27 if channel == 5 else 3.25)
    ax.text(-46, 25.2 if channel == 5 else 3.02, 'Pad', color=green, fontsize=9)
    if channel == 5:
        ax.annotate('Pad mean: 3.39 W', xy=(-25, 3.39), xytext=(-45, 11),
                    arrowprops=dict(arrowstyle='-', color=green), color=green)
        ax.text(apogee + 2, 24.4, r'POST\_APOGEE', fontsize=8, color='#5D6973')

ax = axs[1, 0]
m = (t >= -.5) & (t <= 5)
ax.axvspan(0, drop, color=red, alpha=.08)
ax.step(t[m], power[m, 4], where='post', color=red, lw=1.45)
ax.axvline(drop, color=red, ls='--', lw=1)
ax.annotate('36.98 W / 4.41 A\nobserved peak', xy=(1.11, 36.976632), xytext=(3.0, 35.2),
            arrowprops=dict(arrowstyle='->', color=red), color=red, fontsize=10)
ax.text(2.67, 16, 'Current collapses\nat t = 2.514 s', color=red)
ax.text(.03, .91, r'First 2.514 s: 8.80 W mean $\cdot$ 22.1 J', transform=ax.transAxes, fontsize=9)
ax.set(title=r'\textbf{Both roll servos $\cdot$ 8.4 V rail}', xlim=(-.5, 5), ylim=(-1.5, 43),
       xlabel='Time since FC flight detection (s)', ylabel='Power (W)')

ax = axs[1, 1]
m = (t >= -.5) & (t <= 30)
ax.step(t[m], servo[m, 2] + .5, where='post', color=blue, lw=1.1,
        label='Command (servo 2, trim removed)')
ax.axvline(drop, color=red, ls='--', lw=1)
ax.axvline(apogee, color='#8493A0', ls='--', lw=.8)
ax.set(title=r'\textbf{Commands and supply remain present}', xlim=(-.5, 30), ylim=(-23, 24),
       xlabel='Time since FC flight detection (s)', ylabel=r'Commanded roll angle ($^\circ$)')
other = ax.twinx()
other.grid(False)
other.step(t[m], voltage[m, 4], where='post', color=green, lw=1.2, label='Measured 8.4 V rail')
other.set(ylim=(0, 10), ylabel='Rail voltage (V)')
other.tick_params(axis='y', colors=green)
other.yaxis.label.set_color(green)
ax.legend(ax.lines[:1] + other.lines, [ax.lines[0].get_label(), other.lines[0].get_label()],
          loc='lower right', fontsize=8, framealpha=.95)
fig.text(.075, .035,
         'Converter-output electrical power; converter losses are excluded. Roll channels are combined, not per-servo.\n'
         'The current collapse does not establish its physical cause. Telemetry cannot resolve all short current peaks.',
         fontsize=9, color='#495867')
OUTPUT.mkdir(parents=True, exist_ok=True)
figure_path = OUTPUT / 'zephyrus-power-consumption'
fig.savefig(figure_path.with_suffix('.pdf'))
fig.savefig(figure_path.with_suffix('.pgf'))
subprocess.run(['pdftocairo', '-png', '-singlefile', '-r', '200',
                str(figure_path.with_suffix('.pdf')), str(figure_path)], check=True)
plt.close(fig)
print(json.dumps(results, indent=2))
