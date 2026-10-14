"""Plot measured roll-servo rail current with LaTeX/PGF typography."""

import ast
from pathlib import Path
import subprocess

import matplotlib
matplotlib.use("pgf")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


SOURCE = Path('/Users/mdn/Developer/MIT_Rkt_Team/Avionics2025/RT_Python_Lib/ground_station/telemetry/ZEPH_TEST_FLIGHT_GS2.csv')
OUTPUT = Path(__file__).resolve().parent / 'zephyrus-roll-servo-current'

df = pd.read_csv(SOURCE)
launch_ms = int(df.loc[df.state.eq('state.FLIGHT'), 'flight_time'].iloc[0])
t = (df.flight_time.to_numpy() - launch_ms) / 1000.0
current = np.array([ast.literal_eval(value)[4] for value in df.converter_currents])
apogee = t[df.state.eq('state.POST_APOGEE').to_numpy()][0]
main = t[df.state.eq('state.MAIN').to_numpy()][0]
drop = 2.514
assert np.all(np.diff(t) > 0)
assert current[np.searchsorted(t, drop)] == 0.000625

duration = np.maximum(0, np.minimum(t[1:], drop) - np.maximum(t[:-1], 0))
assert not np.any((duration > 0) & (np.diff(t) > 0.5))
initial_mean = np.sum(duration * current[:-1]) / np.sum(duration)
inflight = np.where(t >= 0)[0]
peak_index = inflight[np.argmax(current[inflight])]
peak = current[peak_index]

plt.rcParams.update({
    'text.usetex': True,
    'pgf.texsystem': 'pdflatex',
    'pgf.rcfonts': False,
    'font.family': 'serif',
    'font.serif': ['Computer Modern Roman'],
    'font.size': 11,
    'axes.spines.top': False,
    'axes.spines.right': False,
    'axes.grid': True,
    'grid.alpha': 0.17,
    'savefig.facecolor': 'white',
})

blue, red, green = '#2463A3', '#B13E41', '#268068'
fig, axes = plt.subplots(2, 1, figsize=(11.5, 8))
fig.subplots_adjust(left=.09, right=.965, top=.82, bottom=.15, hspace=.5)
fig.suptitle(r'\textbf{Zephyrus roll-control current}', x=.09, ha='left', y=.97, fontsize=21)
fig.text(.09, .92, r'Both roll servos combined $\cdot$ Measured 8.4 V rail current $\cdot$ GS2 telemetry',
         color='#495867', fontsize=12)
fig.text(.09, .875, 'Time zero is the first FLIGHT packet, marking FC flight detection.',
         color='#495867', fontsize=10)

# Break any long telemetry gaps rather than drawing a false continuous record.
trace = current.copy()
trace[np.r_[False, np.diff(t) > .5]] = np.nan

ax = axes[0]
ax.step(t, trace, where='post', lw=1.2, color=blue)
ax.axvspan(-5, 0, color=green, alpha=.08)
ax.axvspan(0, apogee, color=blue, alpha=.06)
for event in (0, apogee, main):
    ax.axvline(event, ls='--', lw=.8, color='#8493A0')
ax.text(apogee + 2, 4.05, r'POST\_APOGEE', color='#5D6973', fontsize=9)
ax.text(main - 2, 4.05, 'MAIN', ha='right', color='#5D6973', fontsize=9)
ax.set(title=r'\textbf{Recorded flight}', xlim=(-5, float(t[-1])), ylim=(-.09, 4.85),
       ylabel='Current (A)', xlabel='Time since FC flight detection (s)')

ax = axes[1]
ax.axvspan(0, drop, color=blue, alpha=.06)
ax.step(t, trace, where='post', lw=1.5, color=blue)
ax.axvline(0, ls='--', lw=.8, color='#8493A0')
ax.axvline(drop, ls='--', lw=1, color=red)
ax.hlines(initial_mean, 0, drop, color=green, ls=':', lw=1.5)
ax.text(.14, .91, f'First {drop:.3f} s: {initial_mean:.2f} A time-weighted mean',
        transform=ax.transAxes, fontsize=10, color=green)
ax.annotate(f'{peak:.2f} A observed peak', xy=(t[peak_index], peak), xytext=(2.9, 4.05),
            arrowprops=dict(arrowstyle='->', color=blue), color=blue, fontsize=11)
ax.text(2.69, 2.5, 'Current near zero\nby $t=2.514$ s', color=red, fontsize=11)
ax.set(title=r'\textbf{Initial actuation: close-up}', xlim=(-.5, 5), ylim=(-.09, 5.1),
       ylabel='Current (A)', xlabel='Time since FC flight detection (s)')

fig.text(.09, .045,
         'Telemetry samples shown without smoothing; small negative readings are retained.\n'
         'Observed peaks may miss brief transients. The current drop alone does not establish its physical cause.',
         color='#495867', fontsize=9)
fig.savefig(OUTPUT.with_suffix('.pdf'))
fig.savefig(OUTPUT.with_suffix('.pgf'))
subprocess.run(['pdftocairo', '-png', '-singlefile', '-r', '200',
                str(OUTPUT.with_suffix('.pdf')), str(OUTPUT)], check=True)
plt.close(fig)
print(f'Initial mean: {initial_mean:.6f} A; peak: {peak:.6f} A at {t[peak_index]:.3f} s')
print(OUTPUT.with_suffix('.pdf'))
