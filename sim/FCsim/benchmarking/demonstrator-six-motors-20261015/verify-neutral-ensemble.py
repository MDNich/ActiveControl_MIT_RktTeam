import hashlib
import json
import math
import statistics
import xml.etree.ElementTree as ET
from pathlib import Path

root = Path(__file__).resolve().parent
out = root / 'neutral-tabs-no-fc'
models = Path('/Users/mdn/Developer/MIT_Rkt_Team/2026-7/MIT-RT-26_27-Work/simulation_work/generic/openrocket/OpenRocket2026/2627_official')
hashes = {3: '18c4d206047ccc1962469a1d63da8bb155abbf7adb574beafb9308e411ab1c98',
          4: '80dccac0df5df058cead0c21932ff1e76efb961168ee17e49e2e2f0a8b804788'}
motors = ['J270W', 'J415W', 'J800T', 'K550W', 'K480W', 'K700W']
all_rows = []
reference_draws = None
for iteration in [3, 4]:
    source = models / f'20267.demonstrator.iteration{iteration}.ork'
    assert hashlib.sha256(source.read_bytes()).hexdigest() == hashes[iteration]
    report = json.loads((out / f'iteration{iteration}/motor-batch-results.json').read_text())
    assert not report['input']['fcEnabled']
    xml = ET.fromstring(report['input']['xml'])
    angles = [float(e.findtext('tabangle')) for e in xml.iter('tabctrltrapezoidfinset')]
    assert angles and all(a == 0 for a in angles)
    assert not xml.findall('./simulations/simulation/extension')
    nominal = {r['motor']['designation'].removeprefix('HP-'): r for r in report['rows']}
    for motor in motors:
        n = nominal[motor]
        assert n['selected']['status'] == 'COMPLETE'
        e = json.loads((out / f'iteration{iteration}/{motor}-ensemble.json').read_text())
        assert e['curve_digest'] == n['motor']['digest']
        assert not e['fc_enabled'] and e['roll_tab_angle_deg'] == 0
        assert e['verified_runs'] == len(e['runs']) == 20
        assert e['observed_worker_threads'] == 4
        assert e['settings']['masssigma'] == e['settings']['dragsigma'] == '0.05'
        assert all(float(e['settings'][key]) == 0 for key in [
            'motorsigma', 'temperaturesigma', 'pressuresigma', 'windsigma',
            'winddirectionsigma', 'rollforcesigma', 'rolldampingsigma'])
        assert all(r['ground_hit'] for r in e['runs'])
        draws = [(r['mass_multiplier'], r['cd_multiplier']) for r in e['runs']]
        if reference_draws is None:
            reference_draws = draws
        assert draws == reference_draws
        alts = [r['apogee_m_agl'] for r in e['runs']]
        times = [r['time_to_apogee_s'] for r in e['runs']]
        assert math.isclose(statistics.stdev(alts), e['stdev_apogee_m'], rel_tol=1e-12)
        assert math.isclose(statistics.stdev(times), e['stdev_time_to_apogee_s'], rel_tol=1e-12)
        assert math.isclose(statistics.mean(alts), e['mean_apogee_m_agl'], rel_tol=1e-12)
        assert math.isclose(statistics.mean(times), e['mean_time_to_apogee_s'], rel_tol=1e-12)
        row = {
            'iteration': iteration, 'motor': motor,
            'nominal_apogee_ft_agl': n['selected']['metrics']['APOGEE'] / .3048,
            'nominal_apogee_time_s': n['selected']['metrics']['APOGEE_TIME'],
            'ensemble_mean_apogee_ft_agl': statistics.mean(alts) / .3048,
            'ensemble_stdev_apogee_ft': statistics.stdev(alts) / .3048,
            'ensemble_mean_apogee_time_s': statistics.mean(times),
            'ensemble_stdev_apogee_time_s': statistics.stdev(times),
            'trials_above_3000_ft': sum(a > 914.4 for a in alts),
            'clearance_warnings': n['ignoredIntersections'],
        }
        all_rows.append(row)
        print(f"iteration{iteration} {motor:6} nominal={row['nominal_apogee_ft_agl']:.2f} ft "
              f"ensemble={row['ensemble_mean_apogee_ft_agl']:.2f} +/- {row['ensemble_stdev_apogee_ft']:.2f} ft "
              f"time={row['ensemble_mean_apogee_time_s']:.4f} +/- {row['ensemble_stdev_apogee_time_s']:.4f} s")

summary = {
    'nominal_runs': 12, 'ensemble_runs': 240, 'runs_per_model_and_motor': 20,
    'threads': 4, 'mass_gaussian_sigma_fraction': .05, 'cd_gaussian_sigma_fraction': .05,
    'other_ensemble_variation_sigmas': 0, 'random_seed': 12345,
    'fc_enabled': False, 'roll_tab_angle_deg_both_models': 0,
    'standard_deviation': 'Sample standard deviation, denominator N-1',
    'wind': 'Saved 2 m/s mean and 0.2 m/s turbulence; turbulence trace held fixed across members',
    'mass_scope': 'Whole active vehicle, including remaining propellant; inertia scaled with mass',
    'original_model_sha256': {str(k): v for k,v in hashes.items()},
    'source_models_unchanged': True,
    'rows': all_rows,
}
(out / 'altitude-ensemble-comparison.json').write_text(json.dumps(summary, indent=2) + '\n')
print('Verified all 252 simulations, neutral tabs, FC off, identical paired draws, four workers, and unchanged sources.')
