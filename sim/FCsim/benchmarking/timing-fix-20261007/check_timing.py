#!/usr/bin/env python3
"""Independent checks of saved Zephyrus action logs, packet exports and ORK results."""
from pathlib import Path
import collections, hashlib, json, re, subprocess, sys, zipfile
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parent
SUPPORT=ROOT.parent.parent
PROFILES={
    'ideal': (0,0,0,0),
    'default': (3000,0,0,0),
    'overrun': (3000,10000,0,7000),
    'jitter': (3000,3000,9000,6371),
}
def span(values):
    return {'min':min(values),'max':max(values)} if values else None
def collect(path):
    actions=collections.defaultdict(list)
    previous=-1
    with path.open() as stream:
        for line in stream:
            if not line.startswith('ZEPHYRUS '): continue
            fields=dict(re.findall(r'(\w+)=([^\s]+)',line))
            us=int(fields['boot_us'])
            assert us>=previous, 'Virtual clock went backward'
            previous=us
            actions[fields['action']].append((us,fields))
    return actions
results={}
for name,(sensor,work,jitter,phase) in PROFILES.items():
    run_dirs=list((ROOT/name).glob('zephyrus-*'))
    assert len(run_dirs)==1, (name,run_dirs)
    directory=run_dirs[0]
    a=collect(directory/'OR.log')
    metadata=dict(line.split('=',1) for line in (directory/'metadata.txt').read_text().splitlines() if '=' in line)
    assert metadata['completion']=='complete'
    begin=[t for t,_ in a['fc.loop_begin']]
    intervals=[y-x for x,y in zip(begin,begin[1:])]
    executions=[]; overruns=[]
    for i,(end,f) in enumerate(a['timing.loop']):
        start=int(f['start_us']); execution=int(f['execution_us'])
        assert start==begin[i] and end==start+execution
        assert sensor+work<=execution<=sensor+work+jitter
        deadline=(start//1000+10)*1000
        assert int(f['overrun_us'])==max(0,end-deadline)
        assert int(f['next_start_us'])==max(end,deadline)
        if i+1<len(begin): assert begin[i+1]==int(f['next_start_us'])
        executions.append(execution); overruns.append(int(f['overrun_us']))
    pwm=[t for t,_ in a['pwm.latch'] if t>0]
    assert pwm[0]==(phase or 20000)
    assert all(y-x==20000 for x,y in zip(pwm,pwm[1:]))
    gps=[t for t,_ in a['gps.fix_available']]
    assert all(y-x==100000 for x,y in zip(gps,gps[1:]))
    ages=[int(f['age_us']) for _,f in a['sensors.deliver']]
    assert all(sensor<=age<=sensor+2500 for age in ages)
    power=[t for t,_ in a['power.command']]
    power_intervals=[y-x for x,y in zip(power,power[1:])]
    assert power[0]//1000>100
    assert all(y//1000-x//1000>100 for x,y in zip(power,power[1:]))
    if name in ('ideal','default'): assert set(power_intervals)=={110000} and set(intervals)=={10000}
    if name=='overrun': assert set(intervals)=={13000} and all(overruns)
    if name=='jitter': assert any(overruns) and any(n==0 for n in overruns)
    firing={}; pulses={}
    for us,f in a['pyro.fire']: firing[int(f['channel'])]=us
    for us,f in a['pyro.off']:
        channel=int(f['channel']); start=firing[channel]
        assert us//1000-start//1000>250
        pulses[channel]=us-start
    assert len(pulses)==6
    assert a['fc.transition'][-1][1]['to']=='MAIN'
    command=[sys.executable,str(SUPPORT/'verify-telemetry.py'),str(directory)]
    if not jitter: command+=['--expected-period-ms', '52' if name=='overrun' else '60']
    verification=json.loads(subprocess.check_output(command,text=True))
    (directory/'telemetry-verification.json').write_text(json.dumps(verification,indent=2)+'\n')
    ork=ROOT/f'zephyrus-{name}.ork'
    with zipfile.ZipFile(ork) as z: root=ET.fromstring(z.read('rocket.ork'))
    flight=root.find('./simulations/simulation/flightdata')
    assert flight is not None and flight.find('databranch') is not None
    result={
        'loop_starts':len(begin),'completed_loops':len(executions),'overruns':sum(n>0 for n in overruns),
        'loop_period_us':span(intervals),'execution_us':span(executions),
        'sensor_age_us':span(ages),
        'pwm_sample_age_us':span([int(f['sample_age_us']) for _,f in a['pwm.latency']]),
        'power_commands':len(power),'first_power_us':power[0],'power_period_us':span(power_intervals),
        'pwm_period_us':20000,'gps_fix_period_us':100000,
        'pyro_pulse_us_by_channel':pulses,'transmitted_packets':verification['transmitted_packets'],
        'flight':{k:float(v) for k,v in flight.attrib.items()},
        'run_directory':str(directory.relative_to(ROOT)),'ork':ork.name,
        'ork_sha256':hashlib.sha256(ork.read_bytes()).hexdigest(),
    }
    results[name]=result
# The pre-change flight is useful for the power bug count only (its wind seed was not pinned).
old_dir=next((ROOT/'baseline').glob('zephyrus-*'))
old=collect(old_dir/'OR.log')
power=[t for t,_ in old['power.command']]
results['baseline_unpaired']={'power_commands':len(power),'first_power_us':power[0],
    'power_period_us':span([y-x for x,y in zip(power,power[1:])]),'run_directory':str(old_dir.relative_to(ROOT))}
repeat_dir=next((ROOT/'default-repeat').glob('zephyrus-*'))
first_dir=ROOT/results['default']['run_directory']
for filename in ('packets.bin','transmitted-packets.bin','telemetry.csv','transmitted-telemetry.csv'):
    assert (first_dir/filename).read_bytes()==(repeat_dir/filename).read_bytes(), f'Nonrepeatable {filename}'
with zipfile.ZipFile(ROOT/'zephyrus-default-repeat.ork') as z:
    repeated_root=ET.fromstring(z.read('rocket.ork'))
with zipfile.ZipFile(ROOT/'zephyrus-default.ork') as z:
    first_root=ET.fromstring(z.read('rocket.ork'))
repeated_flight=repeated_root.find('./simulations/simulation/flightdata')
first_flight=first_root.find('./simulations/simulation/flightdata')
assert repeated_flight.attrib==first_flight.attrib
first_branches=first_flight.findall('databranch'); repeated_branches=repeated_flight.findall('databranch')
assert len(first_branches)==len(repeated_branches)
for first_branch,repeat_branch in zip(first_branches,repeated_branches):
    assert first_branch.attrib==repeat_branch.attrib
    names=first_branch.get('types').split(',')
    # Desktop computation time is a wall-clock diagnostic, not a simulated state variable.
    keep=[i for i,name in enumerate(names) if name!='Computation time']
    first_points=first_branch.findall('datapoint'); repeat_points=repeat_branch.findall('datapoint')
    assert len(first_points)==len(repeat_points)
    for x,y in zip(first_points,repeat_points):
        left=x.text.split(','); right=y.text.split(',')
        assert all(left[i]==right[i] for i in keep)
assert any(e.get('key')=='sensorReadUs' and e.text=='3000' for e in repeated_root.findall('./simulations/simulation/extension/entry'))
results['repeat']={'result':'PASS','identical_telemetry_bytes':True,'identical_simulated_state_samples':True,'excluded_wall_clock_column':'Computation time','explicit_settings_persisted':True,
                   'run_directory':str(repeat_dir.relative_to(ROOT)),'ork':'zephyrus-default-repeat.ork'}
source=SUPPORT/'examples/zephy_testlaunch-java-fc.ork'
output={'result':'PASS','source_ork':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'simulation_and_wind_seed':20261007,'timing_jitter_seed':42,'profiles':results}
(ROOT/'timing-results.json').write_text(json.dumps(output,indent=2)+'\n')
print(json.dumps(output,indent=2))
