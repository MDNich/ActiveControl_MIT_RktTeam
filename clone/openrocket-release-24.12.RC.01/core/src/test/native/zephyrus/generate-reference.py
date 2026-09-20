#!/usr/bin/env python3
"""Regenerate Java test oracles from the selected, unchanged FC firmware library."""
import argparse
from pathlib import Path
import subprocess
import tempfile
p=argparse.ArgumentParser()
p.add_argument('firmware',type=Path,help='RT_Firmware_Libs checkout')
a=p.parse_args()
here=Path(__file__).resolve().parent
output=here.parents[1]/'resources/zephyrus'
with tempfile.TemporaryDirectory() as work:
    executable=Path(work)/'reference'
    subprocess.run(['c++','-std=c++17','-ffp-contract=off','-I'+str(here),'-I'+str(a.firmware),str(here/'airbrakes-reference.cpp'),str(a.firmware/'airbrakes.cpp'),str(a.firmware/'rollcontrol.cpp'),'-o',str(executable)],check=True)
    for case,name in enumerate(['integer','fractional','partial','late']):
        with (output/f'airbrakes-{name}.csv').open('w') as stream:
            subprocess.run([str(executable),str(case)],stdout=stream,check=True)
