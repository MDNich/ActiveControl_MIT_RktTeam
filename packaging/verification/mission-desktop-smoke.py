#!/usr/bin/env python3
"""Test the actual macOS app launcher and production Mission UI in a disposable session.
Requires a local desktop and Java 17. Exercises the native Swing interface.
No production settings are changed. Report directory must not exist.
"""
import argparse
import os
from pathlib import Path
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('app', type=Path)
    parser.add_argument('report', type=Path)
    args = parser.parse_args()
    root = args.report.resolve()
    root.mkdir(parents=True, exist_ok=False)
    here = Path(__file__).resolve().parent
    java = Path(os.environ['JAVA_HOME']) / 'bin'
    classes = root / 'agent-classes'
    classes.mkdir()
    subprocess.run([java / 'javac', '-d', classes, here / 'MissionDesktopSmokeAgent.java'], check=True)
    manifest = root / 'agent.mf'
    manifest.write_text('Premain-Class: MissionDesktopSmokeAgent\n\n')
    agent = root / 'desktop-agent.jar'
    subprocess.run([java / 'jar', 'cfm', agent, manifest, '-C', classes, '.'], check=True)
    launcher = args.app.resolve() / 'Contents/MacOS/OpenRocket_MIT'
    environment = {**os.environ, 'JAVA_TOOL_OPTIONS': f'-Dfile.encoding=UTF-8 "-javaagent:{agent}={root}"',
                   'LANG': 'en_US.UTF-8', 'LC_ALL': 'en_US.UTF-8'}
    with (root / 'launcher.log').open('w') as log:
        process = subprocess.Popen([launcher], cwd='/', env=environment, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 240
            while not (root / 'desktop-passed').exists():
                if process.poll() is not None:
                    raise RuntimeError(f'Native launcher exited with {process.returncode}; see {root / "launcher.log"}')
                if time.monotonic() > deadline:
                    raise RuntimeError(f'No Mission workspace appeared; see {root / "launcher.log"}')
                time.sleep(.5)
            if process.wait(timeout=30) != 0 or not (root / 'desktop-passed').exists():
                raise RuntimeError('Native desktop acceptance did not pass')
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
    print(f'Actual Mac application acceptance passed: {root}')


if __name__ == '__main__':
    main()
