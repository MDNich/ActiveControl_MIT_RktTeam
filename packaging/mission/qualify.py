#!/usr/bin/env python3
"""Build a native application image and verify Mission with its bundled Java/Python."""
from __future__ import annotations
import argparse
import importlib.util
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'clone/openrocket-release-24.12.RC.01'


def run(args, **kwargs):
    print(' '.join(map(str, args)), flush=True)
    subprocess.run(list(map(str, args)), check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--skip-build', action='store_true')
    parser.add_argument('--offline', action='store_true')
    args = parser.parse_args()
    target = 'windows-x86_64' if sys.platform == 'win32' else 'macos-' + ('arm64' if platform.machine() == 'arm64' else 'x86_64') if sys.platform == 'darwin' else 'linux-x86_64'
    work = SOURCE / 'build/mission-package' / target
    work.mkdir(parents=True, exist_ok=True)
    environment = {**os.environ, 'JAVA_TOOL_OPTIONS': '-Dfile.encoding=UTF-8', 'LANG': 'en_US.UTF-8', 'LC_ALL': 'en_US.UTF-8'}
    gradle = ['cmd', '/c', 'gradlew.bat'] if sys.platform == 'win32' else ['./gradlew']
    if args.offline:
        gradle.append('--offline')
    if not args.skip_build:
        run(gradle + ['shadowJar'], cwd=SOURCE, env=environment)
    edition = dict(line.split('=', 1) for line in (SOURCE / 'core/src/main/resources/build.properties').read_text().splitlines() if '=' in line and not line.startswith('#'))['build.mit.version']
    jar = SOURCE / f'build/libs/OpenRocket-MIT-v{edition}.jar'
    inputs = work / 'input'
    inputs.mkdir(exist_ok=True)
    shutil.copy2(jar, inputs / 'OpenRocket.jar')
    spec = importlib.util.spec_from_file_location('mission_runtime_bundle', Path(__file__).with_name('bundle-runtime.py'))
    runtime = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runtime)
    python_root = runtime.install(target, inputs / ('mission-python-' + target), SOURCE / 'build/mission-runtime-downloads', args.offline)
    python = python_root / ('python.exe' if sys.platform == 'win32' else 'bin/python3')
    # Headless regression runs with the actual pinned runtime, not the runner's Python.
    if not args.skip_build:
        run(gradle + [':core:test', '--tests', 'info.openrocket.core.simulation.mission.*'], cwd=SOURCE,
            env={**environment, 'MISSION_TEST_PYTHON': str(python)})
    java_home = Path(os.environ['JAVA_HOME'])
    suffix = '.exe' if sys.platform == 'win32' else ''
    image = work / 'image'
    if image.exists():
        shutil.rmtree(image)
    java_runtime = work / 'java-runtime'
    if java_runtime.exists():
        shutil.rmtree(java_runtime)
    run([java_home / ('bin/jlink' + suffix), '--module-path', java_home / 'jmods',
         '--add-modules', 'ALL-MODULE-PATH', '--strip-debug', '--no-header-files',
         '--no-man-pages', '--compress=2', '--output', java_runtime], env=environment)
    run([java_home / ('bin/jpackage' + suffix), '--type', 'app-image', '--name', 'OpenRocket_MIT',
         '--app-version', edition, '--input', inputs, '--main-jar', 'OpenRocket.jar',
         '--main-class', 'info.openrocket.swing.startup.OpenRocket', '--dest', image,
         '--runtime-image', java_runtime, '--java-options', '-Dfile.encoding=UTF-8'], env=environment)
    app_jar = next(image.rglob('OpenRocket.jar'))
    bundled_java = next(p for p in image.rglob('java' + suffix) if p.parent.name == 'bin' and 'runtime' in p.parts)
    classes = work / 'probe-classes'
    classes.mkdir(exist_ok=True)
    run([java_home / ('bin/javac' + suffix), '-encoding', 'UTF-8', '-cp', app_jar, '-d', classes,
         ROOT / 'packaging/verification/MissionPackageProbe.java',
         SOURCE / 'core/src/test/java/edu/mit/rocket_team/RTFCVerificationRocket.java'], env=environment)
    report = work / 'report'
    report.mkdir(exist_ok=True)
    run([bundled_java, '-Djava.awt.headless=true', '-Duser.home=' + str(report / 'home'),
         '-Djava.util.prefs.userRoot=' + str(report / 'preferences'), '-cp', str(app_jar) + os.pathsep + str(classes),
         'MissionPackageProbe', report], env=environment)
    print('Packaged Mission qualification passed: ' + str(report / 'report.json'))


if __name__ == '__main__':
    main()
