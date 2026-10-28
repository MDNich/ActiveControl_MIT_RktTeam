#!/usr/bin/env python3
"""Build a native application image and verify Mission with its bundled Java/Python."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
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
    parser.add_argument('--output', type=Path, help='Separate build directory; preserve an application already in use')
    args = parser.parse_args()
    target = 'windows-x86_64' if sys.platform == 'win32' else 'macos-' + ('arm64' if platform.machine() == 'arm64' else 'x86_64') if sys.platform == 'darwin' else 'linux-x86_64'
    work = args.output.resolve() if args.output else SOURCE / 'build/mission-package' / target
    work.mkdir(parents=True, exist_ok=True)
    environment = {**os.environ, 'JAVA_TOOL_OPTIONS': '-Dfile.encoding=UTF-8', 'LANG': 'en_US.UTF-8', 'LC_ALL': 'en_US.UTF-8'}
    gradle = ['cmd', '/c', 'gradlew.bat'] if sys.platform == 'win32' else ['./gradlew']
    if args.offline:
        gradle.append('--offline')
    if not args.skip_build:
        # The upstream module transform inspects the core JAR during shadow task
        # graph construction, before a clean checkout has created that artifact.
        run(gradle + [':core:jar'], cwd=SOURCE, env=environment)
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
        run(gradle + ['--init-script', ROOT / 'packaging/mission/qualification.init.gradle', ':core:test', '--tests', 'info.openrocket.core.simulation.mission.*',
                      '--tests', 'info.openrocket.core.communication.WelcomeInfoTest'], cwd=SOURCE,
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
    icons = SOURCE / 'swing/src/main/resources/pix/icon'
    icon = icons / ('icon-windows.ico' if sys.platform == 'win32' else 'icon-macos.icns' if sys.platform == 'darwin' else 'icon-256.png')
    run([java_home / ('bin/jpackage' + suffix), '--type', 'app-image', '--name', 'OpenRocket_MIT',
         '--app-version', edition, '--input', inputs, '--main-jar', 'OpenRocket.jar',
         '--main-class', 'info.openrocket.swing.startup.OpenRocket', '--dest', image,
         '--runtime-image', java_runtime, '--icon', icon, '--java-options', '-Dfile.encoding=UTF-8'], env=environment)
    if sys.platform == 'darwin':
        import plistlib
        contents = image / 'OpenRocket_MIT.app/Contents'
        with (contents / 'Info.plist').open('rb') as stream:
            icon_name = plistlib.load(stream)['CFBundleIconFile']
        if (contents / 'Resources' / icon_name).read_bytes() != icon.read_bytes():
            raise RuntimeError('The packaged application icon does not match the OpenRocket icon')
    app_jar = next(image.rglob('OpenRocket.jar'))
    bundled_java = next(p for p in image.rglob('java' + suffix) if p.parent.name == 'bin' and 'runtime' in p.parts)
    classes = work / 'probe-classes'
    classes.mkdir(exist_ok=True)
    run([java_home / ('bin/javac' + suffix), '-encoding', 'UTF-8', '-cp', app_jar, '-d', classes,
         ROOT / 'packaging/verification/MissionPackageProbe.java',
         ROOT / 'packaging/verification/MissionPackageLauncher.java',
         SOURCE / 'core/src/test/java/edu/mit/rocket_team/RTFCVerificationRocket.java'], env=environment)
    report = work / 'report' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    report.mkdir(parents=True)
    run([bundled_java, '-Djava.awt.headless=true', '-Duser.home=' + str(report / 'home'),
         '-Djava.util.prefs.userRoot=' + str(report / 'preferences'), '-cp', str(app_jar) + os.pathsep + str(classes),
         'MissionPackageLauncher', report], cwd=Path(work.anchor), env=environment)
    print('Packaged Mission qualification passed: ' + str(report / 'report.json'))


if __name__ == '__main__':
    main()
