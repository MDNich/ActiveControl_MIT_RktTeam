#!/usr/bin/env python3
"""Make release downloads from the exact native image that passed qualification."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tarfile
import zipfile

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'clone/openrocket-release-24.12.RC.01'


def run(*args):
    subprocess.run(list(map(str, args)), check=True)


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--work', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    work, output = args.work.resolve(), args.output.resolve()
    reports = sorted((work / 'report').glob('*/report.json'))
    if not reports or not all(json.loads(p.read_text())['passed'] for p in reports):
        raise SystemExit('A passing packaged Mission report is required')
    props = dict(line.split('=', 1) for line in (SOURCE / 'core/src/main/resources/build.properties').read_text().splitlines()
                 if '=' in line and not line.startswith('#'))
    version = props['build.mit.version']
    target = 'Windows-x64' if os.name == 'nt' else 'macOS-' + ('arm64' if platform.machine() == 'arm64' else 'x86_64') if sys.platform == 'darwin' else 'Linux-x86_64'
    prefix = f'OpenRocket-MIT-{version}-{target}'
    output.mkdir(parents=True, exist_ok=False)
    app = work / 'image' / ('OpenRocket_MIT.app' if sys.platform == 'darwin' else 'OpenRocket_MIT')
    jar = next(app.rglob('OpenRocket.jar'))
    if sha(jar) != sha(SOURCE / f'build/libs/OpenRocket-MIT-v{version}.jar'):
        raise SystemExit('Qualified image and source JAR differ')
    python_manifest = list(app.rglob('network-runtime.json'))
    if len(python_manifest) != 1:
        raise SystemExit('Qualified network runtime is missing')
    readme = (f'OpenRocket MIT {version} — Mission Simulation BETA\n\n'
              'Mission Simulation is available from File > New Mission Simulation (Beta).\n'
              'Java 17, private Python, network discovery and simulated video support are included.\n'
              'Keep the complete application together. This release is for software testing;\n'
              'simulation results do not qualify flight hardware.\n'
              'Guide: https://github.com/MDNich/ActiveControl_MIT_RktTeam/blob/v6.5/doc/improvements/19-mission-simulation-user-guide.md\n')
    (output / f'{prefix}-README.txt').write_text(readme, encoding='utf-8')
    jpackage = Path(os.environ['JAVA_HOME']) / 'bin' / ('jpackage.exe' if os.name == 'nt' else 'jpackage')
    icon_root = SOURCE / 'swing/src/main/resources/pix/icon'
    if sys.platform == 'darwin':
        run('codesign', '--verify', '--deep', '--strict', app)
        run('ditto', '-c', '-k', '--sequesterRsrc', '--keepParent', app, output / f'{prefix}.zip')
        disk = work / 'release-disk'
        disk.mkdir()
        run('ditto', app, disk / app.name)
        (disk / 'Applications').symlink_to('/Applications')
        (disk / 'README.txt').write_text(readme + '\nDrag OpenRocket_MIT.app to Applications.\nAd-hoc signed; not Apple-notarized.\n')
        dmg = output / f'{prefix}.dmg'
        run('hdiutil', 'create', '-volname', f'OpenRocket MIT {version}', '-srcfolder', disk, '-format', 'UDZO', dmg)
        run('hdiutil', 'verify', dmg)
    elif os.name == 'nt':
        # Hosted Windows runners supply WiX 3. Preserve the existing upgrade family.
        wix = os.environ.get('WIX')
        if wix:
            os.environ['PATH'] = str(Path(wix) / 'bin') + os.pathsep + os.environ['PATH']
        if not shutil.which('candle.exe'):
            raise SystemExit('WiX 3 candle.exe is required for the Windows installer')
        shutil.copy2(app / 'OpenRocket_MIT.exe', app / 'OpenRocket.exe')
        shutil.copy2(app / 'app/OpenRocket_MIT.cfg', app / 'app/OpenRocket.cfg')
        with zipfile.ZipFile(output / f'{prefix}.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
            for path in app.rglob('*'):
                if path.is_file():
                    archive.write(path, path.relative_to(app.parent))
        run(jpackage, '--type', 'exe', '--name', 'OpenRocket_MIT', '--app-version', version,
            '--app-image', app, '--dest', output, '--vendor', 'MIT Rocket Team contributors',
            '--description', f'OpenRocket MIT {version}; Mission Simulation Beta',
            '--license-file', SOURCE / 'LICENSE.TXT', '--icon', icon_root / 'icon-windows.ico',
            '--win-per-user-install', '--win-menu', '--win-menu-group', 'OpenRocket MIT',
            '--win-shortcut', '--win-dir-chooser', '--install-dir', 'OpenRocket MIT',
            '--win-upgrade-uuid', 'a3c8caf9-967c-41d6-8efd-542171dc9e8a')
        installers = list(output.glob('*.exe'))
        if len(installers) != 1:
            raise SystemExit('Expected one Windows installer')
        installers[0].rename(output / f'{prefix}-Setup.exe')
    else:
        with tarfile.open(output / f'{prefix}.tar.gz', 'w:gz', compresslevel=6) as archive:
            archive.add(app, arcname=app.name)
        run(jpackage, '--type', 'deb', '--name', 'OpenRocket_MIT', '--app-version', version,
            '--app-image', app, '--dest', output, '--vendor', 'MIT Rocket Team contributors',
            '--description', f'OpenRocket MIT {version}; Mission Simulation Beta',
            '--license-file', SOURCE / 'LICENSE.TXT', '--icon', icon_root / 'icon-256.png',
            '--linux-package-name', 'openrocket-mit', '--linux-shortcut', '--linux-menu-group', 'Science',
            '--linux-package-deps', 'libx11-6, libxext6, libxi6, libxrender1, libxtst6, libfreetype6, fontconfig, libgl1, libgtk-3-0t64 | libgtk-3-0')
        debs = list(output.glob('*.deb'))
        if len(debs) != 1:
            raise SystemExit('Expected one Debian package')
        debs[0].rename(output / f'{prefix}.deb')
        run('dpkg-deb', '--info', output / f'{prefix}.deb')
        # One canonical standalone JAR and checksum, rather than four duplicate names.
        shutil.copy2(jar, output / f'OpenRocket-MIT-v{version}.jar')
        (output / f'OpenRocket-MIT-v{version}.jar.sha256').write_text(f'{sha(jar)}  OpenRocket-MIT-v{version}.jar\n')
    artifacts = {p.name: {'bytes': p.stat().st_size, 'sha256': sha(p)} for p in sorted(output.iterdir()) if p.is_file()}
    manifest = dict(version=version, source_revision=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                    platform=target, mission_simulation='beta', jar_sha256=sha(jar),
                    qualification=[json.loads(p.read_text()) for p in reports],
                    network_runtime=json.loads(python_manifest[0].read_text()), artifacts=artifacts)
    (output / f'{prefix}-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    (output / f'{prefix}-SHA256SUMS.txt').write_text(''.join(f'{sha(p)}  {p.name}\n' for p in sorted(output.iterdir()) if p.is_file()))


if __name__ == '__main__':
    main()
