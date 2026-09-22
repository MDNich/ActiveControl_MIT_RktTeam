#!/usr/bin/env python3
"""Recompile this OpenRocket edition and stage inputs for package-windows.ps1."""
import argparse
import hashlib
import json
import re
from pathlib import Path
import shutil
import subprocess
from zipfile import ZipFile

ARTIFACTS = [
    (
        'OpenJDK17U-jdk_x64_windows_hotspot_17.0.20.1_1.zip',
        'https://github.com/adoptium/temurin17-binaries/releases/download/jdk-17.0.20.1%2B1/OpenJDK17U-jdk_x64_windows_hotspot_17.0.20.1_1.zip',
        'e53a79c3c3d86865bd7e787903884331068e71321714ffd44f145785affc7cb0',
    ),
    (
        'wix314-binaries.zip',
        'https://github.com/wixtoolset/wix3/releases/download/wix3141rtm/wix314-binaries.zip',
        '6ac824e1642d6f7277d0ed7ea09411a508f6116ba6fae0aa5f2c7daa2ff43d31',
    ),
    (
        'gluegen-rt-2.5.0-natives-windows-amd64.jar',
        'https://jogamp.org/deployment/maven/org/jogamp/gluegen/gluegen-rt/2.5.0/gluegen-rt-2.5.0-natives-windows-amd64.jar',
        'a4f039e2fa9d616be9f26284ffd6afe5fae26d521d21f28126e5eaa073f8a438',
    ),
]


def sha256(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[2] / 'clone' / 'openrocket')
    parser.add_argument('--offline', action='store_true', help='Use only cached Gradle dependencies and downloaded build tools.')
    args = parser.parse_args()
    source = args.source.resolve()
    properties = {}
    for line in (source / 'core/src/main/resources/build.properties').read_text().splitlines():
        if '=' in line and not line.lstrip().startswith('#'):
            key, value = line.split('=', 1)
            properties[key.strip()] = value.strip()
    edition = properties.get('build.mit.version', '')
    if properties.get('build.mit.edition') != 'true' or not re.fullmatch(r'\d+\.\d+(?:\.\d+)?', edition):
        raise SystemExit('Expected a numeric MIT edition version in build.properties')
    package_version = edition if edition.count('.') == 2 else edition + '.0'
    base = source / 'build/windows-installer'
    output = base / edition
    output.mkdir(parents=True, exist_ok=True)
    downloads = base / 'downloads'
    downloads.mkdir(parents=True, exist_ok=True)
    command = [str(source / 'gradlew'), '--console=plain', '--rerun-tasks', 'shadowJar']
    if args.offline:
        command.insert(1, '--offline')
    print('Compiling the current source with all Gradle tasks rerun.', flush=True)
    with (output / 'source-build.log').open('w') as log:
        subprocess.run(command, cwd=source, stdout=log, stderr=subprocess.STDOUT, check=True)
    for name, url, expected in ARTIFACTS:
        path = downloads / name
        if not path.is_file() or sha256(path) != expected:
            if args.offline:
                raise SystemExit(f'Missing verified cached download: {name}')
            temporary = path.with_suffix(path.suffix + '.part')
            subprocess.run(['curl', '--fail', '--location', '--retry', '3', '--output', str(temporary), url], check=True)
            if sha256(temporary) != expected:
                raise SystemExit(f'Checksum mismatch: {name}')
            temporary.replace(path)
        print(f'Verified {name}', flush=True)
    payload = output / 'input'
    if payload.exists():
        shutil.rmtree(payload)  # Only this recipe's generated staging folder.
    (payload / 'native').mkdir(parents=True)
    jar = source / 'build/libs' / f'OpenRocket-MIT-v{edition}.jar'
    shutil.copy2(jar, payload / 'OpenRocket.jar')
    shutil.copy2(source / 'LICENSE.TXT', payload / 'LICENSE.TXT')
    with ZipFile(downloads / ARTIFACTS[2][0]) as archive:
        native = 'natives/windows-amd64/gluegen_rt.dll'
        (payload / 'native/gluegen_rt.dll').write_bytes(archive.read(native))
    metadata = {
        'source_revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=source, text=True).strip(),
        'source_status': subprocess.check_output(['git', 'status', '--porcelain', '--', '.'], cwd=source, text=True).strip(),
        'upstream_version': properties['build.version'],
        'mit_version': edition,
        'package_version': package_version,
        'jar_sha256': sha256(payload / 'OpenRocket.jar'),
        'downloads': [{'name': name, 'url': url, 'sha256': digest} for name, url, digest in ARTIFACTS],
    }
    (output / 'source-manifest.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(f'Windows packaging inputs: {payload}')


if __name__ == '__main__':
    main()
