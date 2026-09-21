#!/usr/bin/env python3
"""Package the current shadow JAR as one universal Mac app with two bundled Java runtimes."""
import argparse
import hashlib
import json
from pathlib import Path
import plistlib
import shutil
import subprocess


def run(*args):
    print(' '.join(map(str, args)), flush=True)
    subprocess.run(list(map(str, args)), check=True)


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[2] / 'clone/openrocket')
    parser.add_argument('--arm-jdk', type=Path, required=True)
    parser.add_argument('--intel-jdk', type=Path, required=True)
    args = parser.parse_args()
    source = args.source.resolve()
    properties = dict(line.split('=', 1) for line in (source / 'core/src/main/resources/build.properties').read_text().splitlines()
                      if '=' in line and not line.lstrip().startswith('#'))
    edition = properties['build.mit.version']
    jar = source / f'build/libs/OpenRocket-MIT-{edition}.jar'
    work = source / f'build/macos-installer/{edition}'
    app = work / 'image/OpenRocket_MIT.app'
    if app.exists():
        raise SystemExit(f'Use a fresh application staging path: {app}')
    contents = app / 'Contents'
    for name in ('MacOS', 'Resources', 'app'):
        (contents / name).mkdir(parents=True, exist_ok=True)
    shutil.copy2(jar, contents / 'app/OpenRocket.jar')
    shutil.copy2(source / 'LICENSE.TXT', contents / 'app/LICENSE.TXT')
    shutil.copy2(source / 'swing/src/main/resources/pix/icon/icon-macos.icns', contents / 'Resources/app.icns')
    runtimes = {}
    for arch, jdk in [('arm64', args.arm_jdk), ('x86_64', args.intel_jdk)]:
        jdk = jdk.resolve()
        runtime = contents / f'runtime-{arch}'
        run('lipo', jdk / 'bin/java', '-verify_arch', arch)
        run(jdk / 'bin/jlink', '--module-path', jdk / 'jmods', '--add-modules', 'ALL-MODULE-PATH',
            '--strip-debug', '--no-header-files', '--no-man-pages', '--compress=2', '--output', runtime)
        run(runtime / 'bin/java', '-version')
        runtimes[arch] = (runtime / 'release').read_text()
    launcher = contents / 'MacOS/OpenRocket_MIT'
    run('xcrun', 'clang', '-arch', 'arm64', '-arch', 'x86_64', '-mmacosx-version-min=11.0',
        '-O2', '-Wall', '-Wextra', '-Werror', Path(__file__).with_name('launcher.c'), '-o', launcher)
    run('lipo', launcher, '-verify_arch', 'arm64', 'x86_64')
    info = {
        'CFBundleDevelopmentRegion': 'en', 'CFBundleExecutable': 'OpenRocket_MIT',
        'CFBundleIdentifier': 'edu.mit.rocket-team.openrocket', 'CFBundleInfoDictionaryVersion': '6.0',
        'CFBundleName': 'OpenRocket MIT', 'CFBundleDisplayName': 'OpenRocket MIT', 'CFBundlePackageType': 'APPL',
        'CFBundleShortVersionString': edition, 'CFBundleVersion': edition,
        'CFBundleIconFile': 'app.icns', 'LSMinimumSystemVersion': '11.0',
        'NSHighResolutionCapable': True, 'NSRequiresAquaSystemAppearance': False,
        'CFBundleDocumentTypes': [{'CFBundleTypeExtensions': ['ork'], 'CFBundleTypeIconFile': 'app.icns',
                                  'CFBundleTypeName': 'OpenRocket Design', 'CFBundleTypeRole': 'Editor',
                                  'LSHandlerRank': 'Alternate'}],
    }
    (contents / 'Info.plist').write_bytes(plistlib.dumps(info))
    (contents / 'PkgInfo').write_bytes(b'APPL????')
    # Local ad-hoc signature only: no Developer ID certificate or notarization is implied.
    run('codesign', '--force', '--deep', '--sign', '-', app)
    run('codesign', '--verify', '--deep', '--strict', '--verbose=2', app)
    output = work / 'output'
    output.mkdir(exist_ok=True)
    archive = output / f'OpenRocket-MIT-{edition}-macOS-Universal.zip'
    run('ditto', '-c', '-k', '--sequesterRsrc', '--keepParent', app, archive)
    disk_source = work / 'disk'
    disk_source.mkdir(exist_ok=True)
    run('ditto', app, disk_source / app.name)
    (disk_source / 'Applications').symlink_to('/Applications')
    (disk_source / 'README.txt').write_text(
        f'OpenRocket MIT {edition}\n\nDrag OpenRocket_MIT.app to Applications.\n'
        'Universal: Apple Silicon and Intel; Java 17 is included.\n'
        'This local build is ad-hoc signed and is not Apple-notarized.\n')
    dmg = output / f'OpenRocket-MIT-{edition}-macOS-Universal.dmg'
    run('hdiutil', 'create', '-volname', f'OpenRocket MIT {edition}', '-srcfolder', disk_source,
        '-format', 'UDZO', '-ov', dmg)
    run('hdiutil', 'verify', dmg)
    artifacts = {p.name: {'bytes': p.stat().st_size, 'sha256': digest(p)} for p in (archive, dmg)}
    (output / 'build-manifest.json').write_text(json.dumps({
        'edition': edition, 'upstream_version': properties['build.version'], 'jar_sha256': digest(jar),
        'architectures': ['arm64', 'x86_64'], 'runtimes': runtimes,
        'signing': 'ad-hoc', 'notarized': False, 'artifacts': artifacts,
    }, indent=2) + '\n')
    print(f'Universal Mac packages: {output}', flush=True)


if __name__ == '__main__':
    main()
