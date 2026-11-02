#!/usr/bin/env python3
"""Create Linux x86_64 packages from the MIT shadow JAR using a Linux JDK 17."""

import argparse
import hashlib
import json
from pathlib import Path
import platform
import re
import shutil
import subprocess
import tarfile
import zipfile


def run(*args):
    print(' '.join(map(str, args)), flush=True)
    subprocess.run(list(map(str, args)), check=True)


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True,
                        help='OpenRocket source root, or a staging tree with matching resource paths')
    parser.add_argument('--jdk', type=Path, required=True)
    parser.add_argument('--work', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if platform.system() != 'Linux' or platform.machine() != 'x86_64':
        raise SystemExit('Run this packaging step on Linux x86_64.')
    source, jdk, work, output = (p.resolve() for p in
                                  (args.source, args.jdk, args.work, args.output))
    properties = dict(line.split('=', 1) for line in
                      (source / 'core/src/main/resources/build.properties').read_text().splitlines()
                      if '=' in line and not line.lstrip().startswith('#'))
    edition = properties['build.mit.version']
    if not re.fullmatch(r'\d+\.\d+(?:\.\d+)?', edition):
        raise SystemExit('Invalid MIT version')
    jar = source / f'build/libs/OpenRocket-MIT-v{edition}.jar'
    if work.exists() or output.exists():
        raise SystemExit('Use fresh work and output directories; existing packages are preserved.')
    java_properties = subprocess.run([str(jdk / 'bin/java'), '-XshowSettings:properties', '-version'],
                                    text=True, capture_output=True, check=True).stderr
    if not re.search(r'os.arch\s*=\s*amd64\b', java_properties):
        raise SystemExit('The supplied JDK is not Linux x86_64.')
    if not re.search(r'java.specification.version\s*=\s*17\b', java_properties):
        raise SystemExit('Use a Java 17 JDK.')
    work.mkdir(parents=True)
    output.mkdir(parents=True)
    inputs = work / 'input'
    (inputs / 'native').mkdir(parents=True)
    shutil.copy2(jar, inputs / 'OpenRocket.jar')
    shutil.copy2(source / 'LICENSE.TXT', inputs / 'LICENSE.TXT')
    natives = []
    with zipfile.ZipFile(jar) as archive:
        for name in archive.namelist():
            if name.startswith('natives/linux-amd64/') and name.endswith('.so'):
                target = inputs / 'native' / Path(name).name
                target.write_bytes(archive.read(name))
                natives.append(target.name)
    if not {'libgluegen_rt.so', 'libjogl_desktop.so', 'libnativewindow_x11.so'} <= set(natives):
        raise SystemExit('Linux graphics libraries are missing from the application JAR.')
    runtime = work / 'runtime'
    run(jdk / 'bin/jlink', '--module-path', jdk / 'jmods', '--add-modules', 'ALL-MODULE-PATH',
        '--strip-debug', '--no-header-files', '--no-man-pages', '--compress=2', '--output', runtime)
    run(runtime / 'bin/java', '-version')
    icon = source / 'swing/src/main/resources/pix/icon/icon-256.png'
    name = 'OpenRocket-MIT'
    image_dir = work / 'image'
    image_args = [jdk / 'bin/jpackage', '--type', 'app-image', '--name', name,
                  '--app-version', edition, '--input', inputs, '--main-jar', 'OpenRocket.jar',
                  '--main-class', 'info.openrocket.swing.startup.OpenRocket',
                  '--runtime-image', runtime, '--icon', icon, '--dest', image_dir]
    for option in ['-Dfile.encoding=UTF-8', '--add-exports=java.base/java.lang=ALL-UNNAMED',
                   '--add-exports=java.desktop/sun.awt=ALL-UNNAMED',
                   '--add-exports=java.desktop/sun.java2d=ALL-UNNAMED',
                   '-Djava.library.path=$APPDIR/native']:
        image_args.extend(['--java-options', option])
    run(*image_args)
    app = image_dir / name
    (app / 'README.txt').write_text(
        f'OpenRocket MIT {edition} for Linux x86_64\n\n'
        'Java 17 is included. Extract this entire folder and run bin/OpenRocket-MIT.\n'
        'Requires an X11 desktop (or XWayland), fontconfig, FreeType, GTK 3 and OpenGL.\n'
        'For Ubuntu, prefer the accompanying .deb package to install dependencies.\n'
        'Portable launch: ./OpenRocket-MIT/bin/OpenRocket-MIT [design.ork]\n'
        'The bundled runtime and launcher are Linux x86_64, not ARM64.\n')
    portable = output / f'OpenRocket-MIT-{edition}-Linux-x86_64.tar.gz'
    with tarfile.open(portable, 'w:gz', compresslevel=6) as archive:
        archive.add(app, arcname=name)
    run(jdk / 'bin/jpackage', '--type', 'deb', '--name', name, '--app-version', edition,
        '--app-image', app, '--dest', output, '--vendor', 'MIT Rocket Team contributors',
        '--description', f'OpenRocket MIT {edition} (OpenRocket {properties["build.version"]})',
        '--license-file', inputs / 'LICENSE.TXT', '--icon', icon,
        '--linux-package-name', 'openrocket-mit', '--linux-menu-group', 'Science',
        '--linux-app-category', 'Science', '--linux-shortcut',
        '--linux-package-deps',
        'libx11-6, libxext6, libxi6, libxrender1, libxtst6, libfreetype6, fontconfig, '
        'libgl1, libglx0, libgtk-3-0t64 | libgtk-3-0')
    debs = list(output.glob('*.deb'))
    if len(debs) != 1:
        raise SystemExit('Expected exactly one Debian package.')
    deb = output / f'OpenRocket-MIT-{edition}-Linux-x86_64.deb'
    debs[0].rename(deb)
    artifacts = {p.name: {'bytes': p.stat().st_size, 'sha256': digest(p)}
                 for p in (portable, deb)}
    (output / 'SHA256SUMS').write_text(''.join(
        f'{meta["sha256"]}  {name}\n' for name, meta in artifacts.items()))
    (output / 'build-manifest.json').write_text(json.dumps({
        'edition': edition, 'upstream_version': properties['build.version'],
        'architecture': 'x86_64', 'jar_sha256': digest(jar),
        'runtime_release': (runtime / 'release').read_text(),
        'native_libraries': sorted(natives), 'artifacts': artifacts,
        'package_host': platform.platform(), 'app_image': str(app),
    }, indent=2) + '\n')
    print(f'Linux packages: {output}', flush=True)


if __name__ == '__main__':
    main()
