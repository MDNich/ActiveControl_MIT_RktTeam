"""Bundle headless HTTP/mDNS/SRT support into the private mission Python runtime."""
from __future__ import annotations
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tarfile
import urllib.request

SRT_SHA = 'd0a8b600fe1b4eaaf6277530e3cfc8f15b8ce4035f16af4a5eb5d4b123640cdd'


def run(arguments, **kwargs):
    subprocess.run(list(map(str, arguments)), check=True, **kwargs)


def install(python, runtime, cache, offline=False):
    requirements = Path(__file__).with_name('network-requirements.txt')
    signature = hashlib.sha256(requirements.read_bytes()).hexdigest()
    marker = runtime / 'network-runtime.json'
    if marker.is_file() and json.loads(marker.read_text()).get('requirementsSha256') == signature:
        return
    wheels = cache / ('network-wheels-' + platform.system() + '-' + platform.machine())
    wheels.mkdir(parents=True, exist_ok=True)
    if not offline:
        # Some pinned packages no longer publish Intel Mac wheels. Build a wheel
        # from the pinned source when necessary, then install only this cache.
        run([python, '-I', '-m', 'pip', 'wheel', '--no-deps',
             '-r', requirements, '--wheel-dir', wheels])
    run([python, '-I', '-m', 'pip', 'install', '--no-index', '--no-deps', '--find-links', wheels,
         '-r', requirements, '--disable-pip-version-check'])
    native = build_srt(cache, offline)
    destination = runtime / 'network-media'
    shutil.copytree(native, destination, dirs_exist_ok=True)
    library = next(p for p in destination.iterdir() if p.name.startswith(('libsrt', 'srt.dll')) and p.suffix in ('.dylib', '.dll', '.5'))
    # Verify dependencies using the packaged interpreter, with no user environment.
    run([python, '-I', '-c', 'import aiohttp,zeroconf,imageio_ffmpeg,ctypes,sys;ctypes.CDLL(sys.argv[1]);print(imageio_ffmpeg.get_ffmpeg_exe())', library])
    manifest = {'format': 'openrocket.mission-network-runtime', 'requirementsSha256': signature,
                'srtSourceSha256': SRT_SHA, 'srtEncryption': False,
                'files': {str(p.relative_to(destination)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in destination.rglob('*') if p.is_file()},
                'wheels': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in wheels.glob('*.whl')}}
    marker.write_text(json.dumps(manifest, indent=2) + '\n')


def build_srt(cache, offline):
    work = cache / ('srt-' + platform.system() + '-' + platform.machine())
    output = work / 'runtime'
    if (output / 'SRT-LICENSE').is_file():
        return output
    work.mkdir(parents=True, exist_ok=True)
    archive = cache / 'srt-1.5.4.tar.gz'
    if not archive.is_file():
        if offline:
            raise ValueError('Pinned SRT source is not cached')
        with urllib.request.urlopen('https://github.com/Haivision/srt/archive/refs/tags/v1.5.4.tar.gz', timeout=60) as source, archive.open('wb') as target:
            shutil.copyfileobj(source, target)
    if hashlib.sha256(archive.read_bytes()).hexdigest() != SRT_SHA:
        raise ValueError('Pinned SRT source digest mismatch')
    with tarfile.open(archive) as source:
        # Build files only: omit POSIX helper symlinks on Windows.
        entries = [m for m in source.getmembers() if m.isfile() or m.isdir()]
        for entry in entries:
            if not (work / entry.name).resolve().is_relative_to(work.resolve()):
                raise ValueError('SRT archive path escapes build directory')
        source.extractall(work, members=entries, filter='data')
    cmake = shutil.which('cmake')
    if not cmake and os.name == 'nt':
        vswhere = Path(os.environ['ProgramFiles(x86)']) / 'Microsoft Visual Studio/Installer/vswhere.exe'
        vs = subprocess.check_output([str(vswhere), '-latest', '-products', '*', '-requires', 'Microsoft.VisualStudio.Component.VC.Tools.x86.x64', '-property', 'installationPath'], text=True).strip()
        cmake = str(Path(vs) / 'Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe')
    if not cmake:
        raise ValueError('CMake is required to build the bundled SRT transport')
    build, source = work / 'build', work / 'srt-1.5.4'
    options = ['-G', 'Visual Studio 17 2022', '-A', 'x64'] if os.name == 'nt' else []
    run([cmake, '-S', source, '-B', build, *options, '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_POLICY_VERSION_MINIMUM=3.5',
         '-DENABLE_ENCRYPTION=OFF', '-DENABLE_APPS=OFF', '-DENABLE_SHARED=ON', '-DENABLE_STATIC=OFF'])
    run([cmake, '--build', build, '--config', 'Release', '--parallel', '4'])
    output.mkdir(exist_ok=True)
    name = 'srt.dll' if os.name == 'nt' else 'libsrt.1.5.dylib' if sys.platform == 'darwin' else 'libsrt.so.1.5'
    library = next(build.rglob(name))
    shutil.copy2(library, output / name, follow_symlinks=True)
    if sys.platform == 'darwin':
        run(['install_name_tool', '-id', '@loader_path/' + name, output / name])
        run(['codesign', '--force', '--sign', '-', output / name])
    if os.name == 'nt':
        # The private CPython runtime already provides vcruntime140. Include the C++ runtime as well.
        vswhere = Path(os.environ['ProgramFiles(x86)']) / 'Microsoft Visual Studio/Installer/vswhere.exe'
        vs = subprocess.check_output([str(vswhere), '-latest', '-products', '*', '-property', 'installationPath'], text=True).strip()
        folders = [p for p in (Path(vs) / 'VC/Redist/MSVC').rglob('Microsoft.VC143.CRT') if 'x64' in p.parts and 'onecore' not in p.parts]
        crt = sorted(folders)[-1]
        for dll in crt.glob('*.dll'):
            shutil.copy2(dll, output / dll.name)
    shutil.copy2(source / 'LICENSE', output / 'SRT-LICENSE')
    return output
