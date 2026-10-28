#!/usr/bin/env python3
"""Install a pinned relocatable CPython runtime beside the packaged OpenRocket JAR."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import tarfile
import tempfile
import urllib.request


def sha(path):
    result = hashlib.sha256()
    with path.open('rb') as source:
        while chunk := source.read(1024**2):
            result.update(chunk)
    return result.hexdigest()


def install(platform, destination, cache, offline=False):
    manifest = json.loads(Path(__file__).with_name('runtimes.json').read_text())
    asset = manifest['platforms'][platform]
    cache.mkdir(parents=True, exist_ok=True)
    archive = cache / asset['name']
    if not archive.exists() or sha(archive) != asset['sha256']:
        if offline:
            raise ValueError('Pinned runtime is not cached: ' + asset['name'])
        part = archive.with_suffix('.part')
        with urllib.request.urlopen(asset['url'], timeout=60) as source, part.open('wb') as target:
            shutil.copyfileobj(source, target)
        if part.stat().st_size != asset['bytes'] or sha(part) != asset['sha256']:
            part.unlink(missing_ok=True)
            raise ValueError('Python runtime checksum mismatch')
        part.replace(archive)
    if destination.exists():
        installed = json.loads((destination / 'mission-runtime.json').read_text())
        if installed.get('archiveSha256') != asset['sha256']:
            raise ValueError('Destination has another runtime; choose a fresh staging directory')
        return destination
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.mission-python-', dir=destination.parent) as temporary:
        root = Path(temporary)
        links = []
        with tarfile.open(archive, 'r:gz') as source:
            members = source.getmembers()
            if len(members) > 100_000 or sum(m.size for m in members) > 2 * 1024**3:
                raise ValueError('Runtime archive exceeds packaging bounds')
            for entry in members:
                name = PurePosixPath(entry.name)
                if name.is_absolute() or '..' in name.parts or not name.parts or name.parts[0] != 'python':
                    raise ValueError('Unsafe runtime entry')
                target = root / str(name)
                if entry.isdir():
                    target.mkdir(parents=True, exist_ok=True)
                elif entry.isfile():
                    target.parent.mkdir(parents=True, exist_ok=True)
                    with source.extractfile(entry) as data, target.open('xb') as output:
                        shutil.copyfileobj(data, output)
                    target.chmod(entry.mode & 0o755)
                elif entry.issym():
                    resolved = (target.parent / entry.linkname).resolve()
                    if not resolved.is_relative_to(root.resolve() / 'python'):
                        raise ValueError('Runtime symlink escapes its root')
                    links.append((target, entry.linkname))
                else:
                    raise ValueError('Unsupported runtime archive member')
        for target, link in links:
            target.symlink_to(link)
        runtime = root / 'python'
        # Keep the publisher's licenses and native libraries; no user site packages are consulted.
        (runtime / 'mission-runtime.json').write_text(json.dumps(dict(
            format=manifest['format'], version=1, python=manifest['python'], platform=platform,
            archiveSha256=asset['sha256'], source=asset['url']), indent=2) + '\n')
        os.replace(runtime, destination)
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--platform', required=True, choices=['macos-arm64', 'macos-x86_64', 'windows-x86_64', 'linux-x86_64'])
    parser.add_argument('--destination', type=Path, required=True)
    parser.add_argument('--cache', type=Path, default=Path(__file__).resolve().parents[2] / 'clone/openrocket/build/mission-runtime-downloads')
    parser.add_argument('--offline', action='store_true')
    args = parser.parse_args()
    print(install(args.platform, args.destination.resolve(), args.cache.resolve(), args.offline))


if __name__ == '__main__':
    main()
