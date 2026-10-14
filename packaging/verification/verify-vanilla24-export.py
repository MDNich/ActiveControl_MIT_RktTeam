#!/usr/bin/env python3
"""Build export fixtures, then load/save/rerun them using the official 24.12 JAR.

Build shadowJar first. Optional positional arguments are additional MIT .ork files;
their originals are read only. All generated files stay in build/vanilla-24.12.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import urllib.request

OFFICIAL_URL = "https://github.com/openrocket/openrocket/releases/download/release-24.12/OpenRocket-24.12.jar"
OFFICIAL_SHA256 = "4959b72f52f5f607941e9722abbb7b7f0c4a38ebbbf84204a329db9f31c4f897"


def sha256(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("rockets", nargs="*", type=Path)
    args = parser.parse_args()
    helper = Path(__file__).resolve().parent
    repository = helper.parents[1]
    source = repository / "clone/openrocket-release-24.12.RC.01"
    base = source / "build/vanilla-24.12"
    for name in ["mit-probe", "official-probe", "exports", "preferences"]:
        (base / name).mkdir(parents=True, exist_ok=True)
    java_bin = Path(os.environ["JAVA_HOME"]) / "bin" if "JAVA_HOME" in os.environ else Path(shutil.which("java")).parent
    official = base / "OpenRocket-24.12.jar"
    if not official.exists():
        temporary = official.with_suffix(".download")
        try:
            urllib.request.urlretrieve(OFFICIAL_URL, temporary)
            assert sha256(temporary) == OFFICIAL_SHA256, "Official download checksum mismatch"
            temporary.replace(official)
        finally:
            temporary.unlink(missing_ok=True)
    assert sha256(official) == OFFICIAL_SHA256, "Cached official JAR checksum mismatch"
    properties = dict(line.split("=", 1) for line in (source / "core/src/main/resources/build.properties")
                      .read_text().splitlines() if "=" in line and not line.startswith("#"))
    mit = source / "build/libs" / ("OpenRocket-MIT-v" + properties["build.mit.version"].strip() + ".jar")
    assert mit.is_file(), "Build shadowJar first"
    inputs = [p.resolve() for p in args.rockets]
    before = {str(p): sha256(p) for p in inputs}
    probe = helper / "Vanilla24CompatibilityProbe.java"
    subprocess.run([str(java_bin / "javac"), "-cp", str(official), "-d", str(base / "official-probe"), str(probe)], check=True)
    subprocess.run([str(java_bin / "javac"), "-cp", str(mit), "-d", str(base / "mit-probe"), str(probe),
                    str(helper / "VanillaExportFixture.java")], check=True)
    common = [str(java_bin / "java"), "-Xmx1g", "-Djava.awt.headless=true",
              "-Djava.util.prefs.userRoot=" + str(base / "preferences")]
    with (base / "export.log").open("w") as log:
        subprocess.run(common + ["-cp", str(base / "mit-probe") + os.pathsep + str(mit),
                                 "VanillaExportFixture", str(base / "exports"), *map(str, inputs)],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    outputs = [base / "exports" / (p.stem + "-vanilla-24.12.ork") for p in inputs]
    outputs.append(base / "exports/mixed-features-vanilla-24.12.ork")
    with (base / "official-verification.log").open("w") as log:
        subprocess.run(common + ["-cp", str(base / "official-probe") + os.pathsep + str(official),
                                 "Vanilla24CompatibilityProbe", *map(str, outputs)],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    assert before == {str(p): sha256(p) for p in inputs}, "Source design changed"
    results = [line for line in (base / "official-verification.log").read_text().splitlines()
               if line.startswith("VANILLA24_")]
    report = {"official_url": OFFICIAL_URL, "official_sha256": OFFICIAL_SHA256,
              "mit_jar_sha256": sha256(mit), "originals_unchanged": before, "results": results,
              "exports": {str(p): sha256(p) for p in outputs}}
    (base / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
    print("\n".join(results))
    print("Verification report:", base / "verification.json")


if __name__ == "__main__":
    main()
