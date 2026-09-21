# Windows installer for OpenRocket MIT

This recipe compiles the current source and packages a Windows x64 installer with Java 17 included. It reads the MIT edition from `build.properties`; edition 6.2 becomes Windows package version 6.2.0. Output is unsigned and uses OpenJDK `jpackage` with WiX, without requiring install4j.

## Build the source and prepare inputs

Requirements: Python 3.11+, Java 17, and curl. From the parent repository:

```sh
JAVA_HOME=/Library/Java/JavaVirtualMachines/jdk-17.jdk/Contents/Home \
python3 packaging/windows/prepare-windows.py --offline
```

Omit `--offline` when the pinned downloads or Gradle dependencies are not cached. Use `--source /path/to/openrocket` for another checkout. All source compilation tasks are rerun. Versioned staging, build log and provenance are written to `clone/openrocket/build/windows-installer/6.2/`; shared verified downloads remain in `build/windows-installer/downloads/`.

Pinned tools: Temurin Windows x64 17.0.20.1+1, WiX 3.14.1, GlueGen 2.5.0 Windows amd64. URLs and SHA-256 checksums are recorded in `prepare-windows.py` and the generated source manifest.

## Package on Windows

Use a new, local work directory and a Windows-visible source path. For this machine's Parallels VM:

```powershell
$repo = '\\Mac\Home\Developer\ActiveControl_MIT_RktTeam'
$source = "$repo\clone\openrocket-release-24.12.RC.01"
$work = Join-Path $env:TEMP ('OpenRocket-MIT-build-' + [guid]::NewGuid().ToString('N'))
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$repo\packaging\windows\package-windows.ps1" `
    -SourceRoot $source -WorkDirectory $work `
    -OutputDirectory "$source\build\windows-installer\6.2\output"
```

The output includes `OpenRocket-MIT-6.2-Windows-x64-Setup.exe`, its checksum, packaging log and manifest. Copy deliverables outside the Gradle build folder before cleaning it.

The installer uses a separate MIT installation directory and upgrade identity, requests per-user installation, and supplies Start menu/optional desktop shortcuts. It does not change `.ork` associations. A bundled Java launcher and `OpenRocket.exe` alias support existing updater lookups; automatic updates remain untested. The JAR now includes GlueGen natives; the recipe also retains its previously tested adjacent DLL and explicit library path.

## Verification

The September 2026 6.2 build was freshly compiled from source. Runtime and 3D renderer checks exercise the installed JAR and bundled Java, including the yellow exhaust animation. `packaging/verification/verify-windows.ps1` tests a disposable installation and refuses to replace an existing MIT installation. It uses the compiled Java probes in `build/package-verification`, then removes only the test installation. `-ResumeOwnedTest` requires the ownership receipt from that same test.

The test host is Windows 11 ARM64 in Parallels, running x64 Java under emulation. See `doc/improvements/04-desktop-builds-6.2.md` and versioned build evidence for final results. Native Intel/AMD Windows hardware, interactive installer pages and automatic updates are outside this packaging check. The earlier 6.1 report remains in `doc/FCsim/windows-installer-build-report.md`.
