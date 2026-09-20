# OpenRocket MIT Windows installer

Built on 20 September 2026 from the local OpenRocket source, repository revision `fa410c71e7c65bb8fedc9850a5166ca3baf37bca`.

| Item | Result |
| --- | --- |
| Application | OpenRocket MIT edition 6.1, based on 24.12.RC.01 |
| Installer | `OpenRocket-MIT-6.1-Windows-x64-Setup.exe` |
| Windows package version | 6.1.0 |
| Architecture | Windows x64 |
| Size | 140,744,704 bytes (134.2 MiB) |
| Bundled Java | Eclipse Temurin 17.0.20.1+1, Windows x64 |
| Packaging | OpenJDK jpackage with WiX 3.14.1 |
| Signing | Unsigned |

The exported installer is in `/Users/mdn/Downloads/OpenRocket-MIT-6.1-Windows-x64/`. The original build output, manifest, and packaging log are in `clone/openrocket/build/windows-installer/output/` under the ActiveControl_MIT_RktTeam repository.

Installer SHA-256:

```text
2d26c699b44b1b2a53d67d1ce04b16e50d708bb4443d663b0a7fe50d12e43148
```

## What was verified

1. **Fresh source compilation passed.** `./gradlew --offline --rerun-tasks shadowJar` reran all 14 build tasks, including the core and Swing Java compilation tasks.
2. **Windows packaging passed.** The installer bundles a Java runtime and the matching GlueGen 2.5.0 Windows native DLL required by graphics initialization. The freshly compiled JAR was copied without modification.
3. **Installation passed.** The installer returned exit code 0. The installed JAR's SHA-256 matched the source-built JAR: `8fc73350b98baf4bd05f0f7dbe66d597c86c4ce5433e09050aae75aa64e10bfe`.
4. **Runtime and graphics checks passed.** The packaged runtime reported Java 17, OpenRocket 24.12.RC.01, and MIT edition 6.1. GlueGen initialized successfully, and OpenGL reported `GL4bc/GL4bc.hw`.
5. **Installed application launch passed.** The installed launcher opened `A simple model rocket.ork` in a responding application window. The screenshot shows the rocket editor and the fork's Airbrakes and TabCtrlTrapezoidal component controls.
6. **Uninstall passed.** Uninstall returned exit code 0; this test installation's directory and registration were removed. The preexisting stock OpenRocket installation remained present.

The test machine was the existing Windows 11 ARM64 Parallels VM, running x64 Java and OpenRocket under emulation. The installer was exercised silently; its interactive wizard pages were not reviewed. Native Intel/AMD Windows testing, automatic updates, the full application test suite, and a new flight simulation run were not part of this packaging check. Saved results visible in the example are not evidence of a newly run simulation.

The installer contains the current Java FC simulation framework. Integrating the Arduino C++ flight computer remains the next development phase.

## Rebuilding and evidence

Reusable scripts and instructions are in `packaging/windows/README.md`, `prepare-windows.py`, and `package-windows.ps1` in the parent repository. The source build and Windows packaging steps were executed successfully; the Python convenience wrapper was syntax checked, but not separately run end to end.

The build directory contains `source-build.log`, `output/windows-build.log`, `output/build-manifest.json`, and the following files under `verification/`: `windows-smoke.log`, `installer-verification.log`, `install.log`, `gui-launch.json`, `windows-launch.png`, `uninstall-verification.json`, and `uninstall.log`.

No production Java or Arduino source files were changed to create this installer.
