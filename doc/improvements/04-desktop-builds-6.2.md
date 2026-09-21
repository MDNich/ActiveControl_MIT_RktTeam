# OpenRocket MIT 6.2 desktop builds

Built on 21 September 2026 from the current source, including the FC simulation, telemetry file selectors, 3D trajectory animation, four red/blue fins, pink/purple parachute and yellow exhaust through motor burnout. The upstream base remains 24.12.RC.01.

## Deliverables

Saved outside the Gradle build tree in:

`/Users/mdn/Developer/ActiveControl_MIT_RktTeam/exports/OpenRocket-MIT-6.2-2026-09-21`

| File | Bytes | SHA-256 |
| --- | --- | --- |
| `OpenRocket-MIT-6.2-macOS-Universal.dmg` | 204,117,310 | `557b1f3c3c1f5c08435fdc6168abbe7107cbe1d023f4e0efbb52fa718136fc7f` |
| `OpenRocket-MIT-6.2-macOS-Universal.zip` | 197,616,835 | `7801c02d6d028b44dde7060d95e8fd3b5b8fc60dd97f800ce0a13943211e5c79` |
| `OpenRocket-MIT-6.2-Windows-x64-Setup.exe` | 140,879,872 | `a6eac88a725175528f9e61ddb5facad44ae754b0d8a91dfae5d7251830c5dd56` |

The DMG and ZIP contain the same universal Mac app. The Windows EXE is an x64 installer. All include Temurin Java 17.0.20.1+1; recipients do not need a separate Java installation. Build manifests, installation instructions and `SHA256SUMS.txt` are adjacent to the deliverables.

## Universal Mac application

`OpenRocket_MIT.app` contains a native launcher with both arm64 and x86_64 slices, one shared application JAR, and separate matching Java runtimes. The selected launcher slice starts the corresponding JVM. No library or module files from different architectures are mixed together.

The bundle has no external filesystem symlinks and does not depend on the developer checkout. The DMG provides an Applications shortcut for drag-and-drop installation. Packaging leaves the existing installed Mac app in place; the existing launcher's compatibility JAR remains a full, matching artifact.

The Mac app is **ad-hoc signed, not Developer-ID signed or notarized**. The Windows installer is **unsigned**. These are local project builds.

## Source and verification

Source revision: `65defde513c30e1b256c65cd8ecb85726950df78`. The source subtree was clean when compiled. The preparation recipe reran all 15 Gradle tasks, including core and Swing compilation, and finished successfully.

Every platform packages the exact same JAR, SHA-256:

`a664988baf2300636175b19c9a6179ebab9a1ec956fb05fd95dd0494a2d7ff56`

- Mac Apple Silicon: bundled runtime, GlueGen/OpenGL, the actual 100,000-sample trajectory renderer, PNG export and universal native launcher passed.
- Mac Intel: the same checks passed through Rosetta on the Apple Silicon host.
- Mac Launch Services: ordinary app opening reached the main rocket editor and passed the disposable launch test.
- Mac distribution: the ZIP contents and JAR were verified; the DMG checksum passed, mounted read-only, and its app signature, two launcher architectures and JAR hash were verified.
- Windows: installer completed successfully, the installed JAR matched the source build, and the bundled runtime reported MIT 6.2 with the exhaust feature present. GlueGen/OpenGL, the actual trajectory renderer and PNG export passed. The installed launcher reached the main rocket editor.
- Windows disposable installation cleanup: uninstall returned exit code 0 and removed the test installation. The preexisting stock OpenRocket installation remained present.

The Windows host is Windows 11 ARM64 in Parallels, running the x64 application under emulation. Intel Mac and Intel/AMD Windows hardware were not available for native-machine testing. Interactive Windows installer pages, full application regression coverage, automatic updates and new flight-data comparisons were not part of this packaging request. The existing trajectory unit tests were already passing before this packaging-only change.

The initial Windows main-window check used a short timeout and process-window polling; the final check instruments only its disposable test JVM and observes the actual Swing editor. A PowerShell exit-code collection issue was corrected in the verifier. These verification fixes did not alter the shipped application or installer.

## Reproduction and evidence

- `packaging/windows/prepare-windows.py` rebuilds the source and records provenance, versions and pinned download checksums.
- `packaging/windows/package-windows.ps1` derives the installer version from staged metadata and retains the established MIT upgrade identity.
- `packaging/macos/package-macos.py` and `launcher.c` produce the universal app, ZIP and DMG. Runtime download checksums and the build command are in `packaging/macos/README.md`.
- `packaging/verification/` contains the packaged-runtime, native-launch and Windows installation checks. The trajectory render fixture remains in `doc/improvements/verification/TrajectoryRenderProbe.java`.

Detailed logs and screenshots are retained in `clone/openrocket/build/windows-installer/6.2/verification/` and `clone/openrocket/build/macos-installer/6.2/verification/`. Packaging outputs and full build logs remain in those versioned build directories. The final deliverables above remain available independently of a Gradle clean.
