# OpenRocket MIT Linux x86_64 build

Built on 27 October 2026 from repository revision `7f0b67ee1c755ec89f154b0f0d2f4e23c843aa87`. The application is MIT edition **6.4**, based on OpenRocket **24.12.RC.01**. Java classes were compiled on macOS with Temurin 17.0.21+9; Linux launchers, the bundled runtime and packages were assembled inside Ubuntu 24.04 x86_64 with the matching Linux x64 JDK.

## Artifacts

The local output directory is [`clone/openrocket/build/linux-installer/6.4/output/`](../../clone/openrocket/build/linux-installer/6.4/output/). These unsigned packages were added to the [GitHub v6.4 prerelease](https://github.com/MDNich/ActiveControl_MIT_RktTeam/releases/tag/v6.4) on 27 October 2026. The existing release was changed to prerelease status; its original macOS, Windows and standalone JAR downloads and tag were preserved. The release notes explicitly distinguish their original `9f71a94196a6284ae2b574639aafbb2bf1bc175a` source revision from the newer Linux build revision.

| Artifact | Size | SHA-256 |
| --- | ---: | --- |
| [Ubuntu installer](../../clone/openrocket/build/linux-installer/6.4/output/OpenRocket-MIT-6.4-Linux-x86_64.deb) | 137,219,300 bytes | `e85e8628c02b5883ab1db1bb70bf39b19bd5fcb50f06c069c3524cadc9b84fdb` |
| [Portable archive](../../clone/openrocket/build/linux-installer/6.4/output/OpenRocket-MIT-6.4-Linux-x86_64.tar.gz) | 143,156,523 bytes | `b722a958895add47d4be00c8be9338bb4e7977a3e52eb9b268628f2beaa5d244` |

Both contain Java 17.0.21+9, the MIT application JAR and eight Linux amd64 JOGL/GlueGen native libraries. The JAR SHA-256 is `773c6a2772160f2868a67b8daa83ea75391762138c13fbce79b3ddf2cafee8f5`. Artifact checksums were verified again after copying the packages from Ubuntu to the Mac. The output directory also contains `SHA256SUMS` and `build-manifest.json`.

The release includes separate Linux checksum and build-manifest assets, named `OpenRocket-MIT-6.4-Linux-x86_64-SHA256SUMS.txt` and `OpenRocket-MIT-6.4-Linux-x86_64-build-manifest.json`. GitHub's reported SHA-256 digests and sizes were verified against all four uploaded files. The publication record, before/after release metadata and a screenshot are retained in `clone/openrocket/build/linux-installer/6.4/github-prerelease/`.

Install on Ubuntu 24.04 with:

```sh
sudo apt install ./OpenRocket-MIT-6.4-Linux-x86_64.deb
/opt/openrocket-mit/bin/OpenRocket-MIT
```

Alternatively, extract the portable archive and run `OpenRocket-MIT/bin/OpenRocket-MIT`. Keep the complete extracted folder together. Java is included, but the desktop graphics and font libraries must be installed. This Debian package was built and tested on Ubuntu 24.04; its generated dependencies include Ubuntu's `libasound2t64`, so compatibility with older Ubuntu releases or every Debian derivative is not established.

## Ubuntu VM

The reusable UTM VM is **Ubuntu 24.04 x86_64 - OpenRocket MIT** (`DEE945B6-4258-4D86-8774-8CBDDEC782EC`). It uses QEMU x86_64 emulation on the ARM Mac, four emulated CPU cores, 6 GiB RAM, a 40 GiB virtual disk, an Xfce desktop and software-rendered graphics. The installed application is `/opt/openrocket-mit/`; test inputs and evidence are under `/home/rocket/`.

The VM bundle is at:

```text
/Users/mdn/Library/Containers/com.utmapp.UTM/Data/Documents/Ubuntu 24.04 x86_64 - OpenRocket MIT.utm
```

Open UTM and select that VM. The dedicated `rocket` desktop account logs in automatically. Its generated console password is in the private local file `/Users/mdn/Library/Application Support/OpenRocket Linux Test/credentials.txt`; it is not stored in the repository. SSH uses a dedicated key and a host-only loopback forward, `127.0.0.1:22240` to guest port 22:

```sh
ssh -F '/Users/mdn/Library/Application Support/OpenRocket Linux Test/ssh-config' openrocket-linux
```

The guest has NAT internet access. It was provisioned specifically for testing, with passwordless sudo for `rocket`. The existing Parallels Windows VM was left untouched.

## Verification

| Check | Result and evidence |
| --- | --- |
| Debian installation and architecture | Passed. Package architecture is `amd64`; the installed launcher and Java executable are ELF x86-64 binaries. The installed JAR matches the compiled JAR byte for byte. |
| Bundled runtime and graphics libraries | Passed. Java 17.0.21+9, MIT 6.4, GlueGen initialization and JOGL profile `GL4bc/GL4bc.sw`. |
| Fresh nominal flight | Passed. The first simulation in the built-in simple-model example reached apogee and ground impact without simulation errors: 50.600 m apogee, 15.888 s duration, 1,549 samples in the final run. |
| Model save/reopen | Passed. The saved model reopened with matching component identities and simulation count. |
| Bundled compiler | Passed. `ToolProvider.getSystemJavaCompiler()` compiled a small Java source using the packaged runtime. |
| FC execution and packet output | Passed. A three-second fixture generated and received 66 packets, totaling 8,448 bytes. Every 128-byte packet passed the checksum check. |
| 3D trajectory | Passed. The actual `Trajectory3DPanel` rendered and produced a PNG, which was visually inspected. |
| Installed launcher | Passed. The native packaged launcher opened the example in a visible `BasicFrame`, using `/opt/openrocket-mit/lib/runtime`. |
| Portable launcher | Passed. The extracted application opened the example from `portable with spaces/OpenRocket-MIT`, using its own bundled runtime. |
| Desktop GUI | Passed. OpenRocket was also launched on the VM's normal Xfce desktop; the example model, component tree and rocket drawing were visually verified. |

The [verification summary](../../clone/openrocket/build/linux-installer/6.4/verification/guest/verification-2/verification.json), [desktop screenshot](../../clone/openrocket/build/linux-installer/6.4/verification/ubuntu-openrocket-desktop.png) and [3D trajectory capture](../../clone/openrocket/build/linux-installer/6.4/verification/linux-trajectory-3d.png) are saved with the local evidence. The screenshot's selected C6 flight configuration displays the example's saved results; the fresh nominal flight result above comes from the separate logged simulation check. The final resumed stage exited successfully with `LINUX_PACKAGE_VERIFICATION=PASS`.

Two verification-helper issues were corrected during testing. The initial FC fixture used the stock example without an `AirbrakeSet`; the final fixture attaches a small synthetic airbrake set to a separate reopened copy. The original launcher observer could initialize Swing before OpenRocket installed its context class loader, causing an error dialog during instrumented startup on this slow emulator. The observer now waits for the application's own event thread, has a configurable startup allowance, and writes failures to stdout so the application's stderr redirection cannot hide them. Both launchers then passed. Completed flight/runtime checks were retained while the launcher stage was resumed against the same packages. Earlier logs are preserved with the evidence.

These are packaging and execution checks, not a full application regression suite or validation against real flight measurements. The FC check verifies bounded execution and packet output; it does not validate every controller, recovery configuration or external telemetry integration. Mesa llvmpipe supplied OpenGL 4.5 software rendering under Xvfb. Native Intel/AMD hardware, GPU acceleration, Wayland-only desktops, other Linux distributions, automatic updates and uninstall were not tested. Emulation timings should not be used as native Linux performance measurements.

## Rebuilding and evidence

Reusable packaging and VM setup instructions are in [`packaging/linux/README.md`](../../packaging/linux/README.md). The new packaging script creates both formats using `jlink` and `jpackage`. It retains `jdk.compiler`, which the FC workflow needs. The scripts operate on fresh staging/output directories to preserve existing artifacts.

The source build required the core and Swing module JARs to be built before `shadowJar`, avoiding the existing clean-build module-transform ordering issue. Both modules were recompiled with UTF-8 and `--rerun-tasks` after the host's unavailable `C.UTF-8` locale caused encoding errors in an earlier attempt. The final module build and `shadowJar` completed successfully; the final JAR was packaged without modification.

Build logs are in `clone/openrocket/build/linux-installer/`: `utf8-build.log`, `shadow-build.log`, and the guest logs copied under `6.4/verification/guest/`. The final artifact manifest records the full Java module list and native libraries. Test probes are in [`packaging/verification`](../../packaging/verification/); Linux-specific verification is driven by [`verify-linux.sh`](../../packaging/linux/verify-linux.sh).

The evidence directory includes the initial failed fixture run, the final `verification-2/` results, the launcher observer thread dump and retry logs, FC packet files, the saved `.ork` model, setup/package logs, VM configuration and verification-source checksums. The expanded portable application is omitted from the evidence archive because the original distribution archive is retained separately. No production application or FC source files were changed to create these packages.

## Download provenance

UTM was downloaded from its [official 5.0.8 release](https://github.com/utmapp/UTM/releases/tag/v5.0.8), installed at `/Applications/UTM.app`, and accepted by macOS code-signature and notarization checks. Ubuntu came from the [official Noble cloud-image release directory](https://cloud-images.ubuntu.com/releases/noble/release/), with its checksum checked against the published checksum file. The Linux JDK came from the [official Temurin 17.0.21+9 release](https://github.com/adoptium/temurin17-binaries/releases/tag/jdk-17.0.21%2B9), with its checksum checked against Adoptium metadata.

| Download | SHA-256 |
| --- | --- |
| `UTM-5.0.8.dmg` | `7b2d7b2e2a02b3de1757ef19a8f36573e6d9cceae8c8cca0abcbcaa4a852019a` |
| `ubuntu-24.04-server-cloudimg-amd64.img` | `04e8f2461a16a44e78a370284471f29616cdd9baf1893df25807b71f985b6c94` |
| `OpenJDK17U-jdk_x64_linux_hotspot_17.0.21_9.tar.gz` | `8b9b611d917e3dfd8419d3566b5949226f69b052fed7e109c1368add1dfe8711` |
