# Linux x86_64 packaging

`package-linux.py` builds an application image, a portable `.tar.gz`, and an Ubuntu/Debian `.deb`. Each contains a Linux x86_64 Java 17 runtime, the MIT shadow JAR and the Linux JOGL/GlueGen native libraries. The package step must run on Linux x86_64; compiling the architecture-independent Java classes may run on another platform.

## Build

Use Java 17 and an available UTF-8 locale. In the OpenRocket source directory, build the module JARs before the aggregate distribution. This avoids the existing clean-build module transform ordering issue.

```sh
./gradlew --no-daemon :core:jar
./gradlew --no-daemon :swing:jar
./gradlew --no-daemon shadowJar
```

On macOS, use `LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 JAVA_TOOL_OPTIONS=-Dfile.encoding=UTF-8` for those commands. The macOS shell's `C.UTF-8` locale is not available on every host, and an ASCII default can corrupt non-ASCII source strings during compilation. Recompile both module JARs with `--rerun-tasks` if an earlier invocation reported encoding errors.

On the Linux packaging machine, install Python 3.11 or newer, `fakeroot`, `binutils`, `dpkg-dev`, and a Linux x86_64 JDK 17. Download the JDK from its official distributor and verify its checksum. Then run:

```sh
python3 packaging/linux/package-linux.py \
  --source clone/openrocket \
  --jdk /path/to/linux-x64-jdk-17 \
  --work /path/to/new-staging-directory \
  --output /path/to/new-output-directory
```

Both work and output directories must be new. The output includes packages, `SHA256SUMS`, and a manifest with the application checksum, runtime release and native libraries. A minimal source staging tree can contain just the built JAR and resource paths read by the script.

## Install or run

On Ubuntu 24.04, install the `.deb` using `sudo apt install ./OpenRocket-MIT-6.5-Linux-x86_64.deb` after packaging this edition. This resolves desktop library dependencies and adds the application menu entry. The generated Debian dependencies reflect the packaging host, including its `t64` library names; older Ubuntu releases and other distributions require separate validation. The portable archive can be extracted to a user-owned folder and launched with `OpenRocket-MIT/bin/OpenRocket-MIT`, optionally followed by a `.ork` filename. It requires an X11 session or XWayland and the system graphics/font libraries listed in its README. Java is included.

## Dedicated Ubuntu test VM

`setup-ubuntu-test.sh` installs an Xfce desktop and packaging/test dependencies in a dedicated Ubuntu 24.04 x86_64 guest. It enables desktop auto-login for the named test account; do not run it on a general-purpose workstation. Run as root, passing an existing test username. UTM/QEMU can emulate this guest on an ARM Mac; native x86_64 virtualization is faster.

The reusable probes under `packaging/verification` are:

- `PackagedSmokeCheck.java`: edition, runtime architecture, native graphics initialization and feature presence.
- `LauncherSmokeAgent.java`: observes the real application's main window using the packaged launcher.
- `LinuxPackageProbe.java`: completes a flight, saves/reopens its model, compiles a small Java source with the bundled runtime, checks FC packet output/checksums, and captures the actual 3D trajectory panel.

Compile the probes against the final JAR, then run them using the **packaged runtime and packaged JAR**. Use an isolated preferences directory and output folder. The FC fixture is a bounded three-second execution check; the separate nominal flight fixture verifies apogee and landing. A Mesa software-rendered VM verifies the graphics path but is not a performance benchmark for native Linux hardware.

The launcher observer waits for the application's own Swing event thread before inspecting windows, preserving OpenRocket's class-loader initialization order. The Linux harness sets `openrocket.verification.launchTimeoutSeconds=240` and an outer 420-second limit to accommodate x86_64 emulation; the observer's default remains 60 attempts for existing callers.

For `verify-linux.sh`, prepare an input directory with the same `build/libs/` and `core/src/main/resources/build.properties` paths used by the packaging script, plus the compiled probes and the example model. From the repository root, with `linux_input` pointing at that staging directory and Java 17's `bin` on `PATH`:

```sh
linux_edition=$(sed -n 's/^build.mit.version=//p' "$linux_input/core/src/main/resources/build.properties")
linux_jar="$linux_input/build/libs/OpenRocket-MIT-v$linux_edition.jar"
mkdir -p "$linux_input/verification"
javac -encoding UTF-8 -cp "$linux_jar" -d "$linux_input/verification" \
  packaging/verification/PackagedSmokeCheck.java \
  packaging/verification/LauncherSmokeAgent.java \
  packaging/verification/LinuxPackageProbe.java
printf 'Premain-Class: LauncherSmokeAgent\n\n' > "$linux_input/agent-manifest.txt"
jar --create --file "$linux_input/launcher-smoke-agent.jar" \
  --manifest "$linux_input/agent-manifest.txt" -C "$linux_input/verification" .
unzip -p "$linux_jar" 'datafiles/examples/A simple model rocket.ork' > "$linux_input/example.ork"
```

Transfer the input directory and packages to the prepared Ubuntu guest. As the test account with sudo access, run `bash verify-linux.sh /path/to/input /path/to/packages /path/to/new-evidence`. It installs the Debian package, uses Xvfb for the automated GUI checks, and verifies the portable launcher from a path containing spaces. Only a successful complete run writes `verification.json` and prints `LINUX_PACKAGE_VERIFICATION=PASS`. The FC probe adds synthetic airbrakes to a separate copy of the example, satisfying FC component binding without changing the nominal flight model.

See the [Linux build report](../../doc/FCsim/linux-x86_64-build-report.md) for recorded artifacts, checksums, guest details and verification results from this build.
