# Universal macOS package

`package-macos.py` packages the current MIT shadow JAR into one `OpenRocket_MIT.app` for Apple Silicon and Intel, then produces a DMG and ZIP. Java is bundled; no system Java is required. The application contains one JAR and two independent Java runtimes. Its native launcher has both arm64 and x86_64 slices and chooses the matching runtime, preserving launch arguments. This avoids mixing architecture-specific JVM libraries or Java module images.

Build the source first with JDK 17 and `./gradlew --offline --rerun-tasks shadowJar` from `clone/openrocket`. The Windows preparation script performs this same source build; both platforms can then package the identical JAR.

Supply the `Contents/Home` directories of Temurin 17 macOS arm64 and x64 JDKs:

```sh
python3 packaging/macos/package-macos.py \
  --arm-jdk 'clone/openrocket/build/macos-installer/downloads/OpenJDK17U-jdk_aarch64_mac_hotspot_17.0.20.1_1/jdk-17.0.20.1+1/Contents/Home' \
  --intel-jdk 'clone/openrocket/build/macos-installer/downloads/OpenJDK17U-jdk_x64_mac_hotspot_17.0.20.1_1/jdk-17.0.20.1+1/Contents/Home'
```

The 6.2 release used these upstream [Temurin 17.0.20.1+1 assets](https://github.com/adoptium/temurin17-binaries/releases/tag/jdk-17.0.20.1%2B1), verified against their published SHA-256 files before extraction:

| Archive | SHA-256 |
| --- | --- |
| `OpenJDK17U-jdk_aarch64_mac_hotspot_17.0.20.1_1.tar.gz` | `196d13ba5f10414bef7f6a05a9b3f00edacb18ebacef2b99485db9e2ee18f0e8` |
| `OpenJDK17U-jdk_x64_mac_hotspot_17.0.20.1_1.tar.gz` | `c01975da12ed4235250ff891fe8bba73a9e73037d444b269c9d0922b5dbc8e0a` |

Xcode command-line tools, Python 3.11+, and Rosetta (to execute Intel packaging tools on Apple Silicon) are required for this recipe. Output is versioned under `clone/openrocket/build/macos-installer/6.2/output/`. The script refuses to replace an existing app staging directory; preserve outputs before rebuilding in fresh staging.

The DMG offers drag-and-drop installation into Applications. The bundle is self-contained, contains no links into the developer's checkout, and uses an ad-hoc signature. It is **not Developer-ID signed or Apple-notarized**. Packaging does not replace the installed `/Applications/OpenRocket_MIT.app`.

The actual launchers, bundled Java/graphics libraries, and 100,000-sample trajectory renderer are exercised for both architectures. Intel execution was tested through Rosetta on Apple Silicon, rather than on Intel hardware. A separate Launch Services check verifies ordinary macOS app opening. The updated app's runtime directories differ from install4j's layout; automatic updating remains untested. See `doc/improvements/04-desktop-builds-6.2.md` for artifact locations and verification results.
