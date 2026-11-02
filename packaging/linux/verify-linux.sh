#!/usr/bin/env bash
# Run in the dedicated x86_64 Ubuntu test guest after package-linux.py completes.
set -euo pipefail
input=$(realpath "${1:?input staging tree}")
packages=$(realpath "${2:?package output directory}")
evidence=${3:?new verification output directory}
test ! -e "$evidence"
mkdir -p "$evidence"
evidence=$(realpath "$evidence")
edition=$(sed -n 's/^build.mit.version=//p' "$input/core/src/main/resources/build.properties")
test "$(uname -m)" = x86_64
deb="$packages/OpenRocket-MIT-$edition-Linux-x86_64.deb"
test "$(dpkg-deb -f "$deb" Architecture)" = amd64
dpkg-deb -f "$deb" > "$evidence/debian-control.txt"
(cd "$packages" && sha256sum -c SHA256SUMS) > "$evidence/artifact-checksums.txt"
sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y "$deb" > "$evidence/install.log" 2>&1
launcher=$(dpkg-query -L openrocket-mit | sed -n '\|/bin/OpenRocket-MIT$|p')
test -x "$launcher"
app=$(dirname "$(dirname "$launcher")")
java="$app/lib/runtime/bin/java"
jar="$app/lib/app/OpenRocket.jar"
cmp "$jar" "$input/build/libs/OpenRocket-MIT-v$edition.jar"
file "$launcher" "$java" > "$evidence/executables.txt"
native="$app/lib/app/native"
options=("--add-exports=java.base/java.lang=ALL-UNNAMED"
         "--add-exports=java.desktop/sun.awt=ALL-UNNAMED"
         "--add-exports=java.desktop/sun.java2d=ALL-UNNAMED"
         "-Djava.library.path=$native" "-Djava.util.prefs.userRoot=$evidence/preferences")
export LIBGL_ALWAYS_SOFTWARE=1
xvfb-run -a glxinfo -B > "$evidence/opengl.txt" 2>&1
timeout 180s xvfb-run -a "$java" "${options[@]}" -cp "$input/verification:$jar" \
  PackagedSmokeCheck "$edition" > "$evidence/runtime-smoke.log" 2>&1
timeout 600s xvfb-run -a "$java" "${options[@]}" -Xmx2g -cp "$input/verification:$jar" \
  LinuxPackageProbe "$input/example.ork" "$evidence" > "$evidence/simulation-render.log" 2>&1
JAVA_TOOL_OPTIONS="-javaagent:$input/launcher-smoke-agent.jar=amd64 -Dopenrocket.verification.launchTimeoutSeconds=240 -Djava.util.prefs.userRoot=$evidence/launcher-preferences" \
  timeout 420s xvfb-run -a "$launcher" "$input/example.ork" > "$evidence/installed-launcher.log" 2>&1
grep -q LAUNCHER_SMOKE_CHECK=PASS "$evidence/installed-launcher.log"
mkdir -p "$evidence/portable with spaces"
tar xzf "$packages/OpenRocket-MIT-$edition-Linux-x86_64.tar.gz" -C "$evidence/portable with spaces"
portable="$evidence/portable with spaces/OpenRocket-MIT"
cmp "$portable/lib/app/OpenRocket.jar" "$jar"
JAVA_TOOL_OPTIONS="-javaagent:$input/launcher-smoke-agent.jar=amd64 -Dopenrocket.verification.launchTimeoutSeconds=240 -Djava.util.prefs.userRoot=$evidence/portable-preferences" \
  timeout 420s xvfb-run -a "$portable/bin/OpenRocket-MIT" "$input/example.ork" > "$evidence/portable-launcher.log" 2>&1
grep -q LAUNCHER_SMOKE_CHECK=PASS "$evidence/portable-launcher.log"
python3 - "$evidence" "$app" <<'PY'
from pathlib import Path
import hashlib,json,platform,sys
root,app=map(Path,sys.argv[1:])
jar=app/'lib/app/OpenRocket.jar'
(root/'verification.json').write_text(json.dumps({
    'platform':platform.platform(), 'architecture':platform.machine(),
    'installed_app':str(app), 'jar_sha256':hashlib.file_digest(jar.open('rb'),'sha256').hexdigest(),
    'debian_install':'PASS','runtime_and_native_libraries':'PASS',
    'nominal_flight_apogee_landing':'PASS','save_reopen':'PASS',
    'bundled_java_compiler':'PASS','fc_packets_and_checksums':'PASS',
    'trajectory_3d':'PASS','installed_launcher':'PASS','portable_launcher_path_with_spaces':'PASS',
    'graphics':'Mesa software rendering under Xvfb; see opengl.txt',
},indent=2)+'\n')
PY
printf 'LINUX_PACKAGE_VERIFICATION=PASS\n'
