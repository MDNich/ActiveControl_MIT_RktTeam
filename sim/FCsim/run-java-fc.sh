#!/usr/bin/env bash
set -euo pipefail
fc_support_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fc_repo="$(cd "$fc_support_dir/../.." && pwd)"
fc_source="$fc_repo/clone/openrocket"
fc_jar="$fc_source/build/libs/OpenRocket-MIT-6.2.jar"
fc_output="${FC_OUTPUT_DIR:-$fc_support_dir/output}"
mkdir -p "$fc_output/runner-classes"
javac -cp "$fc_jar" -d "$fc_output/runner-classes" "$fc_support_dir/FcSimulationRunner.java" "$fc_source/core/src/test/java/edu/mit/rocket_team/zephyrus/RTFCVerificationRocket.java"
java -Xmx2g -Djava.awt.headless=true -Dopenrocket.fc.telemetryDir="$fc_output" -cp "$fc_output/runner-classes:$fc_jar" FcSimulationRunner "$@"
