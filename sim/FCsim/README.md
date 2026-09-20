# Running the Java Zephyrus FC

The translated FC runs through the existing `FlightControllerSimulatorListener`. It prints its actions using `System.out.println`, including sensor delivery, instrument processing, state changes, controller updates, PWM selection, physical airbrake exposure, commands, logical pyros, power, flash, and telemetry. Each line starts with `ZEPHYRUS run=… boot_us=… action=…`.

The implementation and its limits are described in the [implementation report](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/doc/FCsim/java-translation-implementation-report.md).

## Ready-to-inspect verification output

A synthetic flight has been run using the compiled root shadow JAR. It exercises the implementation; it is not a prediction of the test launch.

- [Full console log](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/output/verification-20260920/console.log)
- [Telemetry CSV: 600 packets, 43 columns](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/output/verification-20260920/zephyrus-809613656896466199/telemetry.csv)
- [Raw transmitted payloads](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/output/verification-20260920/zephyrus-809613656896466199/packets.bin)
- [Telemetry metadata](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/output/verification-20260920/zephyrus-809613656896466199/metadata.txt)
- [Independent packet validation](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/output/verification-20260920/telemetry-verification.json)
- [Build and test log](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/output/verification-20260920/build-and-tests.log)

`output/` is ignored by Git. Each run creates a unique `zephyrus-*` directory and prints the absolute CSV, binary, and metadata paths. Files are flushed after each packet and closed at simulation end, including errors.

## Run from the command line

Use JDK 17. Build from the OpenRocket directory:

```sh
cd /Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket
./gradlew shadowJar
```

Run the synthetic verification without opening a window:

```sh
cd /Users/mdn/Developer/ActiveControl_MIT_RktTeam
mkdir -p sim/FCsim/output
sim/FCsim/run-java-fc.sh --synthetic 2>&1 | tee sim/FCsim/output/synthetic-console.log
```

The supporting runner compiles against `build/libs/OpenRocket-24.12.RC.01.jar` and calls the existing `Simulation.simulate(...)` method. It uses the packaged FC classes. Set `FC_OUTPUT_DIR=/absolute/path` to change the output parent directory. Console filenames in these examples are overwritten when reused; telemetry directories remain unique.

To inspect the supplied benchmark rocket and resolve its motor without simulating:

```sh
sim/FCsim/run-java-fc.sh --inspect \
  sim/FCsim/examples/zephy_testlaunch-java-fc.ork \
  sim/old/dat/ork/TestLaunch_v14.eng
```

The original benchmark files remain in `benchmarking/`. The prepared [Java FC example](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/examples/zephy_testlaunch-java-fc.ork) has the FC listener selected and its previous simulation results removed. Geometry, motor configuration, and saved launch conditions are unchanged. The N8406 motor resolves without loader warnings when the specified ENG is supplied.

For the next, separately requested launch comparison, change `--inspect` to `--run` after reviewing the launch conditions. The runner uses the first saved simulation and replaces its saved extensions with exactly one Java FC listener. It does not modify the input ORK. It never feeds recorded benchmark telemetry into the FC.

To launch the GUI from a terminal and capture its output:

```sh
java -Xmx2g \
  -Dopenrocket.fc.telemetryDir=/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/output \
  -jar /Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket/build/libs/OpenRocket-24.12.RC.01.jar \
  /Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/examples/zephy_testlaunch-java-fc.ork \
  2>&1 | tee /Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/output/gui-console.log
```

Add [TestLaunch_v14.eng](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/old/dat/ork/TestLaunch_v14.eng) to OpenRocket's user-defined thrust-curve files if the GUI does not already know the N8406 motor. Run the saved simulation with the Java-code extension class `info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener`. Use a single active stage with exactly one `AirbrakeSet`.

## Telemetry contract

`telemetry.csv` uses exactly the 43 column names and order in all three `ZEPH_TEST_FLIGHT_GS*.csv` benchmarks. Arrays use quoted Python-style list cells. CSV values are decoded from the actual synthesized FC packet bytes using the ground-station decoder's conventions.

`packets.bin` contains consecutive 128-byte FC payloads, including their checksum at byte 127. It does not contain modem framing, RSSI suffixes, or a radio propagation model. The FC's strict `>50 ms` telemetry condition produces one packet every 60 ms with the 10 ms loop.

- `timestamp` is simulated boot seconds plus `-Dopenrocket.fc.epochSeconds` (default 0). Set that property when an explicit epoch alignment is needed. No wall clock determines FC behavior.
- `flight_time` retains the historical decoder's meaning: firmware boot **milliseconds**, not seconds after liftoff. The console records the actual `flight_begin_ms` separately. Physical simulation time zero occurs at boot 1000 ms after stationary warmup.
- RSSI and ground-station position/fix fields are blank. GPS uncertainty/satellite count and pyro resistance are unmodeled zero. Power readings use nominal simulated voltages and zero current.
- Accelerometer/gyro raw counts are synthesized from calibrated engineering readings and clamped to sensor range. Historical ground-station conversions omit the FC's accelerometer-X gain and gyro biases. Its temperature calibration constants also differ from the FC's. Those CSV values intentionally retain the decoder convention; the console contains FC engineering values.
- Pyros are recorded logical outputs. OpenRocket's configured recovery controls the trajectory. Physical roll-tab coupling is disabled. Airbrake motion uses an ideal linear mapping from latched PWM to exposed area.

Check any completed telemetry directory with:

```sh
python3 sim/FCsim/verify-telemetry.py /absolute/path/to/zephyrus-RUN_DIRECTORY
```

The check compares benchmark headers, checks every checksum, independently decodes key scalar/vector fields, and checks packet cadence and sequence.

## Before the actual launch comparison

The saved benchmark launch site is 28.61°, −80.6°, altitude 0 m; its telemetry reports a location near 42.7042°, −77.1919°. Resolve the actual site, elevation, atmosphere, wind, sensor mounting, recovery setup, and time alignment before treating a run as a launch match. These have not been fitted or silently changed during implementation.
