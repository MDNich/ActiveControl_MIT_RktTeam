# Zephyrus Java FC implementation report

Completed 20 September 2026. The existing Java FC classes and `FlightControllerSimulatorListener` now execute the FC-included firmware behavior inside OpenRocket. The root shadow JAR was built and used for a complete synthetic verification run. Telemetry is exported as both benchmark-compatible CSV and raw FC packet payloads. The actual test-launch comparison remains the next stage.

## Implementation and evidence

The implementation extends the approved class structure. `RTFC.init → pre_loop → loop` remains the lifecycle, instruments supply the controller inputs, and `RTSimulationCommunicator` applies the effective airbrake output to the existing `AirbrakeSet`. No JNI or replacement simulation framework was added. The separate `AirbrakesControllerListener` and standalone airbrakes sketch were excluded.

The [source manifest](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/doc/FCsim/java-translation-source-manifest.json) records the selected sketch, library, ground-station decoder, benchmark files, and original packaging baseline. The selected sketch revision is `4cd2660eec92a47be42c09ed7b61f38295bcf970`; the firmware-library revision is `1db1f223c0c7d2287611c9f66d5de6c691455359`. File hashes identify the actual checked-out contents.

| Plan step | Completed behavior | Verification |
| --- | --- | --- |
| 1. Baseline | Source revisions/hashes, build baseline, saved native-reference traces | Source manifest; four checked-in C++ oracle CSVs |
| 2. Lifecycle | Per-flight FC, clock, instruments, output and files; shared live state across copies of the same flight; fresh state for each new run | Independent/interleaved FC tests, repeated physical runs, parachute-stepper copies, missing-component diagnostic |
| 3. Airbrakes | FC-included library translated in place, including sampling, fit, target calculation, ramp, plateau, integral and output request | 16,004 input rows compared against unchanged C++ library |
| 4. Instruments | Accelerometer readiness/integration, pressure-derived barometer filter, decoded GPS height/fix, gyro rate integration and zeroing | Pad, threshold, stale-data, offset, fix-loss, sign and clock-wrap checks |
| 5. FC | Source loop order, state transitions, strict timers, controller enable flags, uplink bytes and peripheral side effects | State/command replay, boundary times, emergency masks, second preflight, incomplete reset behavior |
| 6. Pyros and roll | Six logical pyro channels; source roll algorithm and servo requests | Arm/continuity/250 ms timeout tests; roll output included in all native-reference traces |
| 7. Effective output | Manual/automatic selection, integer PWM, 20 ms latch, ideal pulse-to-area linkage | PWM hold/closure, rejection of nonfinite actuator values, monotonic component drag, active-versus-closed trajectory |
| 8. Sensor adapter | Coherent first-derivative observations, explicit mounting, gravity/coriolis removal, world-to-body rotation, pressure/temperature and GPS units | Upright/tilted pad, free fall, thrust, rotated gyro, degree coordinates and GPS integer precision |
| 9. Timing | 10 ms FC ticks, 20 ms PWM, maximum 2.5 ms physics step, 1 s pad warmup; deadline handling across RK4/RK6, recovery and ignition delays | Identical FC schedules at 2.5/1 ms and under RK4/RK6; off-grid ignition, zero launch rod, repeat runs and existing engine regressions |
| 10. Packaging | Root shadow JAR, CLI support runner, prepared ORK, streamed console and telemetry files | Packaged synthetic flight; independent binary/CSV validation; benchmark motor-load inspection |

The proposed lifecycle/timing/roll test names were consolidated into `RTFCOutputTest`, `RTFCSimulationTest`, and `RTAirbrakesReplayTest`. Sensor-adapter tests are in `RTFCSensorAdapterTest`. All test sources live under [the FC test directory](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket/core/src/test/java/edu/mit/rocket_team/zephyrus/).

The validation selected **33 tests: 25 FC cases and 8 existing simulation tests**. All passed. The output tests were subsequently rerun successfully after adding an explicit nonfinite automatic-output assertion. Both core and Swing compiled, and root `shadowJar` succeeded with JDK 17.0.11 and Gradle 8.12.1.

Native-reference tests use a tiny fake Arduino clock and static controller initialization matching the sketch. Four traces cover integer FC time, fractional library time, partial samples, and late preparation. States and sample counts match exactly; deployment, prediction, target, integral and roll angle satisfy `abs(error) <= 1e-5 + 1e-5 * abs(reference)`. Reference outputs must be finite. This is numerical equivalence on those traces, not a claim of bit-for-bit equivalence on the embedded processor.

The [native fixture generator](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket/core/src/test/native/zephyrus/generate-reference.py) compiles the selected unchanged `airbrakes.cpp` and `rollcontrol.cpp` only to regenerate test data. Ordinary tests and the shipped JAR need no native FC library.

## Packaged result

Built artifact: [OpenRocket-24.12.RC.01.jar](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket/build/libs/OpenRocket-24.12.RC.01.jar).

SHA-256: `53812e46ddcc2b50d7788b672df0fbc803d99b8e66f91d903d22faef31dbe2fb`.

The packaged run used the shared synthetic rocket fixture, deterministic wind seed, one airbrake component and explicitly configured parachute recovery. It completed 35 seconds of physical simulation plus 1 second of stationary firmware warmup: **3,601 FC loops**, final FC state `APOGEE`, maximum altitude **1637.543 m above launch**. This rocket is a verification fixture, not the real launch model.

Observed firmware events, in boot time:

| Boot time | Event |
| --- | --- |
| 0.500 s | Preflight command queued and consumed at the source command-read position |
| 0.510 s | `GROUND_TESTING → PRE_FLIGHT` |
| 1.000 s | Physical simulation released from the pad |
| 1.080 s | FC detects `FLIGHT` from measured acceleration |
| 6.080 s | Airbrake controller enters `PREP` |
| 9.890 / 9.900 s | `PREPROCESS`, then `WAIT_FOR_START` |
| 14.080 / 15.080 s | Ramp, then plateau |
| 19.210 s | Controller enters `DONE` as estimated velocity becomes nonpositive |
| 27.090 s | FC enters `APOGEE` after its source lockout; closes/disables controllers and commands pyros 0/1 |

The physical tests demonstrate lower apogee with active airbrakes than with exposure held closed. Repeating the same run reproduces apogee within `1e-8 m`; changing the physics cap between 2.5 ms and 1 ms, or RK6 and RK4, preserves the FC tick sequence and passes the declared 2 m trajectory tolerance. Descent timers through `MAIN` and `END`, including all six pyro channels, are covered by deterministic replay even though the 35-second physical fixture does not wait through the 55-second main-deployment lockout.

Artifacts:

- [Complete console log](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/output/verification-20260920/console.log)
- [Telemetry CSV](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/output/verification-20260920/zephyrus-809613656896466199/telemetry.csv)
- [128-byte payload stream](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/output/verification-20260920/zephyrus-809613656896466199/packets.bin)
- [Telemetry check: 600 packets, 76,800 bytes, 43 columns](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/output/verification-20260920/telemetry-verification.json)
- [Build/test evidence](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/output/verification-20260920/build-and-tests.log) and [packaged source/build hashes](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/output/verification-20260920/build-manifest.json)

The console logs all FC actions through `System.out.println`, including accepted physics intervals, sample acquisition/delivery times, measured/estimated state, controller requests, selected PWM, exposed area, commands, and logical peripheral operations. Simulator truth is labeled separately in the physics lines. This follows the user's later request for complete console output, superseding the plan's earlier suggestion to remove per-step output.

## Preserved source behavior and declared differences

| Behavior | Treatment |
| --- | --- |
| Integer airbrake time | Preserved: elapsed milliseconds are divided by integer 1000 before conversion to float. Roll time remains fractional. |
| Prediction/target patches | Preserved: fixed 5046 m prediction, dynamic target floor, source constants, zero-area-to-full-deployment behavior and partial-fit rules. |
| Initialization and repeated preflight | Fresh Java objects reproduce static-storage startup; preflight only performs source resets. One-shot flags, controller history and disabled roll signal are not silently reset for another flight in the same boot. |
| Power command timer | **Correction from the subsequent timing audit:** firmware does update `lastPowerPkt`; Java currently omits the update and sends every loop after boot 100 ms. This is a translation error, also encoded in the existing test, not a source quirk. See [the hardware-contract assessment](flight-computer-simulation-performance-report.md). |
| Telemetry timer | Preserved strict `>50 ms` comparison: 60 ms packet interval at 100 Hz. |
| Malformed uplinks | Checksum, byte order, reserved-byte behavior, ground-state gates and command-read latency retained. Invalid state values and channel masks are rejected instead of permitting undefined access. Manual angles must be finite and within ±180°; nonfinite automatic actuator outputs stop the simulation with a diagnostic. |
| Emergency-all command | Preserved source channels 0–4; channel 5 is not added to that command. |
| Sensor boundary | Deterministic calibrated engineering readings before FC processing; sensor noise off. Source raw-conversion helpers remain available. GPS retains integer 1e-7-degree coordinates and millimeter height before source offset arithmetic. |
| Sample timing | First acceleration evaluation at interval start is captured with its matching pose/location, published only after interval acceptance, then delivered at the next FC tick. Latency is up to one physics interval. Intermediate RK evaluations do not run the FC. |
| Mounting | Assumed sensor X = body Z, sensor Y = body X, sensor Z = body Y. X still drives FC velocity integration while Z separately enters the airbrake library. Board mounting is not claimed to be calibrated. |
| PWM/linkage | Source −67°/−117° mapping gives closed/open pulses 941/525 µs. Integer pulse selection is translated; exposed fraction is an ideal linear endpoint mapping, clamped at the physical component boundary. No mechanical lag model. |
| Recovery and roll | Six logical pyro channels; continuity defaults connected and remains so unless explicitly changed. A timed pulse alone therefore does not imply successful continuity break. Physical recovery follows the ORK configuration. Roll computation runs, but physical roll-tab coupling is off. |
| Peripheral hardware | Nominal power, camera/video command state, bounded flash page and byte queues record side effects. No electrical buses, erase waits or RF propagation are modeled. |
| Scheduler integration | FC-only fixes remove step overrides/floors that could cross deadlines, prevent zero-length launch-rod stalls, preserve ticks during delayed ignition, and offset thrust sample events by ignition time. Auxiliary coast calculations exclude the live FC. Non-FC paths retain their existing behavior and pass selected regressions. |

CSV is decoded from the actual payload bytes, using the historical ground-station schema. All three benchmark headers match exactly. `flight_time` is boot milliseconds; `timestamp` is configurable-epoch simulated boot seconds. RSSI and ground-station location/fix are blank. GPS uncertainty/satellite count and pyro resistance are unmodeled zero; power uses nominal voltages, zero currents and a 20°C BMS temperature.

The raw sensor payload is synthesized with firmware calibration and range clamping. The ground decoder omits the FC's accelerometer-X factor 1.060 and gyro biases, and negates gyro Y. It also uses temperature constants `C5=0x91E3`, `C6=0x6FEC`, whereas the selected FC barometer uses `0x8405`, `0x6D91`. Consequently some decoded CSV values differ from the FC's calibrated engineering values. These conventions are preserved and recorded in each run's metadata; they should not be silently calibrated away during later comparisons.

## Benchmark handoff

The supplied [zephy_testlaunch.ork](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/benchmarking/ork/zephy_testlaunch.ork) was inspected without simulating the actual launch. Its `MITRT N8406` motor resolves using [TestLaunch_v14.eng](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/old/dat/ork/TestLaunch_v14.eng), with no ORK loader warnings.

The prepared [zephy_testlaunch-java-fc.ork](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/examples/zephy_testlaunch-java-fc.ork) selects the existing FC listener and removes the old saved simulation result. The rocket XML and simulation conditions were checked unchanged. The original benchmark and telemetry files are untouched.

The saved site is 28.61°, −80.6°, 0 m; the recorded telemetry is near 42.7042°, −77.1919°. Actual launch-site/weather settings, time alignment, mounting, linkage and recovery assumptions should be established in the next comparison. No recorded flight data were used as controller inputs or used to tune this implementation.

See [run instructions](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/README.md) for GUI launch, headless execution, output paths and telemetry validation. A future JNI implementation can use the saved source provenance and replay fixtures through its separate listener.
