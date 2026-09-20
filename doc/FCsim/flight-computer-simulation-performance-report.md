# Flight computer simulation: hardware-contract assessment

20 September 2026. Assessed the supplied run `zephyrus-2679935281387596436` against the selected FC sketch and libraries, emphasizing time contracts.

**Verdict: partial compliance.** The simulation reproduces the nominal FC, PWM, telemetry and GPS schedules in virtual time. A power-command timer translation error violates the source contract. Hardware execution time, interrupt phase and peripheral latency are not represented, so the current implementation cannot establish that the physical FC meets its deadlines.

All selected firmware file hashes still match the [source manifest](java-translation-source-manifest.json). The power-timer discrepancy is therefore an implementation error, not source drift. The selected revision has not independently been established as the exact firmware flown in GS1.

## Contract results

“Pass” below means the stated virtual-time property was observed or tested. It does not mean the corresponding physical hardware timing has been measured.

| Contract | Evidence | Assessment |
| --- | --- | --- |
| Nominal FC iteration: 10 ms | All 13,295 intervals between 13,296 logged loop starts are exactly 10,000 µs. The listener prevents physics steps from skipping an FC deadline. | **Pass: nominal schedule.** Hardware only waits while elapsed time is below 10 ms; an overrun can make an actual iteration longer. The simulator assumes no overruns. |
| PWM refresh: 20 ms | All intervals between distinct logged PWM latches are exactly 20,000 µs. One additional initialization write occurs at boot zero. | **Pass: refresh period. Partial: phase/latency.** Firmware uses an independent timer interrupt; Java latches after the FC loop on aligned 20 ms boundaries. Interrupt ordering, compare-register behavior and servo travel time are not modeled. |
| Telemetry: strict elapsed time `>50 ms` | 2,215 packets, every 60 ms, independently validated against the binary payload and checksum. | **Pass: nominal packet generation.** At exact 10 ms iterations, strict `>50` gives 60 ms, not 50 ms. Radio airtime, blocking, buffering and delivery delay are unqualified. |
| GPS configuration: 10 Hz | 1,330 fresh simulated fixes, every 100 ms. | **Pass: nominal update rate.** Receiver solution latency, serial delivery and real fix dropouts are not simulated in this run. |
| Power commands: strict elapsed time `>100 ms`, then advance timer | Firmware assigns `lastPowerPkt = millis()` when sending. Java omits that assignment. The run sends 13,285 commands, beginning at 110 ms and then every 10 ms. | **FAIL.** On the simulator's nominal clock the correct interval is 110 ms. Existing test `sourcePowerTimerRemainsUnadvanced` asserts the erroneous behavior and must be corrected with the implementation. |
| Pyro switch-off: strict elapsed time `>250 ms` | All six logged fire-to-off intervals are 260 ms. Existing boundary test retains firing at 250 ms and turns it off when checked at 251 ms. | **Pass: source polling semantics.** A 10 ms polling loop gives 260 ms for tick-aligned firing. This would fail a separate hardware requirement of a maximum 250 ms pulse; the code alone does not establish such a requirement. Electrical behavior and recovery actuation are outside this model. |
| Staged deployment timers | The run fires channels 3/4 at apogee +3,010 ms and channel 5 at +5,010 ms. Existing tests exercise strict 26 s/35 s apogee and 55 s main gates. | **Pass on the exercised logical boundaries.** Main deployment also depends on sensor/state conditions, so it need not occur immediately after its lockout. |
| Causal sensor delivery | Logged adapter acquisition-to-delivery age is 0–2,500 µs; no future-dated deliveries. | **Pass: simulation causality.** This is not total sensor age: it excludes conversion waits, filtering and the age of a retained GPS fix. |
| Sensor conversion and processing time | Firmware barometer performs two explicit 1,500 µs waits per loop, before bus/compute costs. The simulated clock does not advance during FC function calls. | **Not represented.** At least 3 ms of source conversion waits per hardware loop are absent from the execution-time model. The 20-sample barometer average is implemented; its nominal linear-filter delay is 95 ms at 100 Hz. |
| Clock arithmetic and controller time units | Unsigned 32-bit elapsed arithmetic and a microsecond-wrap integration test are present. Airbrake input time preserves integer division by 1000; roll time remains fractional. | **Verified for inspected code and selected tests.** Whole-FC operation across all clock wraps and actual oscillator error are not established by this short run. |

The simulated FC consumes samples, handles state/controllers, generates telemetry, and then reads incoming commands in source order. Tests cover the resulting next-iteration response for received state commands. Real command arrival, parsing time and interrupt interleavings remain unmodeled.

## Qualification limits that matter

The hardware loop is paced by `millis()` after doing its work. It is not a hard guarantee of exactly 100 Hz. Barometer waits alone consume roughly 30% of a nominal 10 ms period. SPI/UART work, controller computation, radio transmission, flash operations and interrupts use additional time. The Java implementation assigns these operations zero virtual duration, so its perfect cadence cannot reveal missed hardware deadlines or reproduce intra-loop sensor timestamps.

Likewise, a 20 ms PWM period does not prove a 20 ms sensor-to-actuator response. That response includes sample age, FC scheduling/computation, wait for the timer update and mechanical movement. The current simulation implements a chosen synchronous ordering and ideal actuation. Hardware timer phase and setup delays require separate evidence.

The earlier implementation verification reported 33 passing tests and 16,004 C++/Java reference rows. The reference rows cover the airbrake and roll libraries, not the complete FC loop and peripheral timing. The discovered power-timer error demonstrates why passing tests derived from the translation cannot independently establish every firmware contract. This audit does not change the runtime or the supplied JAR.

## Actions needed to close the timing assessment

1. Correct Java's power timer and replace the incorrect expectation with checks at 100, 110, 120, 210 and 220 ms: sends only at 110 and 220 ms on this nominal schedule. Rebuild and verify the resulting console intervals.
2. Instrument the physical FC with loop-entry/exit timestamps and an observable PWM-update signal. Capture acquisition/ready times, command acceptance, telemetry enqueue/transmit and pyro transitions. Measure execution time, start-to-start jitter, overruns and sensor-to-output latency through busy telemetry/flash/controller periods. Ground-station packets at roughly 60 ms spacing cannot resolve these 10 ms contracts.
3. Use those measured durations and interrupt phases to exercise the existing simulator/test harness with delayed sensors, missed iterations and independent PWM timing. State explicit allowed bounds before claiming a timing contract passes. Preserve the distinction between code semantics (for example `>250 ms`) and any stricter electrical requirement.

## Flight fidelity and desktop speed

These remain separate from timing compliance. With barometric height as the primary reference, simulated apogee is 940 m (17.4%) low; a 580 m deficit already exists at 10 s, before braking. GPS altitude retains very low confidence. Airbrake command disagreement averages 61.4 µs over the active comparison window, but the real and simulated controllers received different inputs, so this does not isolate a translation error.

The log reports approximately 10 seconds of desktop execution for 132 seconds of simulated flight, about 13 times real time. That coarse whole-simulation measurement is useful for turnaround, not embedded CPU deadline qualification.

Detailed measurements: [timing and command metrics](../../sim/FCsim/benchmarking/comparison-20260920-2679935281387596436/fc-qualification-metrics.json), [flight comparison](../../sim/FCsim/benchmarking/comparison-20260920-2679935281387596436/comparison-report.md), and [earlier implementation verification](java-translation-implementation-report.md).

Source locations: [FC loop and power timer](/Users/mdn/Developer/MIT_Rkt_Team/Avionics2025/RT_Arduino/Zephyrus/FC/FC.ino:140), [PWM timer](/Users/mdn/Developer/MIT_Rkt_Team/Avionics2025/RT_Arduino/Zephyrus/FC/FC.ino:107), [barometer conversion waits](/Users/mdn/Developer/MIT_Rkt_Team/Avionics2025/RT_Firmware_Libs/baro.cpp:20), [Java power-timer discrepancy](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket/core/src/main/java/edu/mit/rocket_team/zephyrus/FC/RTFC.java:89).
