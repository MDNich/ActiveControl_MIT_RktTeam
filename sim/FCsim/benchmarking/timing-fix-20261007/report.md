# Zephyrus FC timing corrections and flight verification

7 October 2026. The power-command translation error is fixed. The existing Java FC listener now models nonzero execution time and independent PWM interrupts, so excessive work produces delayed iterations and recorded overruns.

This is verification of the software model. The selected firmware contains two explicit 1.5 ms barometer waits; their combined 3 ms is the new default. No MCU execution-time measurements were supplied. Additional processing, bus, radio, flash and interrupt costs remain assumptions until measured on the flight computer.

## What changed

- The power command updates `lastPowerPkt` immediately before transmission, matching `FC.ino`. The boundary test checks 100, 110, 120, 210 and 220 ms and requires sends only at 110 and 220 ms for immediate execution.
- Each FC iteration has three scheduled phases: start/acquisition, sensor readiness/processing, and control/communications completion. Physics continues between them. The coherent sample selected at loop start is held until readout; readout age is logged.
- The next iteration follows the source's `millis()` busy-wait: `max(completion_us, (floor(start_us / 1000) + 10) * 1000)`. A late iteration starts the next one immediately; no fictitious catch-up iterations are executed. Fractional-millisecond starts can produce a wait slightly shorter than 10 ms in real microseconds, faithfully reflecting integer `millis()`.
- PWM runs independently every 20 ms with a configurable phase. On an exact tie, the interrupt uses the previous completed controller result. GPS fixes become available on an independent 100 ms schedule, with the most recent unread fix selected at loop start. GPS solution and serial transport delays are still unmodeled.
- The existing flight-computer settings box adds sensor readout, additional work, work jitter, PWM phase and a separate timing seed. Values persist in the existing extension and survive save/reopen. Jitter adds a seeded, independent uniform delay from zero through the chosen maximum each iteration; it is a stress input, not measured noise.
- Logs contain `timing.settings`, `timing.loop`, `timing.summary`, `sensors.deliver`, `gps.fix_available`, `gps.consume` and `pwm.latency`. Existing action logs and benchmark-compatible CSV/binary telemetry remain available.

The work is intentionally grouped into phases, not instruction-level MCU execution. Sensor readout defaults to 3 ms; additional work and jitter default to zero. PWM phase defaults to zero. Setting all execution costs to zero provides an ideal comparison, still with independent interrupt ordering. Intra-phase interrupt interleavings, different acquisition instants for each sensor, clock oscillator error, hardware-specific setup/register timing, actuator travel and actual RF transport are not resolved by this model.

## Complete-flight checks with the supplied rocket

Input: [zephy_testlaunch-java-fc.ork](../../examples/zephy_testlaunch-java-fc.ork), first saved simulation. The source file was not modified. All four comparison runs pin both simulator and wind seeds to `20261007`; jitter uses timing seed `42`. Rocket geometry, selected motor, launch conditions and recovery configuration were retained. Runs include one second of pad warmup; counts below include warmup.

The default profile uses 3 ms sensor acquisition and no extra work. Fixed overrun uses 3 + 10 ms with PWM phase 7 ms. Variable work uses 3 + 3 ms plus uniform 0–9 ms jitter, with PWM phase 6.371 ms. These are deliberately chosen test profiles, not inferred flight-hardware loads.

| Profile | Execution time (ms) | Start-to-start interval (ms) | Overruns / completed loops | Apogee (m) | Maximum sample age at PWM (ms) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Ideal comparison | 0 | 10 | 0 / 13,298 | 4472.035 | 12.500 |
| Default: known sensor waits | 3 | 10 | 0 / 13,299 | 4472.352 | 12.393 |
| Fixed overrun stress | 13 | 13 | 10,237 / 10,237 | 4475.886 | 27.500 |
| Variable work stress | 6–14.999 | 9.001–14.999 | 6,906 / 11,803 | 4473.542 | 30.837 |

All four flights completed and reached FC state `MAIN`. A second default flight produced identical simulated state samples (excluding the desktop wall-clock “Computation time” column) and byte-for-byte identical received/transmitted telemetry; its saved `.ork` explicitly records the timing settings. The default apogee is 0.317 m above the ideal comparison. This small change in this flight is not evidence that hardware timing is always unimportant. Different execution costs can change the inputs, controller decisions and filter delays.

The default profile delivered sensor samples aged 3.000–5.393 ms, versus 0–2.500 ms in the ideal comparison. The age at PWM includes retained-sample age, modeled execution and wait for the PWM interrupt. It does not include barometer filtering or mechanical servo travel. The existing 20-sample barometer filter alone adds nominal 95 ms linear-filter delay at 100 Hz, and that delay changes when iterations slow down.

| Check | Result |
| --- | --- |
| Power-command timer | Default: 1,209 sends, every 110 ms, first at boot 103 ms because work completes 3 ms after loop start. Pre-change run: 13,286 sends, every 10 ms after boot 110 ms. |
| Default telemetry | 2,216 packets, every 60 ms, first at boot 53 ms. |
| Fixed overrun telemetry | 2,559 packets, every 52 ms: four 13 ms iterations satisfy strict `>50 ms`. |
| Jitter telemetry | 2,356 packets; every send satisfies strict `>50 ms` using the firmware's integer clock. |
| PWM and GPS | Every scheduled PWM interval is 20,000 µs in every profile; GPS-fix availability remains 100,000 µs. |
| Pyro polling | All six channels fired and shut off. Default/ideal pulses: 260 ms; fixed overrun: 263 ms; jitter: 252.227–259.879 ms. Every shutoff satisfies source `millis()` elapsed `>250 ms`. No maximum-250-ms requirement was invented or silently imposed. |
| Clock and deadlines | Monotonic virtual clock, no skipped scheduled event, and each next loop obeys the source wait/overrun rule. |
| Telemetry integrity | Independent verification passed for all profiles: 43-column benchmark schema, 128-byte payloads, checksums, decoded fields, sequence, receiver/transmitter correspondence and timing. |
| Persistence | All four complete flight histories were saved to separate `.ork` files. Timing settings also passed save/reopen and clone tests. |

The pre-change baseline did not pin its wind seed and is used only to demonstrate the power-send bug; its trajectory is not used for a paired comparison. The ideal comparison above uses the corrected code with zero execution costs and the same pinned seed as the delayed runs.

## Validation and outputs

The existing 32 FC tests passed after the runtime change. Six new timing tests passed, covering defaults, fixed overruns, shared PWM/completion deadlines, deterministic jitter at 2.5/1 ms physics steps, 1 µs deadlines under RK4/RK6 and validation of settings. The existing persistence test now also checks timing settings. Three Swing checks passed, including the existing simulation-options layout/progress tests and timing-control editing, bounds and alignment. The application JAR rebuilt successfully and its startup classes, checksum and installed-launcher copy were checked.

- [Machine-readable timing results](timing-results.json) and [independent timing checker](check_timing.py).
- Local saved flights: `zephyrus-default-repeat.ork`, `zephyrus-ideal.ork`, `zephyrus-overrun.ork`, and `zephyrus-jitter.ork`.
- Local profile directories contain `OR.log`, received/transmitted CSV, raw packets, metadata and `telemetry-verification.json` in their `zephyrus-*` subfolders. These generated artifacts are retained locally rather than committed; the summary, checker and reproduction commands are versioned.
- [Build checksum](build-sha256.txt).

To reproduce, use the existing runner from the repository root, with JDK 17 on PATH. Choose a new output directory and a new output `.ork` name:

```bash
FC_OUTPUT_DIR=/path/to/new/output bash sim/FCsim/run-java-fc.sh \
  --run sim/FCsim/examples/zephy_testlaunch-java-fc.ork \
  --simulation-seed=20261007 --save-ork=/path/to/new/result.ork
```

Add `--sensor-us=0` for the ideal comparison; `--work-us=10000 --pwm-phase-us=7000` for fixed overrun; or `--work-us=3000 --jitter-us=9000 --pwm-phase-us=6371 --timing-seed=42` for variable work. Receiver loss/delay flags remain independent. The runner resolves the current full launcher JAR; `FC_JAR` can select another compatible build. `--expected-period-ms=60` is an optional exact-cadence check in the independent telemetry verifier; variable-work runs instead check the source's strict `>50 ms` contract.

Actual hardware deadline qualification still requires loop-entry/exit, sensor-ready, PWM and communication timestamps measured on the board. Those measurements can now populate the timing controls and exercise overruns instead of being hidden by an always-instantaneous FC.
