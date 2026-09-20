# Plan: flight computer checkbox and telemetry settings

Implemented in 6.2; measured checks and remaining platform smoke tests are in [the implementation report](03-implementation-report.md).

## User-facing result

In **Edit simulation → Simulation options**, place the titled **Flight computer simulation** box below **Simulator options** in the left column. This supersedes the initial placement under Add extensions. The right column remains available for other extensions:

```text
[ Simulator options ]

Flight computer simulation
  [ ] Enable Zephyrus flight computer
      Packet loss       [ 0.0 ] %
      Downlink delay    [   0 ] ms
      Random seed       [   1 ]

  Loss and delay apply to received telemetry.
  Received telemetry CSV [ folder / filename ] [ Browse… ]
  FC simulation log      [ folder / filename ] [ Browse… ]
  Leave paths empty for a new folder each run.
```

Disable parameter widgets when the checkbox is off, retaining their values. Use ordinary labeled Swing controls, keyboard navigation, unit labels and localized tooltips. Avoid requiring users to enter a Java class name. The existing extension list remains usable for other extensions.

| Setting | Stored value | Validation and meaning |
| --- | --- | --- |
| Enabled | Boolean, default false | Controls installation of the existing FC listener |
| Packet loss | Fraction, default 0 | UI 0–100%; independent probability of dropping each transmitted downlink packet |
| Downlink delay | Integer milliseconds, default 0 | UI 0–10,000 ms; fixed delay for each surviving packet |
| Random seed | Integer, default 1 | UI 0–2,147,483,647; a fresh run reproduces its loss sequence |

The delay range is an initial UI limit, not a claimed radio specification. Neither loss nor delay is a model of RF propagation or CPU execution time. Keep FC loop/PWM periods out of this box: those are firmware contracts, not tuning knobs.

## Existing code to extend

Source root: `/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket-release-24.12.RC.01`.

| Location relative to source root | Planned responsibility |
| --- | --- |
| `swing/src/main/java/info/openrocket/swing/gui/simulation/SimulationOptionsPanel.java` | Insert the box; synchronize managed FC configuration and extension list |
| `swing/src/main/java/info/openrocket/swing/gui/simulation/SimulationConfigDialog.java` | Preserve save/cancel, cloning and multi-simulation editing behavior |
| `core/src/main/java/info/openrocket/core/simulation/extension/impl/` | Add `ZephyrusFlightComputer` and its standard extension provider |
| `core/src/main/java/info/openrocket/core/document/Simulation.java` | Notify configuration changes and mark existing results outdated when FC settings change |
| `core/src/main/java/info/openrocket/core/simulation/listeners/FlightControllerSimulatorListener.java` | Accept immutable run settings and pass them into the existing FC/telemetry objects |
| `core/src/main/java/edu/mit/rocket_team/zephyrus/telemetry/RTTelemetryEngine.java` | Apply loss/delay after packet construction; preserve transmitted bytes and produce received output |
| `core/src/main/java/edu/mit/rocket_team/zephyrus/FC/RTFC.java` | Pass settings to existing telemetry initialization without changing loop order or controller logic |
| `core/src/main/resources/l10n/messages.properties` | Labels, units, help and validation messages |

Also inspect/update the existing CLI runner and verifier under `/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/` so they honor the same settings and output semantics.

## 1. Persist settings through the extension system

- [x] Add a small `AbstractSimulationExtension` subclass with typed accessors for `enabled`, `packetLossFraction`, `downlinkDelayMs`, `randomSeed` and a configuration version. Validate both GUI changes and values loaded from files.
- [x] Register its provider using the existing plugin mechanism. Persist through the existing `<extension>`/`Config` serializer; no separate top-level `.ork` settings format is needed.
- [x] When enabled, create the existing `FlightControllerSimulatorListener` with immutable settings. When disabled, install no FC listener. Keep the disabled extension configuration so toggling off/on retains parameters.
- [x] Keep the no-argument listener constructor and current CLI entry path functional with zero loss/delay defaults. Create clocks, random generators, pending packets and output streams per run; never share them through extension clones or global state.
- [x] Add a narrow extension-change notification path in `Simulation`, covering managed extension insertion/removal and changes to its config. Mark results outdated, retain the old results for inspection, and notify the document. Ensure clone/load/cancel restores both settings and status. Merely changing the visible checkbox must not leave results marked current.

Completion: enabled and disabled settings survive save/reopen, duplicate simulation and cancel/reopen; changing a setting marks a previously run simulation outdated.

## 2. Support existing FC-enabled files

Existing files can have a generic `JavaCode` extension naming exactly `info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener`.

- [x] Recognize that exact class as an enabled legacy FC configuration with zero-loss/delay defaults. Inspecting a file or opening its dialog should not silently rewrite it.
- [x] When the user changes the checkbox or parameters, replace only matching legacy FC entries with one managed extension. Use the dialog's existing snapshot/undo path so Cancel restores the original list.
- [x] Present one FC control surface; do not also show the managed FC as an independently removable duplicate in the generic list. Keep generic extension copy/menu operations synchronized with the box.
- [x] Before execution, verify that the resolved listeners contain at most one FC simulator, including listeners supplied by the CLI. Reject unresolved duplicate installations with a clear message before opening output files or mutating the rocket. Never run two FC instances against the same airbrakes.
- [x] Preserve unrelated Java listeners. In particular, do not enable or modify the separate `AirbrakesControllerListener`.
- [x] Preserve current missing-airbrake and unsupported-stage checks. Explain a configuration error in the normal simulation error UI; do not create airbrake geometry automatically.

Completion: the supplied legacy benchmark file, a new simulation and a file containing other extensions all produce the expected checkbox state and exactly one or zero FC instances.

## 3. Add deterministic received-telemetry behavior

Apply link effects after `RTFC.constructTelemetryPacket()` and before the receiver export. Flash logging continues to receive the original FC packet regardless of downlink loss.

For every generated packet:

1. Assign a run-local transmission index and capture immutable payload bytes and transmission boot time. Preserve firmware packet number, boot time and checksum.
2. Append to the complete transmitted stream.
3. Draw one value from a dedicated seeded generator; drop if it is below the configured loss fraction. Count and log the decision. Keep the algorithm/order stable, with one draw per transmission even at 0% or 100%.
4. Otherwise queue a receive event at `txBootUs + delayMs * 1000`. Fixed delay preserves order; the transmission index resolves ties.
5. Export due receive events with their scheduled arrival timestamp. Delivering a queue during a later FC tick must not quantize the recorded arrival time to that tick.

The queue uses simulated timestamps and never sleeps. It does not advance the FC clock, delay an FC iteration or modify sensors. At normal simulation completion, export surviving pending arrivals at their scheduled times without extending physical flight or executing extra FC loops. On cancellation/failure, mark output incomplete and report pending packets rather than fabricating a completed run.

### File contract

CSV and log selectors accept independent absolute paths. Custom CSV `flight.csv` produces sibling `flight-packets.bin`, `flight-transmitted.csv`, `flight-transmitted-packets.bin` and `flight-metadata.txt`. The default log then becomes `flight.log`; an explicitly selected log path overrides it. Parents are created. Existing files are never overwritten; choose fresh filenames for another run. See the implementation report for batch-run constraints.


| File in the existing per-run directory | Contents |
| --- | --- |
| `telemetry.csv` | Received packets using the existing 43-column GS schema |
| `packets.bin` | Matching received 128-byte payloads, in CSV order |
| `transmitted-telemetry.csv` | Every generated packet, timestamped at transmission |
| `transmitted-packets.bin` | Every generated 128-byte payload |
| `OR.log` | Copy of FC action lines also printed to the console |
| `metadata.txt` | Settings, random algorithm/version, seed, generated/received/dropped/pending counts, completion state and timestamp definitions |

For received CSV, `timestamp = configuredEpoch + scheduledArrivalBootUs / 1e6`; payload `flight_time` remains the firmware transmission-time field. Do not renumber received packets or insert fake rows for dropped packets. A 100% loss run has a header-only received CSV and an empty received binary, with complete transmitted output. At zero loss and zero delay, received output remains compatible with the current files.

Continue `System.out.println` action logging: settings at startup; each transmitted/dropped/received packet with transmission index and times; summary counts and absolute output paths. A delayed receive log should carry its explicit receive time rather than pretending that the FC clock advanced to it. Show the received CSV path through the normal simulation-completion UI as well as the console.

Completion: changes in link parameters affect only receiver output and link logs. For the same physical run, FC states, PWM requests, transmitted packet bytes and trajectory are identical across link settings.

## 4. Focused verification and delivery

- [x] Test 0% and 100% loss exactly; fixed-seed reproducibility; clone/run isolation; timestamp arithmetic; delayed packets after normal completion; cancellation and output close/error paths.
- [x] Verify packet/checksum preservation and pair each CSV with its corresponding binary. Update `verify-telemetry.py` to accept legitimate receiver gaps and empty receiver output while retaining strict validation of transmitted packets.
- [x] Check below/at/above input limits, invalid persisted values and duplicate listeners.
- [x] Compare an otherwise identical run at zero/default settings and nonzero loss/delay. Require identical FC tick/PWM sequences and physical results; only received telemetry changes.
- [ ] Complete a manual cross-platform sweep of editor enable/disable, save/load, Cancel, duplicate simulation and editing several selected simulations. Follow the existing dialog's apply-to-all behavior, cloning config rather than sharing mutable objects.
- [x] Run focused core tests and the affected Swing checks; build root `shadowJar`. Launch the packaged application from the CLI and check the box, resulting files and logs.

The earlier power-timer bug and hardware execution/interrupt timing remain deferred. Link delay is expressly a receiver effect and must not be presented as a fix for those issues.
