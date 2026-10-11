# Flight computer controller blocks

Planned 11 October 2026 against the current MIT 6.4 source in `clone/openrocket-release-24.12.RC.01`. Implemented in MIT 6.4; see [implementation, usage and verification](13-flight-computer-controller-implementation.md), including the refined output-ownership rule. This document retains the original design proposal.

## Intended experience

In the flight computer designer, add a controller, choose its type, select the measured quantity to control, enter a baseline/setpoint, and select its output. For a fin actuator, choose **Fin tabs** or **Fin cant**, then select an eligible fin set from a dropdown. Several blocks can independently control several fin cans.

Extend the existing designer, FC listener, virtual scheduler and actuator bridge. Keep the controller definition in the external `.fc` library file; keep rocket-specific assignments in the simulation's `.ork` settings, following the existing recovery mapping pattern. All editing labels remain independent of any particular flight computer model.

The first implementation supports single-input feedback blocks with an optional derivative input and velocity input. A block may drive several explicitly assigned actuators with separate direction, trim and limits. Arbitrary mathematical graphs, cascaded controllers and automatic gain tuning are later additions.

## What exists and what must change

| Existing implementation | Consequence for this feature |
| --- | --- |
| `RTRollController` implements PID with aerodynamic gain scaling and servo compensation; `RTAirbrakesController` implements an apogee controller with a sensitivity-scaled, per-update integral. | Preserve these algorithms as existing controller modes. Neither can silently become a generic PID without changing old simulations. |
| `FlightComputerSensors` uses seeded measurement noise and acquisition periods, selecting one active component per sensor role. `RTFC.signal()` exposes measured/estimated values. | Introduce a registry of individual sensor/estimator outputs so dropdowns identify the actual source. Reading a signal must not sample fresh noise. |
| `RTSimulationCommunicator` drives named tab-controlled fin sets and airbrakes after PWM latching. | Reuse this physical path; add stable-ID assignments and fin-cant commands. |
| `FinSet.setCantAngle()` already changes cant, within ±90° in this fork. `FinSetCalc` currently caches cant at construction. | Dynamic cant requires an aerodynamic-cache audit and regression test; calling the setter alone is insufficient evidence that control works. The accepted angle range is not a claim of aerodynamic accuracy at large deflections. |
| Java boards have independent virtual periods/costs, frozen measured inputs, and outputs published at completion. Servos latch every 20 ms. | Controller blocks must follow these same timing contracts. A 1 ms controller does not imply 1 ms sensors or servos. |
| Current Java writes use broadcast/named overrides and last-completed-write behavior. | Introduce explicit ownership for block-controlled outputs, while retaining old behavior for existing designs. |

## Designer and simulation interface

Add **Controller** to the designer library and a **Controllers** tab alongside Hardware and Behavior & timing. A controller appears in the tree and on the canvas, with input/output connections and a concise summary such as `Roll rate → PID → Aft tab servos`. Board placement is organizational; it does not imply that CPU contention or board communications are modeled.

Selecting a block opens these properties:

| Group | Fields |
| --- | --- |
| Identity | Name, controller type, enabled, optional parent board |
| Input | Sensor or estimator dropdown, quantity dropdown, units, coordinate frame; optional measured derivative |
| Baseline | Fixed value, capture after ground calibration, or capture on entry to a selected state |
| Controller | Gains, derivative filter, integral limit, output limits; velocity settings when applicable |
| Operation | Active states, update period, modeled execution cost, stale-input timeout and response |
| Output | Logical servo/output ID, actuation type, direction, trim/neutral and limits |
| This simulation | Eligible physical-component dropdown, assignment status, physical limits; add another target when needed |

Use the same assignment editor in the simulation's **Flight computer** tab. Show names with stage/parent context to distinguish duplicates. In a standalone designer without a rocket, show the logical output and **Assign in a simulation**; do not require a rocket to save a valid `.fc` design.

Provide Add, Duplicate, Delete, Enable and Disable, existing undo/redo and keyboard conventions, and Enter-to-apply for single-line properties. Show **Applied / unapplied changes**, **Valid / needs attention**, and **Mapped / recorded only** separately. Built-in blocks do not need Java compilation. Applying changes invalidates affected simulation results; Cancel restores both design edits and simulation assignments.

## Controller types and baseline semantics

| Type | Meaning |
| --- | --- |
| PID | Conventional feedback with selectable P, PI, PD or PID terms. |
| Velocity-weighted PID | PID with a bounded multiplier based on a separately selected measured/estimated velocity. |
| Constant output | Fixed command during selected states; useful for checking actuator signs and physical coupling. |
| Existing roll controller | Adapter retaining the current aerodynamic gain, inertia and servo-compensation formulas and legacy timing. |
| Existing apogee controller | Adapter retaining the FC-included airbrake algorithm, its predictor, target altitude and integral convention. |

For a new PID, define `e = setpoint − measurement` and `u = trim + direction × (Kp·e + Ki·I − Kd·filteredMeasurementDerivative)`. Integrate using actual virtual elapsed seconds. Display gain units derived from the chosen input and output; changing dimensions clears incompatible gains rather than reinterpreting them. Use derivative-on-measurement to avoid a setpoint kick, with either a compatible measured derivative or a filtered difference of distinct acquired samples. Re-reading a held sample must not repeatedly differentiate it. An angle derivative can come from the corresponding measured angular rate.

Apply finite-value checks, output saturation and anti-windup against the effective output limits, including velocity weighting and target calibration. For fan-out, freeze integration if any assigned saturated target would be driven farther into saturation. Initialize/reset derivative history explicitly and suppress integration on the first invocation. Expose integral reset on state entry; default to reset when activating or changing the baseline. Retaining the integral across states is an explicit advanced setting.

The proposed velocity-weighted mode uses:

```text
w = clamp((referenceSpeed / max(abs(measuredSpeed), speedFloor))^exponent,
          minimumWeight, maximumWeight)
u = trim + direction × w × PID_correction
```

Expose the exponent and bounds; offer inverse-speed and inverse-speed-squared presets. This is a bounded gain schedule, not the existing roll controller's full density/Mach/inertia compensation. Make low-speed behavior explicit: below an activation threshold, output neutral and freeze/reset the integral, with hysteresis to avoid rapid switching. Do not multiply trim by the weight. Apply the stale-input policy to the velocity source too.

The current `velocity` signal is accelerometer-integrated vertical velocity, **not measured airspeed**. Label it accordingly. The first version offers available velocity estimates without inventing a pitot sensor or silently using simulator truth. Do not recommend a gain schedule as physically validated merely because it runs.

Baseline options:

- **Fixed:** a typed value in the input's units, such as roll rate `0 °/s`, roll angle `0°`, or a target altitude with an explicit launch-relative/MSL reference.
- **Ground capture:** latch the first valid selected measurement after calibration; optionally average a configured interval of distinct valid samples. Display the captured value and time.
- **State-entry capture:** latch the latest valid measurement on entry to the chosen state, with an optional numerical offset. If no fresh value exists, remain inactive until the configured input policy is satisfied; never substitute zero.

For cyclic angles, offer shortest-angle error versus continuous/unwrapped turns and test the ±180° boundary. Label the FC frame and positive rotation explicitly. An altitude baseline does not make a direct altitude PID an apogee predictor; use the apogee-controller adapter for that behavior.

## Selecting sensors and values

Add a typed signal registry keyed by stable sensor/estimator ID and channel, rather than display names. Each entry supplies quantity, unit, frame, acquisition time, availability time, validity and the latest held value. The input dropdown includes individual barometers, accelerometers, gyroscopes, GPS receivers and their supported derived estimates.

Refactor `FlightComputerSensors` to acquire each configured source once per due acquisition, keeping stable-ID random streams. Give each source its own held state; derived filters/integrators must also have explicit source IDs and per-run history. Preserve a selected legacy-role adapter for existing firmware and `io.signal(...)` programs. Multiple installed sensors must not make legacy input selection ambiguous.

Expose only implemented channels: measured pressure/temperature/altitude, acceleration axes, angular-rate axes, supported attitude and velocity estimates, and GPS quantities. Separate raw acceleration from gravity-removed acceleration, and raw sensor axes from transformed FC axes. Derived values carry the timestamps/validity of their dependencies. No control input falls back to a matching “real” simulation channel.

Default invalid/stale-input behavior is neutral output and frozen/reset control history. Allow a bounded last-command hold with a configured timeout. Log loss and resumption of validity. Faster controllers reuse held sensor data; their reads neither advance the sensor clock nor consume additional noise draws. Keep legacy acquisition order and noise sequences unchanged for old designs.

## Physical outputs and ownership

| Actuation type | Eligible physical targets and interpretation |
| --- | --- |
| Fin tabs | Active `TabControlledTrapezoidFinSet` components; signed tab deflection, with existing ±90° support and configurable narrower limits. Flag zero tab area or zero aerodynamic coefficient as having no authority. |
| Fin cant | Active supported `FinSet` components; collective cant of the entire selected fin set. Default to an offset from its saved cant, with an explicit absolute-angle option. Final angle respects component limits. |
| Airbrakes | Active `AirbrakeSet` components; exposed fraction 0–1, respecting configured deployment limits. |

Display unavailable/inactive components with a reason. Resolve assignments by UUID in the run's rocket copy; names are labels. Renaming is safe, while deleting a mapped target produces an actionable unresolved-assignment error. Never choose the first matching name or move a binding to a different fin set automatically. Permit an intentional **Recorded only / no physical target** assignment, preserving the existing expectation that unused outputs may be discarded.

Allow several targets per logical output, each with inversion, trim and travel bounds. Keep input baseline separate from actuator neutral/trim. A fin-tab command is distinct from a fin-cant command; the latter does not rotate the fin set around the rocket to a new circumferential position. Individual-fin steering and pitch/yaw mixing are outside this first version.

Add a typed actuator-command layer carrying logical output ID, source, target UUID, mode and timestamp. At run preparation, assign one owner per physical `(component UUID, actuation mode)` in each state. Reject overlapping block ownership unless the active-state sets are disjoint. Claim mapped targets explicitly so the built-in controller cannot also drive them.

For existing custom Java programs, retain named and broadcast APIs. In designs using blocks, broadcasts address only targets not reserved by an active block; an explicit Java write to a reserved target is a clear runtime conflict, not a silent overwrite. Show this rule in the mapping panel and log skipped broadcast targets. Revalidate ownership at state transitions and invalidate unfinished commands from a previous owner. State-machine close/disable actions retain authority to put outputs into their defined inactive condition. Block-inactive behavior defaults to neutral at the next servo latch; indefinite hold must be explicitly selected.

Apply only to the simulation copy. Preserve the saved rocket geometry and restore ordinary fixed cant/tab settings when FC simulation is off. Recovery remains in its separate event/channel mechanism; a continuous controller output must not implicitly fire a recovery channel.

## Timing and physical integration

Implement `FlightComputerControllerRuntime` in the current core FC package. Share the existing virtual-task scheduling mechanics with `JavaBoardRuntime`; do not add a simulation listener, wall-clock timer or controller-specific physics engine.

1. At task start, freeze the selected measured inputs, baseline/state and their timestamps.
2. At `start + executionCost`, publish the calculated command if the block still owns that output and remains active. Use actual start-to-start elapsed time for the control law.
3. Schedule the next start at `max(start + period, completion)`; report overruns without executing a burst of catch-up iterations against one sample.
4. Add task deadlines to the existing listener's next-deadline calculation throughout ascent and descent. Keep deterministic ID ordering and define shared-time processing for old task completions and new snapshots; same-time new starts see the same published sensor state.
5. Preserve the independent 20 ms PWM latch. On an exact completion/PWM tie, PWM sees the previous completed command, as it does today. Physical movement follows the latch with the existing ideal linkage.

Default new blocks to a 10 ms period and explicitly labeled zero modeled execution cost. These are simulation defaults, not measurements of a processor. Blocks use independent virtual schedules like current Java boards; board grouping does not add CPU resource contention or communication delays. Display the existing algorithm-testing limitation without suggesting hardware timing has been qualified.

For dynamic cant, extend `RTSimulationCommunicator` and the latched output representation. Ensure aerodynamic calculations see the current cant despite cached `FinSetCalc` instances, and that geometry transforms are invalidated correctly. Prefer a per-run actuator update path with bounded cache lifetime; do not trigger document/UI rebuilds on every command. Log actual applied angle and verify resulting torque. Continue to use OR's current cant aerodynamic model; this work does not add high-angle aerodynamics or a physical servo response model.

## Persistence, diagnostics and plots

Introduce `.fc` format version 2 for designs using controller blocks, while continuing to read version 1. Add a validated `controllers` section and explicit legacy input/output routing. Stable block IDs link controller properties, canvas layout and logical outputs. Upgrade on an intentional feature edit; do not silently rewrite the supplied baseline or replace legacy algorithms with generic PID. Older applications should reject version 2 rather than run a design while omitting its controllers.

Controller definitions include type, source/channel references, baseline settings, parameters, state gates, timing and logical outputs. Physical assignments live in extension settings such as `controllerActuatorMappings`, with their own schema version, FC design ID and target UUIDs. Validate both halves when switching designs or flight configurations. Include behavioral properties and mappings in freshness/fingerprint checks; labels and canvas positions remain presentation only. Freeze everything per run/ensemble member.

Use the existing trace path, including `System.out.println` and `OR.log`, for configuration, baseline capture, activation, each calculation, output publication/latching, saturation, stale inputs, conflicts and discarded commands. Include input acquisition/start/completion/latch times so total latency can be inspected. Stream traces to disk and retain only bounded UI history; do not accumulate one persistent object per control step in editor caches.

Add per-block plots for measurement, baseline, error, P/I/D terms, velocity weight, requested and limited output, actual physical output, saturation and active/valid state. Carry IDs, names, units and provenance in result metadata. Preserve these in `.ork` results and OR.log import, without requiring the original `.fc`. Keep the existing GS-compatible telemetry packet/CSV unchanged; controller diagnostics belong in the structured log and simulation result channels. Use step interpolation for discrete states and latched actuator outputs.

## Implementation order and completion criteria

1. **Schema and ownership model.** Add typed controller definitions, `.fc` v1/v2 loading, signal references and simulation mapping settings. Specify state ownership and validation before building widgets. Check old-file behavior, semantic fingerprints, missing IDs and renamed components.
2. **Signal registry.** Extend `FlightComputerSensors` and `RTFC` with per-source held readings and estimator adapters. Test distinct sensor selection, seeded noise, timestamps, units/frames, held samples and unchanged legacy signals.
3. **Control math and execution.** Add PID, velocity weighting, constant output and existing-controller adapters; connect deadlines through `FlightControllerSimulatorListener`. Test elapsed-time integration, derivative filtering, wrapped angles, capture baselines, anti-windup, stale data, activation and overruns.
4. **Actuator routing and cant.** Extend `RTFC.Output`, PWM latching and `RTSimulationCommunicator`; update `FinSetCalc`/component handling as necessary. Test target UUIDs, fan-out, independent fin cans, Java/built-in conflicts, disabled blocks and real physical response.
5. **Designer interface.** Extend `FlightComputerDesigner` and `FlightComputerModels`; add Controllers, canvas/tree entries and the shared mapping editor in `FlightComputerPanel`. Verify eligible dropdowns, units, Enter, undo/redo, Apply/Cancel and opening a design without a rocket.
6. **Results and verification.** Extend `FlightComputerData`, plot presets and `FlightComputerLogReader`, plus result persistence where needed. Run focused and existing FC regressions, build root `shadowJar`, and exercise the packaged UI. Produce an implementation report with measured virtual timing and remaining modeling limits.

Acceptance scenarios must include:

- A roll-rate PID reads the selected noisy gyro, maintains a `0 °/s` baseline and changes the roll response of a rocket with nonzero initial spin. Check damping only with a verified sign and deliberately chosen test gains; arbitrary user gains may be unstable.
- Two blocks drive different fin cans independently, including one tab output and one cant output. Changing cant changes physical roll torque, not merely the output graph.
- A constant-output block checks positive/negative deflection and airbrake drag; fan-out/inversion and duplicate names preserve correct target identity.
- A 1 ms block, slower sensor, nonzero execution cost and 20 ms PWM show the expected held input and delayed physical response. Test exact ties, overruns, state changes during execution and post-recovery operation.
- Zero/stale velocity never creates infinite weighting; no valid baseline never becomes an implicit zero baseline. The same seed reproduces noisy control traces.
- Renaming/deleting targets, switching FC files, save/reload, log import, repeated/parallel runs and cancellation preserve isolation. The original rocket stays unchanged and memory use does not grow across completed runs.
- Existing version-1 FC designs and Java programs retain their present controller behavior, except where a user explicitly enables block ownership. Recovery authority and deployment timing remain unchanged.

The feature is complete when a user can build, bind, run and inspect these controllers entirely through the UI, with no Java required for ordinary feedback control.
