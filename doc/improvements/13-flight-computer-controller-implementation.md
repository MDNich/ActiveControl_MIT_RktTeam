# Flight computer controller blocks — implementation and verification

Implemented on 11 October 2026 in OpenRocket MIT 6.4, extending the existing FC designer, simulation listener, virtual scheduler and actuator bridge. This implements the Signal flow interface selected after [the controller-block plan](12-flight-computer-controller-blocks.md).

## Using the interface

1. Open a simulation's **Flight computer** tab and open its `.fc` file in the designer.
2. Select **Controllers → Add controller** (also available as Controller in the component library). New blocks start disabled with zero gains.
3. In **Setup**, name the block, choose its type, select a measured sensor and quantity, and enter the baseline. Optional board grouping is organizational.
4. Choose **Fin tabs**, **Fin cant** or **Airbrakes**. Select the rocket component under **This simulation — physical assignments**. Add more targets for fan-out. Each target has direction, trim and limits; cant supports an absolute angle or an offset from the saved cant. A block without a physical assignment is recorded only.
5. In **Tuning**, enter gains, logical neutral/trim and limits. Velocity-weighted PID additionally exposes a velocity source, reference speed, floor, exponent, weight bounds and activation hysteresis. Exponent 1 gives inverse-speed weighting; exponent 2 gives inverse-speed-squared weighting.
6. In **Timing & states**, select active states and set period, modeled execution cost, stale-input timeout and optional invalid-input hold. Baseline capture can average distinct sensor acquisitions.
7. Enable the block, **Apply controller**, then **Save** the `.fc`. Enter applies text/numeric properties; Ctrl/Command+Enter applies the block. Add, duplicate, delete and undo/redo are supported. The footer distinguishes unapplied changes, validation status and assigned/recorded-only outputs.
8. Run the simulation. **Plot data** includes per-controller tracking, PID terms, output and status presets, plus per-actuator commanded/physical output presets. Individual channels remain available for custom plots.

The central diagram shows **measured input → controller → actuator**, with held input and 20 ms PWM annotations. The simulation's Flight computer tab also has the same actuator assignment editor, so changing rocket-specific assignments does not require changing the portable controller. **Clear all actuator assignments** repairs stale assignments after switching designs or deleting controllers outside the editor.

The designer's existing Save/Discard lifecycle applies to both controller definitions and draft mappings. Applying a controller changes the editor's draft; saving writes the external file and its simulation assignments. Changes in the simulation-side assignment panel are saved with its Apply button. No Java compilation is required for these blocks.

## Controller behavior

| Mode | Behavior |
| --- | --- |
| PID | P/PI/PD/PID with elapsed virtual seconds, derivative on measurement, optional compatible measured derivative, filtering, integral limit and conditional anti-windup. |
| Velocity-weighted PID | Generic PID correction multiplied by a bounded inverse-speed power. Trim is not multiplied. Speed floor, activation threshold and hysteresis handle low speed. |
| Constant output | Fixed command plus trim/direction in active states; separate neutral outside those states. |
| Existing roll controller | Uses the current FC roll algorithm, including its aerodynamic scaling, internal limit and servo compensation. Selected gyro angle/rate, selected velocity estimate, and primary barometer height provide its inputs. |
| Existing apogee controller | Uses the FC-included airbrake algorithm with the selected barometric altitude, selected accelerometer velocity/Z acceleration and explicit target altitude. Retains its integer flight-second input and per-update integral convention. |

Generic PID uses `error = baseline − measurement`. Shortest-angle error is optional. A fixed baseline is explicit; ground capture waits for calibration; state-entry capture waits for a fresh measurement. Missing/stale data never becomes an implicit zero baseline. Default invalid-input behavior is neutral with reset history; a bounded hold is optional.

For generic PID, anti-windup considers logical limits and each assigned actuator's calibrated physical limits. Invalid fan-out calibrations with no common command range are rejected. Existing-controller adapters retain their own internal control laws and anti-windup. Their generic integral-limit/filter controls are hidden and the UI describes that distinction. Their plotted P/I/D contributions come from the actual legacy algorithms before internal limits; the apogee error is a predicted-apogee error, available when that algorithm is controlling.

Sensors are selected by stable ID. Measurements include the existing seeded noise, bias, mounting rotation, held acquisitions and per-source estimator histories. Re-reading a signal does not draw new noise. Additional sensors have independent histories/random streams. Existing firmware and Java `io.signal(...)` programs retain their selected primary inputs. When a file first gains controllers, those primary selections are made explicit as `legacyInputs`.

The current velocity estimate integrates longitudinal accelerometer X after gravity subtraction; it is not a pitot measurement or true airspeed. Gyro roll is minus sensor X; labels identify that convention. Sensor acceptance still follows the FC read cadence, and GPS inputs follow the existing GPS fix delivery path. A faster block does not increase sensor bandwidth. GPS sample age retains the original fix acquisition timestamp.

## Physical ownership and persistence

Definitions are stored in optional `.fc` **version 2** `controllers`; old version-1 files continue to load without changing their algorithms. The supplied baseline is not automatically rewritten. Physical bindings are stored under the simulation extension's `controllerActuatorMappings` setting in the `.ork`, using the FC design ID, controller ID and component UUID. Renaming a component does not change its assignment. Missing/inactive/incompatible targets and overlapping owners are rejected.

A deliberate refinement to the original proposal: **an enabled, mapped block reserves its target even outside its active states**, when it supplies neutral or an explicitly selected held output. This prevents a legacy loop or Java broadcast from unexpectedly taking over during a state transition. Explicit named Java writes to reserved targets are rejected with the owning block named; broadcasts skip reserved targets. Disabled or unmapped blocks do not claim hardware. Multiple blocks may share an actuator only with disjoint active states. Between owners, the previous owner's hold/neutral policy applies until another becomes active.

Tabs and cant are separate control modes, including on the same fin set. Both use the per-flight rocket copy. Cant now reaches the existing aerodynamic calculator dynamically instead of a constructor-cached value. Actuator movement is idealized at the PWM latch; no servo slew model was added. Existing FC recovery authority remains unchanged, and continuous controller outputs cannot fire recovery channels.

The Signal flow view lives in the new Controllers tab; the hardware tree lists controllers as well. It is a single-input controller editor, not an arbitrary mathematical graph or automatic controller tuner.

## Time contracts and traces

- Each block runs on its own virtual schedule, with frozen measured inputs at invocation start and output publication after its modeled execution cost.
- The next start is `max(previous start + period, previous completion)`. Overruns do not cause overlapping calls or a catch-up burst.
- Blocks and custom Java programs share the existing listener's deadline dispatcher. Board grouping does not simulate shared CPU contention or inter-board communication.
- Servo values latch every **20,000 µs**. At an exact completion/PWM tie, the interrupt sees the previous completed command; the new command waits for the next latch.
- State changes invalidate work started in the previous state. Explicit controller-close actions also invalidate pending work. A block can run in an intermediate/recovery state if that state is selected.
- Logs report `controller.start`, `controller.update`, `controller.baseline`, `controller.state`, `controller.discard`, `controller.close`, bindings and physical application. Start/update records include sample/start/completion timestamps; physical records include latch timestamps.

All records use the existing console/`OR.log` trace path. `fc.controller_values` adds time-stamped plot samples, while plot metadata carries stable symbols, names, units and sensor provenance. Controller and physical-actuator channels survive log import and `.ork` result persistence without requiring the original `.fc`. Discrete status and command channels use step interpolation. Existing antenna-compatible telemetry packets and CSV columns are unchanged.

Controller histories and plot-channel caches belong to a run; they do not retain a growing history of every controller invocation. The simulation result still stores its plotted samples, and detailed traces can be large at short periods.

## Verification

The automated controller tests cover:

- Version-1 compatibility and version-2 round trips; explicit primary inputs and independent additional sensors.
- Frozen inputs, acquisition timestamps, deterministic measurement noise, GPS fix age and elapsed-time integration.
- A 1 ms block with 20 ms execution cost: completion at 20 ms, the coincident PWM still holds the old value, and the new value appears at 40 ms. Overrun scheduling starts the next invocation at completion.
- Stale-input neutral, zero-speed weighting, activation thresholds, ground baseline capture, angle wrapping and state changes during execution, including control after entering an apogee state.
- Anti-windup at asymmetric target limits; ownership validation, stable UUIDs across renames and explicit Java conflicts.
- Independent tab/cant outputs, changing cant torque through an already-created aerodynamic calculator, airbrake drag, controller log import and `.ork` result round trips.
- Closed-loop roll damping on the verification rocket: from an initial 3 rad/s, the final roll rate at 2 s is **0.0957 rad/s with the test controller**, versus **2.8236 rad/s in the comparison run**. These are regression-test gains for that rocket, not a general tuning recommendation.
- Swing editing, Enter/Apply, save/reload of definitions plus bindings, undo, and rendered Setup/Tuning/Timing views.

The final broad regression run passed **90 core tests and 9 Swing/plot tests**. Root `shadowJar`, its checksum and the installed-app compatibility alias were verified. Native packaged verification also opens the actual designer and simulation dialog, runs twice with a **1,000 µs period / 1,500 µs execution cost**, checks deterministic results with separate log files, imports the controller channels, saves a demonstration `.ork`, and verifies that the original fin remains unchanged. The packaged logs confirm 1,500 µs between starts, 1,500 µs execution with 500 µs overrun, and physical updates only on 20 ms boundaries.

Build artifact: `clone/openrocket-release-24.12.RC.01/build/libs/OpenRocket-MIT-v6.4.jar`. The build also refreshes the compatibility JAR referenced by `/Applications/OpenRocket_MIT.app`; restart the application to load the new code.

Repeatable packaged verification: `sim/FCsim/ControllerDesktopVerification.java`. Its generated files, screenshots and summary are in `clone/openrocket-release-24.12.RC.01/build/controller-desktop-verification/`.

This validates the implemented virtual timing and physical routing. It does not qualify a real processor, communications link, servo or high-deflection aerodynamic model.
