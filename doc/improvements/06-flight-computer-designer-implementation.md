# Flight-computer designer: implementation and use

7 October 2026. Implemented in the existing OpenRocket source; current local version is **6.4** (`OpenRocket-MIT-v6.4.jar`). This change does not publish a release or produce new platform installers.

## Open the designer

1. Restart OpenRocket to load the rebuilt application.
2. Open **Edit simulation → Flight computer** (a dedicated tab).
3. Choose **Load .fc…** to import a design, or **New…** to make an independent flight-computer design using the current timing settings.
4. Choose **Edit…** to open its hardware, behavior, and test views. With a legacy simulation and no selected file, Edit opens the protected Zephyrus template for inspection.
5. Enable the flight-computer checkbox to use the selected design in the simulation.

Editor controls are model-agnostic. The computer dropdown shows the built-in controller, Iris, and Balius; imported design names remain as authored. Iris and Balius are labeled unavailable; their firmware and hardware definitions have not been invented.

## Standard flight-computer library

| Platform | Default directory |
| --- | --- |
| macOS | `~/Library/Application Support/OpenRocket/FlightComputers/` |
| Windows | `%APPDATA%\OpenRocket\FlightComputers\` (home-directory fallback follows OpenRocket's existing policy) |
| Linux/Unix | `~/.openrocket/FlightComputers/` |

**Loading an external `.fc` imports a copy into this library.** The simulation then references the library copy; editing it does not edit the original import source.

- Byte-identical imports reuse the existing library file.
- An existing filename with different contents is preserved; the imported copy receives a hash suffix, with another suffix if necessary.
- Files already in the library are reused.
- New designs and Save as copies receive independent design IDs.
- **Library…** and **Library folder** open the directory.
- Tests/advanced deployments can override it with `-Dopenrocket.fc.library=/absolute/path`.

The packaged baseline is installed as `Templates/zephyrus.fc`. Saving a protected template creates an independent copy. Template installation preserves a manually changed or older template and uses an alternate fingerprint-suffixed filename if necessary.

Source-controlled baseline: [zephyrus.fc](../../clone/openrocket-release-24.12.RC.01/core/src/main/resources/flightcomputers/zephyrus.fc). Board groupings and bus/resource notes come from the FC sketch. This is a functional representation, not a verified PCB schematic; unspecified chip identities remain unspecified.

## External-file persistence

New configurations store a `library:` reference, design ID, and computer family in `.ork`, alongside simulation-specific telemetry/link/output settings. Hardware, rules, layout, sensor settings, and timing parameters remain exclusively in `.fc`.

This managed-library policy supersedes the earlier project-relative default-path proposal. Share the `.ork` and export its `.fc`; the recipient imports/selects the latter. A missing or renamed reference requires explicit location/selection rather than substitution.

Results record the design ID, computer/model ID, exact-file SHA-256, semantic fingerprint, and library reference. These survive `.ork` save/reopen, including individual ensemble archives. A changed/missing functional definition marks existing results outdated while preserving them. Label/layout-only changes do not alter the semantic fingerprint.

The enabled design is validated and frozen at run start. All N ensemble members use that immutable definition, even if its library file changes during the batch, with separate runtime state and existing unique telemetry filenames. Reproducing old results still requires the original `.fc` revision; hashes do not reconstruct files.

Legacy configurations continue without files being created merely by opening the simulation editor. **New…** is the explicit conversion route: it copies effective timing settings into an external design, retains link/output settings with the simulation, and selects the new reference.

## Hardware editor

The left pane contains the component library and board/component tree. The canvas supports:

- Adding/dragging components; editing board membership and properties.
- Nested boards, collapse/expand, and moving a board with its contents.
- Shift-click selection, alignment, pan, zoom, and Fit.
- Duplicating components/board subtrees with new IDs and remapped internal connections.
- Deleting components/subtrees and affected connections.
- Undo/redo and saved layout.
- Connecting supported components to named processor channels with bus, resource, and chip-select/address/pin information.
- Selecting connections for inspection/removal and selecting diagnostics to locate affected items.

Press **Enter** in a property field, or click **Apply properties**, to commit its form to the editor buffer. Design names can be edited at the tree root. Form headings are bold and explanatory text wraps. Notes and Java source retain normal newline entry. The tree uses OpenRocket’s rocket-tree expand/collapse style and individual vector component icons. Connection labels sit toward their connected components, about three-quarters of the way from the processor, and avoid node boxes. Canvas coordinates do not change rocket mass or physical mounting.

The design can contain multiple processors, or use a custom Java board without a separate processor symbol. **Connect to processor / Java board…** lists both kinds of host. Connections may distribute roles across hosts, and the same sensor can connect to several hosts. The built-in controller uses one shared set of inputs and flight state; separate processor symbols do not instantiate independent firmware. Each Java board runs its own program. Distinct sensors cannot silently compete for the same shared input; validation reports that ambiguity. Bus-address and pin conflicts are checked separately per host.

Airbrake, roll and recovery output components are optional. Commands to missing outputs are logged and discarded; missing airbrakes remain physically closed, and no physical AirbrakeSet is required in that case. Extra devices can be drawn; a second device cannot silently drive an occupied role. Disconnect the old role before connecting a replacement supported sensor. Validation also checks required roles, duplicate IDs, parent-board cycles, unsupported fields/models, and missing SPI selects/I²C addresses.

## Executable edits

| Edit | Runtime effect |
| --- | --- |
| Sensor minimum interval | Holds prior readings between eligible FC boundaries; zero means every loop for barometer/accelerometer/gyroscope |
| GPS period | Changes independently scheduled fix cadence |
| Sensor bias/noise | Changes measurements using per-component seeded streams |
| Accelerometer/gyro rotations | Rotates vectors relative to existing Zephyrus mounting, in X/Y/Z order |
| Barometer average length | Changes the altitude filter window |
| Transition condition tree | Changes actual decisions with all/any groups and explicit comparisons |
| Telemetry/power elapsed thresholds | Changes strict elapsed-time send conditions |
| Apogee lockout and secondary recovery delays | Changes existing state/timer checks |
| Maximum requested airbrake fraction | Limits the active controller request before PWM conversion |
| Sensor-read cost, extra work, work jitter, PWM phase, seeds | Uses existing virtual-time scheduling, including overruns |

Bias/noise units are pressure hPa, acceleration m/s², gyro deg/s, and GPS altitude meters. Scalar vector bias/noise applies to each axis. GPS latitude/longitude remains unchanged by the absolute altitude bias/noise. New percentage noise applies to every raw measurement, including latitude/longitude, and to temperature in kelvin. Percent noise is σ as a percentage of the absolute reading; zero-valued channels need absolute noise if noise at zero is desired. Set **Measurement Gaussian noise σ (% of reading)** under Behavior & timing for the whole design, or an additional percentage on each sensor. Independent global, per-sensor percentage and absolute noise variances add; the seed makes runs reproducible. Pressure and temperature are bounded positive, latitude is bounded and longitude wraps. Percentage noise on geographic coordinates can represent a large position error; use small values. These settings are stored in `.fc`.

## PID tuning

Open **Flight computer → Edit… → Behavior & timing**, then scroll the lower settings pane to **Airbrake PID** and **Roll PID**. Edit Kp, Ki and Kd, press Enter or **Apply timing / controller settings**, then Save. All six coefficients live in the external `.fc` parameters; changing them invalidates earlier simulation results through the design fingerprint. Older files without these keys use the original defaults.

| Controller | `.fc` parameter keys | Default Kp / Ki / Kd |
| --- | --- | --- |
| Airbrakes | `airbrakeKp`, `airbrakeKi`, `airbrakeKd` | 1 / 2 / 0 |
| Roll | `rollKp`, `rollKi`, `rollKd` | 0.08444 / 0 / 0.02111 |

Airbrakes retain the source controller's gain scheduling, feed-forward deployment and nonstandard per-update integral recurrence: `I += (2/K) × error` when its existing saturation rule permits, and requested area is `Astar + (Kp/K) × error + (Ki/K) × I + (Kd/K) × d(error)/dt`. Here K is the existing altitude/area sensitivity and error is predicted apogee minus target altitude. The derivative uses actual elapsed virtual seconds, including when the firmware's flight-time input remains an integer number of seconds. The first derivative sample is zero. Zeroing all gains retains the initial deployment schedule; remove the airbrake output or use a zero deployment limit to suppress deployment.

Roll uses `Kp × error + Ki × integral(error dt) + Kd × (-measured roll rate)` before its existing aerodynamic gain scaling and servo compensation. Roll error is negative measured roll in degrees, the integral uses degree-seconds, and the derivative input is degrees per second. The new integral holds when it would drive farther into the existing pre-compensation ±10° limit. PID histories reset at setup and launch. Both controllers log configured gains and their integral/derivative state. Default settings still match all four original C++ replay fixtures. Roll output is plotted/logged; its physical aerodynamic coupling remains unimplemented.

Acquisition occurs at accepted FC boundaries. The minimum interval is not an independent conversion pipeline; aggregate sensor-read duration remains the conversion/read timing model. Bus/resource edits describe topology and conflicts, not bus arbitration or electrical transfer delays. Power and storage actions are recorded. Board/power geometry is descriptive.

## Behavior and timing

Select a state in the strip to edit its name, flight phase and ordered entry actions. **Insert state after selected…** adds an intermediate state, initially reached after a configurable time in the previous state, with an optional recovery-channel action. Existing outgoing transitions move behind it. For example, insert “Drogue descent” after Apogee and give it a recovery action. The phase describes sensor/telemetry behavior; it does not re-run the original phase’s entry actions.

Transitions have explicit endpoints, priorities and conditions. Select one to edit nested ALL/ANY groups and comparisons (`>`, `>=`, `<`, `<=`, `==`, `!=`). Add/delete states and transitions to change the graph. One eligible transition is taken per completed FC loop, in increasing priority order. Entry actions run once per entry; `state_ms` resets on entry. Other signals include virtual boot, flight and apogee time, acceleration, altitude/descent from peak, estimated velocity, gyro angles, GPS fix and command ID.

Actions include preparation, launch, apogee, main recovery, finishing, individual recovery channels 0–5, closing controllers, explicit airbrake fraction, roll angle and log messages. **Automatic delayed recovery outputs** retains the source’s secondary/tertiary recovery timers; disable it when custom states take responsibility for those channels. Both options produce logical recovery outputs: OpenRocket still controls physical parachute deployment.

Graphs are saved inside the external `.fc`. Validation rejects missing endpoints, duplicate priorities/IDs, invalid actions/conditions, unreachable states and an invalid initial state. Legacy files without a state graph preserve the original behavior until edited. Wire telemetry retains its original phase codes; plot data, 3D labels and logs additionally identify the custom state by name and code.

Timing costs are modeled assumptions unless measured. The 3 ms default represents source barometer waits; a passing simulation does not qualify hardware execution timing. FC recovery and roll actions are recorded: OpenRocket still controls physical recovery, and physical roll coupling remains disabled.

## Test and traces

- **Pad test (3 s)** runs the current editor buffer through the same timing dispatcher as flight integration. It labels scratch/unsaved use and writes telemetry/log files.
- **Replay raw sensor CSV…** holds supplied engineering-unit observations between timestamps and uses the same dispatcher. It does not decode GS telemetry estimates.
- The trace plot selects barometric altitude, integrated velocity, applied airbrake fraction, state ID, or sample age against time. Logs include transitions and timing events; summaries report loops, overruns, execution duration, and sample/output ages.
- **Run saved design on this rocket** runs one test flight on a simulation copy with fresh output paths. It preserves existing simulation results and shows its complete log/telemetry paths. Use the normal simulation workflow for persistent plotted flight results.
- **Export trace…** saves the displayed log. Displayed text is bounded; complete logs remain at the reported paths.

Replay header:

```text
time_us,ax,ay,az,pressure_hpa,temperature_c,gx,gy,gz,latitude,longitude,gps_altitude_m
```

First timestamp: zero. Later timestamps: strictly increasing. Maximum: 100,000 rows through 600 seconds. Acceleration uses Zephyrus sensor axes and gyro values use deg/s. Already-filtered altitude must not be supplied as pressure.

## Shared edits and saving

Saving commits an external file independently of the simulation dialog; cancelling that dialog does not undo an already-saved `.fc`. The designer provides undo/redo and Save/Discard/Cancel on close.

Save checks for external changes since opening. A conflict requires Reload or Save as. Export produces a distributable file; Save as creates/selects an independent library design. Existing results retain their original fingerprint.

## Verification

Core and Swing regression checks pass. The PID/parallel-board update passed **42 core tests and 5 Swing tests**; the earlier plotting update also passed its **12 Swing tests**. Checks cover import collision/deduplication, template round-trip, semantic hashes, invalid graphs, real effects of sensor/rule edits, default-flight equivalence, missing/changed files, immutable ensemble batches, reference-only `.ork` persistence, archived provenance, and existing FC/timing/telemetry/ensemble behavior. New coverage verifies all six PID coefficients, all four unmodified C++ replay fixtures, virtual-time integration/differentiation, independent board periods and overruns, frozen inputs and completion-time outputs, a two-board full flight, multiple processors and Java-board hosting, external working copies, intermediate recovery states, Java compilation/approval, optional airbrake outputs, noise statistics, plotted sensor units, log import, exact saved measurement precision, and 3D state seeking.

All three editor views were rendered for visual inspection. A separate smoke check uses the packaged JAR to compile a Java board, load the installed template, render the designer and dedicated simulation tab, and open imported plotting options. The user's earlier `zephyrus-2679935281387596436/OR.log` imports as one flight with **84,808 samples**. Imported results also survive `.ork` save/reopen without motors or an FC definition; their observations are retained even when normal simulated data is omitted. The root `shadowJar` build succeeds and the installed macOS app's JAR target matches the new build. Windows paths follow the existing platform helper; a Windows interactive sweep and Windows installer are outside this build task.

## Custom Java boards

Add **Custom Java board** from the component library. It is a board container that can hold sensors/components. Select it and choose **Edit Java code…**. Source, period and assumed execution cost are stored in `.fc`.

Implement `public class BoardProgram extends JavaBoardProgram` with `public void step(Context io)`. The supplied example and **API help** document the methods. Inputs are the measured FC signals, not hidden simulation truth. Use `io.timeUs()`, `io.dtUs()`, `io.stateId()`, `io.altitudeM()`, `io.velocityMps()`, `io.accelerationMps2()` or `io.signal(name)`. Outputs are `io.setAirbrakes(fraction)`, `io.setRollDegrees(angle)`, `io.fireRecovery(channel)` and `io.log(text)`. `io.connected(port)` reports whether an output is present.

**Compile** reports Java diagnostics without initializing/executing the class. **Compile & enable** applies the source and records local approval of its exact hash. Loading a file does not execute code; changed/imported code needs local enablement before a run. Source runs inside OpenRocket with the application's privileges, not in a sandbox, and callbacks must return promptly. The embedded [Janino compiler](https://janino-compiler.github.io/janino/) works without a separate JDK; use ordinary Java classes and explicit types rather than unsupported `var`, records or lambdas.

Every simulation creates new program instances and class loaders. **All programmable boards run in tandem on independent virtual schedules**, alongside the main FC. Each board starts at boot time zero and follows its own period. Its execution cost delays its own output; it does not block other boards or add to the main FC's work. An overrun starts that board's next iteration immediately after completion, without overlapping invocations. Logs record each start, completion, execution cost and overrun. The main timing summary remains the main FC's timing.

Boards read immutable snapshots of the same shared measured signals and flight state at their invocation start. `io.timeUs()` is that start time; `io.dtUs()` is the interval between this board's starts. Outputs become available at completion and are latched by the independent PWM timer. At equal timestamps, PWM runs first, the main FC is serviced next, then existing board work completes before new board starts; simultaneous completions within each batch use stable board-ID order. All boards starting together capture their inputs before any zero-cost program publishes output. Multiple writes to a shared output are resolved by the last completed write. Java overrides a selected output until another controller/state/manual command changes it. Host wall-clock compile/execution speed does not masquerade as hardware timing.

**Custom Java boards are for algorithm experiments. Communication between boards is not accurately simulated: boards share measured inputs directly, without bus delays, packet loss or contention. These results are not authoritative predictions of hardware behavior.** This notice is shown in both the Java editor and Test & traces view. The telemetry link's loss/delay settings still apply to the radio output; they do not introduce communication between custom boards.

### Editing Java externally

In **Edit Java code…**, choose **Choose external editor…** to select an application/executable, or **Use system editor** for the OS Java-file association. **Edit externally…** exports the current source to a unique `BoardProgram.java` under the FC library's `JavaEditing/` directory and opens it. Use an editor or IDE's own highlighting and completion features. No new dependency or external editor is required for the existing in-app editor.

Save changes in the external editor, click **Reload external changes** in OpenRocket, then **Compile & enable** and save the `.fc`. Reloading does not execute or automatically enable imported code. Conflicting unsaved edits are preserved unless explicitly replaced. The `.fc` remains the saved flight-computer artifact; the Java file is an editing copy and is retained after closing so external work is not discarded. Editor choice is a local preference and is not stored in `.fc`.

## Keyboard shortcuts

| Shortcut | Action |
| --- | --- |
| Ctrl/⌘ S | Apply the active form and save `.fc` |
| Ctrl/⌘ Shift S | Save an independent copy |
| Ctrl/⌘ Z; Ctrl Y / ⌘ Shift Z | Undo; redo |
| Ctrl/⌘ D | Duplicate a selected component (outside text editing) |
| Delete / Backspace | Delete selected hardware when tree/canvas has focus |
| F2 | Focus/select the name field |
| Ctrl/⌘ Shift F | Fit the canvas |
| Enter / Ctrl/⌘ Enter | Apply property form |

The **Shortcuts** button lists these. Java source has its own text undo/redo, Ctrl/⌘ S saves it with the design and Ctrl/⌘ Enter applies it to the editor buffer.

## FC plots and log import

After running, **Plot data → 2D plots → preset configurations** contains FC servo PWM outputs, angles, airbrake deployment, state, measured/physical height and velocity, and paired raw accelerometer XYZ, gyro XYZ, pressure, temperature and GPS latitude/longitude plots. Presets requiring absent recorded columns are hidden; available columns remain selectable individually and exportable. Values are persisted in `.ork` and per-run ensemble archives with correct units.

Measured curves are the actual held sensor readings/estimates, including sample cadence, noise and filtering. Physical acceleration includes gravity/specific force and is rotated into the same sensor mounting axes; gyro rates use the same mounting. Estimated sensor-X velocity is compared with physical vertical velocity, so tilt/integration differences are real model differences. Zeroed barometric/GPS height is compared with launch-relative physical altitude.

In **3D trajectory**, a label follows the rocket and shows the FC state, including custom state names. State transitions are held, not interpolated, when scrubbing/rewinding. Ensemble mean trajectories show **MIXED STATES** when runs disagree rather than averaging state numbers.

**Flight computer → Import FC log as result…** reads an OR action log in the background, adds separate imported simulations to the document and opens their plotting view. It preserves existing simulations and never executes content from the log. New logs include `fc.plot_metadata`, `fc.plot` records (nominal 10 ms spacing) and recorded flight events for trajectory playback. Old logs recover the recorded sensors, servo outputs, state and vertical motion only; no horizontal position or attitude is invented. Their approximate launch-relative altitude uses the first physical sample as the reference. The importer explains these limits. Imported results can be saved in `.ork` and plotted without the original `.fc` file.

## Remaining model limits

Iris/Balius firmware execution, arbitrary sensor models, general signal-flow wiring, independent firmware/sensor contexts for each processor, independent conversion queues, bus contention/electrical models and the separate JNI listener remain future work. Physical roll and FC-driven physical recovery coupling are still absent. Editable state graphs and programmable Java boards are implemented on the existing controller's scheduler and sensor/control boundary.
