# Plan: external flight-computer files and graphical designer

**Status:** implementation authorized on 7 October 2026; the first Zephyrus designer is implemented. See [the implementation and usage report](06-flight-computer-designer-implementation.md) for delivered features, verification, and remaining extensions.

**Date:** 7 October 2026.
**Application:** OpenRocket MIT; no release-version change is proposed here.

## 1. Intended result and agreed decisions

Users will select a flight computer, load its standalone `.fc` file, and edit its hardware architecture and behavior graphically inside OpenRocket. The first supplied design will be Zephyrus. Iris and Balius will become additional choices when their definitions and simulation implementations are available.

The following decisions are established by the discussion:

1. Add a computer dropdown to the existing flight-computer simulation panel.
2. Store the flight-computer definition in an external `.fc` file, analogous to a motor definition in an `.eng` file. Do **not** embed that definition in `.ork` files.
3. Provide a graphical hardware canvas for a main board, additional boards, sensors, processors, connectors, and their connections.
4. Provide a linked behavior editor for sensor processing, flight states, controllers, and outputs.
5. Make boards expandable containers, with a component library on the left and properties for the selected item on the right.
6. Supply a protected Zephyrus template and allow users to create independent editable copies.
7. Extend the existing Swing application, extension configuration, Java FC integration, and virtual-time scheduling. Keep the later JNI implementation behind its separate simulation listener, as previously agreed.

The file encoding, internal model types, and development stages below are proposed implementation decisions. They make the agreed product concrete without claiming that these features already exist.

## 2. Starting point

Source root: `../../clone/openrocket-release-24.12.RC.01/` relative to this document.

The current implementation already provides an enable checkbox, telemetry-link settings, output-file selectors, virtual execution-time settings, and a translated Zephyrus FC. The listener constructs `RTFC` directly; a computer dropdown alone would not provide interchangeable computer models.

Source inspected for this plan on 7 October includes the power-command timer correction and staged FC execution. `FlightComputerTimingSettings` supports sensor-read duration, extra work, work jitter, PWM phase, and a random seed. Its default sensor-read duration is 3,000 microseconds, reflecting the two source barometer waits. The listener has separate CPU/PWM/GPS scheduling and overrun reporting. These are simulation assumptions and source-derived durations, not measured execution times for a particular board. This plan does not repeat the earlier assessment that those mechanisms are entirely absent; it also does not independently requalify them.

The current runtime still identifies pyro outputs as recorded only and roll coupling to the physical simulation as disabled. The designer must accurately expose each output's actual capability.

Relevant starting files:

| Existing location | Planned use |
| --- | --- |
| `swing/src/main/java/info/openrocket/swing/gui/simulation/FlightComputerPanel.java` | Computer selection, external-file controls, launch designer, show file status |
| `swing/src/main/java/info/openrocket/swing/gui/simulation/SimulationOptionsPanel.java` | Preserve the left-column placement below ensemble controls |
| `core/src/main/java/info/openrocket/core/simulation/extension/impl/ZephyrusFlightComputer.java` | Evolve persisted configuration into a computer/file reference while preserving legacy readers |
| `core/src/main/java/info/openrocket/core/simulation/listeners/FlightControllerSimulatorListener.java` | Resolve a prepared Java model into the existing sample, scheduling, output, and logging lifecycle |
| `core/src/main/java/edu/mit/rocket_team/zephyrus/FC/RTFC.java` | Retain the translated Zephyrus implementation and expose supported configuration points |
| `core/src/main/java/edu/mit/rocket_team/zephyrus/FC/FlightComputerTimingSettings.java` | Reuse current timing semantics and distinguish source-derived, estimated, and measured values |
| `core/src/main/java/edu/mit/rocket_team/zephyrus/util/RTSimulationCommunicator.java` | Keep the boundary between logical FC outputs and rocket actuation |
| `core/src/main/java/info/openrocket/core/document/Simulation.java` | Track selected-file changes, result provenance, and stale results |
| `core/src/main/java/info/openrocket/core/simulation/ensemble/EnsembleRunner.java` | Freeze one resolved design for all N members of a batch |

## 3. Simulation-panel workflow

Retain the current panel location: below ensemble simulation controls in the left column of the simulation editor. Keep the initial panel compact; the full designer opens separately.

```text
Flight computer simulation
  [x] Enable

  Computer     [ Zephyrus                         v ]
  Design file  [ flight-computers/zephyrus.fc        ] [ Browse... ]
               [ Edit... ] [ New... ] [ Save as... ]
  Status: Ready — Zephyrus baseline

  > Telemetry link and output files
  > Rocket connections
```

- The computer dropdown initially shows Zephyrus, Iris, and Balius. Mark unavailable models clearly and disable running them; never substitute Zephyrus for an unavailable model.
- Loading a valid `.fc` file selects its declared computer automatically. A computer selection must not relabel an incompatible file: choosing another computer requires a matching file or template.
- **New** starts from a supported template. **Edit** opens the referenced file. Protected templates open for inspection; editing creates a copy through **Save as**.
- **Save as** writes an independent design with a new design ID and a recorded origin. It can then be selected for this simulation.
- Show actionable states such as ready, missing file, changed since last run, incompatible model, or invalid design.
- Turning FC simulation off retains the file reference and other settings.
- Keep received-telemetry loss/delay and output destinations available. Those are simulation/link settings, distinct from the onboard telemetry definition in the `.fc` file.

## 4. External `.fc` file contract

### 4.1 Ownership and contents

Use a standalone, human-readable, versioned format. **Proposed encoding: UTF-8 JSON**, with an explicit schema identifier/version and stable IDs for graph objects. Use deterministic formatting so Git diffs remain useful. The design should be self-contained for the first version; installed component-model IDs are dependencies, but additional external design fragments are deferred.

| Data | Stored in `.fc` |
| --- | --- |
| Identity | Design ID, name, computer family, description, origin/template information |
| Compatibility | File schema version; required Java model, component-library IDs, and their versions |
| Hardware | Boards, board hierarchy, components, named ports, buses, power connections, and parameters |
| Sensors | Orientation, sample/conversion rates, units and frames, range, filtering, bias/noise/delay models |
| Behavior | Typed processing blocks, connections, flight states, ordered transitions, actions, controller parameters |
| Outputs | Logical channels, signal limits, refresh behavior, and modeled actuator characteristics |
| Timing | Task periods/order, acquisition and execution costs, interrupt phase, assumptions and provenance |
| Onboard telemetry | Supported packet format, fields, generation cadence, and onboard logging configuration |
| Presentation | Node positions, board bounds, collapsed groups, labels, and saved editor views |

Values must carry unambiguous units and coordinate-frame conventions. Missing information must remain explicitly unknown; a blank field must not silently mean zero delay, zero noise, or unlimited capacity. Store configurable noise/timing seed defaults explicitly and record any run-derived seeds in results.

The file declares supported models and data. It does not load arbitrary Java class names, execute embedded scripts, or contain a compiled firmware image. Unknown model IDs must produce a clear compatibility error before a run.

### 4.2 What remains in `.ork`

The `.ork` file stores only the association and simulation-specific information:

- FC enabled state, selected computer, and a reference to the `.fc` path/design ID.
- Bindings from logical FC outputs to this rocket's component IDs, plus installation-specific placement if modeled.
- Receiver/link settings, run seed choices, and output CSV/log destinations.
- Ordinary simulation results and provenance identifying which external design produced them.

Do **not** store the hardware graph, behavior graph, graphical layout, or a hidden complete FC snapshot in `.ork`. Existing full histories for all N ensemble members remain simulation results and continue to be saved normally.

### 4.3 Paths, updates, and reproducibility

1. **Updated requirement:** copy each externally loaded `.fc` into a standard user library under OpenRocket’s application-data directory (`FlightComputers`), preserving the source. Deduplicate identical imports and preserve distinct files with colliding names.
2. Store library-relative references in `.ork`. Export/share the `.fc` with a project and import/select it on the receiving machine. Missing references require explicit location/import; do not silently substitute another file. This supersedes the original project-relative default-path proposal.
3. Existing results remain viewable when the `.fc` is missing or changed. An FC-enabled run requires a compatible, validated design.
4. Record the exact-file SHA-256, a canonical simulation-definition fingerprint, model/library versions, and run seeds. The semantic fingerprint excludes purely graphical layout and descriptive labels; moving nodes alone should not invalidate physical results.
5. Detect external changes when opening the editor and before running. A changed functional definition or model version marks associated results outdated, without deleting them or automatically rerunning them.
6. Read, validate, and freeze one immutable design at run start. All N ensemble members use that same design and model version, with separate runtime state and recorded member seeds. Mid-run file edits apply only to later runs.
7. Reproducing an old result requires its original `.fc` revision and compatible runtime models. A hash identifies that revision but cannot reconstruct it. Keep both `.ork` and `.fc` files in the project/version-control workflow.

Write `.fc` files atomically. Reject malformed or unsupported newer schemas with useful diagnostics, preserve the original file, and offer explicit migration for supported older versions. Avoid silently dropping unknown model data during editing.

## 5. Graphical hardware designer

### 5.1 Workspace

```text
Library              Hardware / Behavior / Test                Properties
-----------------    -------------------------------------    ------------------
Boards               [ Main board                       ]     Selected: Barometer
Processors           [   MCU ---- SPI bus ---- Barometer]     Model
Sensors              [    |                             ]     Sample rate
Communication        [    +------ IMU                   ]     Conversion time
Power                [ External ports                   ]     Noise / bias
Output drivers       [__________________________________]     Orientation
Connectors                    | UART        | power           Bus settings
                              GPS board     Power board

                     Diagnostics: select an item to highlight it
```

The diagram is illustrative, not a verified Zephyrus schematic. Build the supplied Zephyrus template from inspected firmware and available board documentation; do not infer physical wiring from the example above.

Required interactions:

- Drag components from the library; add, rename, duplicate, and delete boards or components.
- Expand a board to see its contents; collapse it to expose named external ports.
- Connect compatible ports, disconnect them, and inspect connection properties.
- Represent shared buses explicitly, including multiple devices, addresses, SPI chip-select lines, and external bus connectors.
- Pan, zoom, fit the design, select multiple items, align items, and use undo/redo.
- Provide a searchable component tree and keyboard-accessible property forms so essential editing does not depend on dragging.
- Preserve stable IDs when moving or renaming items. Duplication creates new IDs and remaps copied internal connections.
- Clicking a diagnostic highlights the responsible component, port, connection, or rule.

### 5.2 Component and connection models

Start with a small supported library: main/expansion boards, processor/task host, barometer, accelerometer/gyro or IMU, GPS, radio, storage, power board, connectors, and output channels. Use the selected Zephyrus source to determine the actual first template. Add part-specific models only when their behavior is defined.

Every component declares typed ports, configurable parameters, and a capability classification:

| Classification | Meaning to the user |
| --- | --- |
| Simulated | The component affects measurements, timing, logic, or outputs through an implemented model |
| Recorded only | The action is logged but does not drive a corresponding physical effect |
| Descriptive | Architecture/documentation only; no simulated behavior is implied |

A descriptive component can be saved in a draft. If a required control path depends on behavior it cannot provide, validation must prevent running that design. Display capabilities next to the affected component, not only in a general disclaimer.

Check connection direction, signal/bus type, resource ownership, pin conflicts, duplicate addresses, required SPI selects, and voltage compatibility where ratings are specified. Report missing ratings as unknown. Power wiring may initially support connectivity and declared-rating checks; it must not imply a voltage-drop, battery, or brownout model unless those effects are implemented.

Canvas positions are drawing coordinates. Moving a board on the canvas does not move its physical mounting point, change sensor orientation, or alter rocket mass. Physical mounting is an explicit property; any future automatic mass contribution must integrate with rocket components without double-counting existing avionics mass.

## 6. Linked behavior editor

The behavior view consumes the hardware model's named sensor channels and drives its named output channels. Selecting an item in either view highlights its counterpart and shows where data is used.

Support three coordinated views of the same executable definition:

1. **Signal flow:** sensor channels → filters/estimators → controllers → logical outputs.
2. **Flight states:** states, entry/exit actions, and transition arrows with editable conditions.
3. **Task/timing properties:** when blocks run, what data they consume, and how they interact with output refresh.

Start with typed, bounded blocks implemented in Java: existing controller blocks, constants, comparisons, all/any conditions, timers, filters, latches, and channel mappings. Property forms are the initial editing mechanism; arbitrary expressions and general-purpose code editing are deferred.

The first behavior diagram can inspect the reference Zephyrus implementation. Editable rules should be enabled only when the runtime executes them. Clearly distinguish a reference firmware model from a customized experimental design; never accept a graphical edit that the simulation silently ignores.

For editable state logic, define and test:

- Initial state, state entry/exit actions, timer origins, and reset behavior.
- Explicit `>`, `>=`, `<`, and `<=` comparisons with units.
- Whether grouped conditions mean all or any, and a visible priority for competing transitions.
- A deterministic rule for transition count per iteration and action ordering.
- Sample freshness and behavior when a sensor is unavailable or stale.
- Output ownership, command limits, holds/defaults, and disabled-controller behavior.
- No unbounded same-tick transition loops or algebraic cycles without an explicit state/delay block.

The existing translated Zephyrus remains the initial runtime reference. Before declaring a graphical Zephyrus representation equivalent, compare its state, command, telemetry, and timing traces against the source-derived reference cases, including exact logical boundaries. Do not simplify ordering or integer-time behavior merely to make the diagram cleaner.

## 7. Simulation and timing integration

Use a narrow computer-model selection boundary around the current Java integration: resolve a design, create a fresh per-run FC instance, deliver sensor data, advance scheduled work, collect outputs, and close logs. Reuse existing clock, engine deadline handling, sensor adapters, communicator, and telemetry infrastructure. Keep Zephyrus-specific behavior in its existing classes where appropriate; generic file/graph definitions should live outside its package.

The `.fc` model must connect the displayed hardware and behavior to actual runtime semantics. Do not expose editable fields solely because a graphical widget is easy to add. A parameter is runnable only when the selected implementation supports it.

Timing should follow the modeled path:

```text
Physical sample → conversion ready → bus delivery → FC task → output register → PWM/output update
```

- Reuse current virtual-time costs, jitter, overrun behavior, and independent PWM scheduling first.
- Preserve current `millis()` and strict-comparison semantics for the reference FC. An assumed 10 ms target is not a guarantee of an exact 100 Hz hardware loop.
- Show each duration's provenance: firmware-derived, assumed/estimated, or measured, with optional measurement notes.
- When adding per-device or per-bus delays, define whether they replace or contribute to existing aggregate costs. Never count the same barometer wait twice.
- Make bus occupancy, queueing, and transfer-completion behavior explicit before claiming contention is simulated.
- Preserve causal delivery; do not expose future samples or execute FC logic inside trial physics evaluations.
- Use reproducible random streams keyed to stable model IDs. Merely changing canvas layout or traversal order must not change noise realizations.
- Preserve one FC installation per simulation, per-run isolation, and unique telemetry output paths for ensemble members.
- Continue detailed `System.out.println` action logging. Include design ID, fingerprints, model versions, resolved file path, timing assumptions, and output capabilities at startup; retain explicit simulated timestamps and final file paths.

The designer should expose the current recorded-only recovery and disabled physical roll coupling honestly. Connecting an output in the canvas does not itself implement new recovery or roll physics. Any later change in who controls physical recovery requires its own explicit integration and validation.

The eventual JNI backend uses a separate listener and may consume the same design identity/hardware metadata. It must declare which edits it supports. Native firmware must not silently ignore customized behavior graphs that only the Java model can execute.

## 8. Editing, sharing, and compatibility

- Keep editor changes in an undoable buffer until explicitly saved. Cancel discards unsaved edits.
- `.fc` saving and `.ork` dialog cancellation are separate operations. Once a user saves a shared external file, cancelling a simulation dialog does not roll back that file. Make this distinction clear in the editor workflow.
- Detect external edits before overwriting a file; offer reload or Save as to avoid losing another editor's changes.
- A shared `.fc` change affects all simulations referencing it. Open simulations should update their status; unopened projects detect the change when loaded or run.
- Preserve the existing managed Zephyrus extension and exact legacy Java-listener compatibility during migration. Opening an older `.ork` must not create files or silently rewrite its configuration.
- Allow old configurations to run through an explicit legacy compatibility path. Conversion writes a standalone `.fc` using the legacy effective settings, then replaces those embedded design settings with a reference when the user saves the converted project.
- Treat the existing historical embedding as a legacy reader concern, not as a reason to introduce a new embedded format. New designer-based configurations always use external definitions.
- Preserve unrelated simulation extensions and leave the separate `AirbrakesControllerListener` untouched.
- Ship the Zephyrus baseline as a protected, real `.fc` resource available to the file workflow. User copies remain separate from application upgrades. Record which template/model version produced a design.

## 9. Test workspace

Provide a **Test** view in the designer with:

- A short pad/bench scenario using controlled inputs.
- Sensor-recording replay with explicit channel mapping, units, timestamps, and missing-data rules.
- A full OpenRocket flight using the currently selected rocket and simulation options.
- Synchronized plots of sensor values and ages, flight state, controller requests, applied outputs, and timing events.
- Event explanations that identify the transition/condition responsible for an action.
- Timing summaries for loop durations, overruns, task/output phase, and sensor-to-output age.

Replay must distinguish raw measurements from already filtered estimates to avoid filtering twice or feeding unavailable truth into the FC. Comparisons with GS1 should retain the established high confidence in barometric altitude and low confidence in GPS altitude, while treating those packets as insufficient to measure all hardware execution timing.

Any full-flight result used outside an unsaved editor session must identify a saved `.fc` revision. Scratch tests may run an in-memory editor buffer if clearly labeled unsaved; they must not be presented as reproducible saved-flight results.

## 10. Implementation stages and completion criteria

Each stage extends the preceding one. Do not activate Iris or Balius simply to populate the menu.

### Stage 1 — Define supported models and the file boundary

- [ ] Inventory current Zephyrus components, settings, timing assumptions, and output capabilities against firmware and board evidence.
- [ ] Define stable computer/component/model IDs and a schema for the external design and its layout.
- [ ] Define the small Java model-selection boundary around the existing listener, without replacing the simulation engine.
- [ ] Build a standalone Zephyrus template and versioned reader/writer/validator.
- [ ] Implement deterministic semantic fingerprinting, exact-file hashing, and atomic saves.

**Done when:** the template round-trips without losing IDs or semantics, invalid/unsupported definitions are diagnosed, and loading does not execute or change the rocket.

### Stage 2 — Add selection, references, and migration

- [ ] Add the computer dropdown and Browse/New/Edit/Save as controls to the existing panel.
- [ ] Store file references and simulation-specific bindings through the existing extension mechanism.
- [ ] Resolve relative paths, missing files, external changes, and Save as rebasing.
- [ ] Record provenance and preserve old results while marking changed definitions stale.
- [ ] Add legacy read/conversion behavior and freeze the selected design for an entire ensemble batch.

**Done when:** a saved rocket can be reopened or moved with its `.fc`, run through the current Zephyrus integration, and saved without embedding the design. Missing files do not prevent viewing existing results. Compare equivalent settings with the pre-designer runtime, not an obsolete timing baseline.

### Stage 3 — Deliver the hardware canvas

- [ ] Implement the palette, board containers, ports/buses, selection, pan/zoom, and properties panel in Swing.
- [ ] Add undo/redo, duplication, tree/keyboard editing, and saved layout.
- [ ] Implement connection/resource diagnostics and visible capability classifications.
- [ ] Expose only hardware parameters supported by the runtime; allow clearly marked incomplete drafts.

**Done when:** a user can open Zephyrus, create a copy, add a board and supported sensor, connect it, edit its properties, save, and reopen the same graph. A connected supported sensor supplies its declared channels; an unsupported required path blocks a run with a useful explanation.

### Stage 4 — Link and edit behavior

- [ ] Add the reference state/flow inspection view and cross-highlighting with hardware.
- [ ] Implement supported processing/rule blocks using the existing Java controller and scheduling infrastructure.
- [ ] Add explicit transition/action ordering, data freshness, output ownership, and units validation.
- [ ] Enable graph edits only when their effects are executed by the selected model.
- [ ] Compare a representable baseline with source-derived boundary/replay cases before claiming equivalence.

**Done when:** changing an editable condition or controller parameter produces the predicted trace change, saved edits survive reopening, and conflicting/cyclic invalid logic cannot run silently.

### Stage 5 — Add test/replay and timing inspection

- [ ] Add bench inputs, recorded-input replay, and full-flight execution from the editor.
- [ ] Display synchronized state/sensor/output traces and causal event explanations.
- [ ] Expose existing virtual-time settings with provenance and model limitations.
- [ ] Add supported per-device/per-bus timing incrementally with no duplicate costs.

**Done when:** a user can explain a state transition or delayed output from the trace and reproduce a seeded test independently of canvas layout. Hardware timing claims remain tied to measurements, not merely to a passing simulation.

### Stage 6 — Verify and package the complete first version

- [ ] Complete the verification matrix below and update user documentation with actual supported models.
- [ ] Build the root `shadowJar` using the existing `OpenRocket-MIT-v<version>.jar` naming convention.
- [ ] Perform packaged macOS and Windows GUI checks when preparing their builds.
- [ ] Record implemented capabilities, remaining descriptive-only components, and known limitations.

**Done when:** the Zephyrus external-file/designer workflow works end to end with existing plotting, telemetry, and ensemble results. A release/version bump is a separate decision.

### Later additions

- Iris and Balius templates and runtimes, after receiving their firmware, board/connection definitions, and relevant validation data.
- The separately planned JNI listener and its supported-edit contract.
- Additional component libraries and measured timing models.
- Circuit schematics, PCB layout, firmware generation/flashing, and detailed analog/power simulation are outside the initial designer scope.

## 11. Verification matrix

| Area | Required evidence |
| --- | --- |
| File contract | Round-trip functional and layout data; stable IDs; malformed files; unknown schemas/models; interrupted writes leave prior file intact |
| External references | Relative/absolute paths, moved project, missing/replaced file, Save as, unsaved rocket, and exact/semantic hash changes |
| Persistence boundary | Inspect saved `.ork` and confirm reference/provenance/results only, with no hidden hardware/behavior/layout snapshot |
| Legacy compatibility | Open/cancel without mutation; equivalent explicit conversion; unrelated listeners preserved; duplicate FC rejection |
| Graph editing | Board nesting, ports, shared buses, delete/duplicate, undo/redo, keyboard editing, and highlighted diagnostics |
| Execution | Supported edits affect outputs; sensor channels follow bindings; conflicting writers and unsupported required paths rejected |
| Reference behavior | Source-derived thresholds and ordering, controller replay, power cadence, telemetry, and current timing semantics retained for equivalent settings |
| Timing | Causal samples, overruns, independent PWM, reproducible jitter, no double-counted delays, stable scheduling across appropriate physics-step changes |
| Reproducibility | Layout-only changes preserve semantic hash and seeded output; functional/model changes invalidate freshness; original revision required for replay |
| Ensemble isolation | All N runs share one immutable definition, use isolated state/seeds, survive external edits during execution, and retain provenance on save/reopen |
| User workflow | Protected templates, editable copies, external-save versus dialog-cancel behavior, shared-file changes, legible sizing, and packaged platform checks |

Use focused tests for semantic, persistence, scheduling, and integration contracts. Visual interaction/layout checks should exercise actual editor workflows. The implementation report records completed checks and distinguishes delivered capabilities from the remaining planned extensions.

## 12. Follow-on information needed

These inputs can be collected during implementation without blocking the initial schema and Zephyrus workflow:

- Authoritative board/component inventories and pin/bus mappings for each supplied hardware template.
- Firmware revisions and reference traces for Iris and Balius before enabling those computer choices.
- Measured sensor, bus, computation, and output timing to replace or supplement estimates.
- A prioritized list of additional real sensor/processor parts once the initial supported library is usable.

Related material: [existing FC controls](01-flight-computer-controls.md), [ensemble simulation](ensemble-simulation.md), [Java translation plan](../FCsim/java-translation-implementation-plan.md), and [historical hardware-contract assessment](../FCsim/flight-computer-simulation-performance-report.md). Treat the historical assessment as dated evidence; the current source observation in section 2 supersedes its claim that the power timer and execution-time model are still uncorrected/absent.
