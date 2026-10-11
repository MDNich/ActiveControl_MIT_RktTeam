# Motor batch simulation and CSV export

Planned 11 October 2026 for OpenRocket MIT. Status: implemented; see [implementation and verification](08-motor-batch-implementation.md). Updated to include the requested parallel workers, installed-motor clearance checks, and NAR/Tripoli motor levels.

## 1. Result to deliver

Select a rocket and a baseline simulation, choose manufacturers and motor filters, run the matching motors, and export a CSV identifying which combinations meet the selected flight criteria. Show the failed combinations and their reasons as well. Use the existing Java simulation engine, motor catalogue, flight computer integration, and plotting tools.

**Export format: CSV only.** No Excel workbook, XLSX exporter, or spreadsheet-library dependency.

The reference feature is MMRocket Sim's motor batch workflow. Its [user guide](https://github.com/mtnmanak/mmrocket-sim/blob/65e502bacc6953fa25f66a5d52b958097f616427/packages/app/user-guide.md), [dialog](https://github.com/mtnmanak/mmrocket-sim/blob/65e502bacc6953fa25f66a5d52b958097f616427/packages/app/src/components/BatchSimulate.tsx), and [batch service](https://github.com/mtnmanak/mmrocket-sim/blob/65e502bacc6953fa25f66a5d52b958097f616427/packages/app/src/services/batchSweep.ts) were inspected at commit `65e502bacc6953fa25f66a5d52b958097f616427`. They provide the example of filtering a motor catalogue, simulating candidates, and grading the results. The design below is for our Swing/core application.

### First implementation boundaries

- One selected baseline simulation and one motor mount varied per batch.
- One active flight stage. Homogeneous clusters and repeated instances of the selected mount use the same candidate motor and delay. Other mounts in that stage retain their baseline motors and ignition settings.
- One preferred thrust curve per physical motor entry. Expose the chosen curve and digest so the comparison is reproducible.
- One nominal flight per motor/delay trial. The existing Gaussian engine/atmosphere ensemble mode remains available for subsequent analysis of shortlisted motors.
- Parallel execution with user-selected thread count (default 4), progress, cancellation, and partial CSV export.
- Result summaries saved in the rocket document; detailed trajectories obtained through explicit reruns of selected rows.

Defer simultaneous sweeps of several mounts, mixed-motor cluster combinations, multistage delay optimization, all-curve sensitivity sweeps and nested Monte Carlo sweeps. Detect unsupported configurations before starting and explain the limitation in the dialog.

## 2. User workflow

Add **Batch simulate motors…** to the **Flight simulations** tab. Enable it when exactly one editable simulation is selected. This simulation supplies the launch setup, active configuration, recovery events, numerical options, initial conditions, and enabled extensions.

Open a resizable dialog with three areas:

| Area | Controls and behavior |
| --- | --- |
| Rocket and launch | Baseline simulation name, active configuration, target mount selector, number of motor instances, mount dimensions, launch conditions, FC enabled/disabled summary |
| Motor selection | Manufacturer checklist with All/None, text search, nominal diameter, available length, impulse-class range, motor type, and catalogue availability filter; expandable numeric ranges for total impulse, average thrust, and burn time |
| Flight criteria | Minimum rail-exit velocity and ascent stability, optional apogee range, maximum speed/Mach/acceleration, deployment speed and landing descent speed limits; delay policy |

Place a sortable candidate preview below the settings. Show the number matching the catalogue filters, the number excluded by fit checks, and the number of actual flight trials, including delay alternatives. Validate ranges immediately; zero candidates is a normal empty result with an explanation.

Proposed starting criteria are 15 m/s minimum rail-exit velocity and 1.0 caliber minimum ascent stability, both editable and switchable. Other numerical limits start disabled. These are screening settings, recorded with every batch; **Meets criteria** means the simulated result satisfies those settings under this launch configuration.

Buttons: **Run batch**, **Cancel**, **Export CSV…**, **Create simulations from selected rows**, and **Close**. After starting, show candidate and trial progress separately. Keep completed rows visible while running. Closing a running dialog cancels its worker and releases resources; keep the partial summary available in the document.

The result table defaults to all considered candidates and supports **Meets criteria**, **Does not meet criteria**, and **All** views. Sort numbers numerically. Suggested visible columns: manufacturer, motor, diameter, length, delay, status, apogee, rail-exit speed, maximum speed/Mach/acceleration, minimum ascent stability, deployment speed, and reasons. An expandable detail view holds curve identity, launch mass, warnings, and output paths.

## 3. Candidate selection and motor compatibility

Use `Application.getThrustCurveMotorSetDatabase()` and the same manufacturer/impulse conventions as the existing motor selector. Do not download a second motor database or introduce a separate physics implementation.

1. Snapshot the catalogue entries when the batch is prepared. Apply explicit user filters to motor sets. These filters define the search universe; excluded manufacturers need not produce thousands of CSV rejection rows.
2. Select one curve per remaining set using the current preferred-curve policy: user preference first, existing heuristic second, stable digest tie-break last. Extract that policy into a shared helper so the ordinary motor selector and batch selector agree. Freeze the chosen curves for the batch.
3. Keep distinct curves/motors with identical display names distinguishable by manufacturer, designation, dimensions, type, and digest. Never use the table label as a unique key.
4. Screen against the selected mount's actual bore and the chosen nominal diameter class. A smaller motor is not automatically a compatible fit: select the corresponding modeled adapter mount when using a smaller nominal class. The first implementation verifies geometric clearance, not motor retention or adapter engineering.
5. Default the length warning threshold to mount length plus permitted overhang. Longer motors still run, with `MOTOR_TOO_LONG` retained as an advisory finding. Independently place each motor cylinder using its actual length and the mount overhang, then check active component instances, hollow bores, bulkheads, packed masses/recovery devices, fin tabs, shoulders/caps, other motors, and cluster spacing. The batch-only intersection override can allow those overlaps while preserving their descriptions in results. Name intersecting components; conservatively bounded shapes receive a clearance-review reason.
6. Preserve every motor that passed the catalogue filters as a result row, including `DOES_NOT_FIT` or `NEEDS_REVIEW` when fit is rejected or cannot be determined. Only eligible rows proceed to simulation.
7. Treat the catalogue's availability flag as catalogue metadata, not a live stock or current certification check. Label the control **Hide motors marked unavailable**.

Install each candidate through `MotorConfiguration` and `MotorMount.setMotorConfig`, preserving overhang and ignition behavior. Count physical instances with the existing mount/assembly logic, including clusters and assembly copies. Recalculate loaded mass, CG, inertia, and motor consumption using the normal engine; never reuse the baseline motor's launch mass.

Preserve mass overrides exactly and report them in batch settings. Add a regression test establishing how the existing mass calculator combines dry-rocket overrides and replacement motors. Do not silently reinterpret a user-entered mass as either wet or dry to make results appear plausible.

## 4. Ejection delay and recovery

Motor delay can determine whether a candidate works, so it must be an explicit part of the batch rather than a fixed delay copied indiscriminately from the original motor.

Offer these policies:

| Policy | Execution |
| --- | --- |
| Keep baseline delay | Use the exact selected finite delay only if it is listed for the candidate, or preserve plugged/no-ejection behavior. An unknown or unsupported finite delay produces `NEEDS_REVIEW`; never silently round it. |
| Choose best listed delay | For a motor-ejection recovery configuration, simulate every distinct finite delay listed for the selected curve and select a result by the rule below. Show the expanded trial count before running. |

Default to **Keep baseline delay**. Offer the second policy when recovery depends on motor ejection. Preserve electronic, altitude, apogee, and FC-controlled deployment settings; changing a motor must not replace the rocket's recovery strategy.

Use the selected curve's `getStandardDelays()` as the conservative delay source. `ThrustCurveMotorSet.getDelays()` is a union across curves and the ordinary picker also offers a plugged choice; neither is sufficient evidence that an arbitrary finite delay is available for this candidate. Missing delay metadata is unknown, not a zero-second delay. Plugged motors cannot satisfy a recovery configuration requiring their ejection charge. User-verified adjustable/drilled delay ranges can be added later.

For **Choose best listed delay**, evaluate all trials against the same criteria. Among passing trials, select the lowest speed at the first deployment caused by the varied mount's ejection event; break ties by smaller absolute time separation from apogee, then shorter delay. If none passes, display the best completed deployment trial by the same ordering as a failed result, with the union of relevant failure reasons available in details. If no valid deployment trial exists, retain an explicit incomplete/error result. A cancelled search is not presented as having established the best delay.

Store all trial summaries, including rejected trials. The normal CSV has one row per motor and the selected delay. A second export choice, **All delay trials**, writes one row per motor/delay to CSV. The engine's optimum-delay estimate may be displayed as a diagnostic; it must not generate an unavailable delay or replace validation through an actual recovery simulation.

## 5. Running independent, repeatable simulations

Create a batch snapshot before leaving the event-dispatch thread: rocket geometry with component IDs preserved, selected configuration, options, cloned extensions, preferred motor curves, filters, criteria, and random seeds. Reuse the FC designer's existing immutable input-resolution mechanism for any linked `.fc` definition and its behavior dependencies, recording their hashes. Keep that resolved definition in memory for the running batch; preserve the established external-file boundary when saving. Validate required inputs once before starting.

Two existing copy methods need special care: `Simulation.copy()` shares the rocket, while `duplicateSimulation(newRocket)` currently retains the source document reference. Add an explicit copy-to-document operation that binds the simulation to a detached document and detached rocket while preserving the stepper/calculator classes and extension configuration. Use a fresh trial copy of that snapshot for every motor/delay. Tests must prove edits, listeners, FC state, cancellation, and exceptions cannot leak into the user's original simulation or into the next trial.

The batch runner then performs:

1. Check cancellation and prepare the candidate's private configuration.
2. Disable ensemble variation only on the private trial, with the dialog explicitly stating **Single flight per motor/delay; ensemble variation is not applied**. Preserve the original ensemble settings for later shortlist runs.
3. Use the same recorded baseline wind/engine random seed and FC seed for every trial. Reinitialize the actual wind model as well as the option field, following the existing ensemble seed handling. This pairs stochastic launch conditions across candidates without promising identical FC random draws when their event sequences differ.
4. Preserve enabled FC behavior, sensor/timing contracts, roll control, and other simulation extensions. Validate extension clone/reset behavior; stop with a useful explanation if an enabled extension cannot be isolated.
5. Call the ordinary `Simulation.simulate(...)` path, with a step-level cancellation listener.
6. Extract metrics, event completeness, warnings, and output paths; evaluate criteria and publish a typed result to the UI.
7. Release detailed flight samples after extracting the summary, then proceed to the next trial.

Use a bounded pool with a user-selected thread count, default 4. Allocate a separate document, rocket, options, listeners, wind generator, and FC runtime to each concurrent trial. Dispatch private worker baselines on the coordinating thread. Permit the current FC extension in parallel; require one thread for other extensions whose shared state has not been established. Verify one-worker/four-worker equivalence and independent FC output paths. Snapshot all UI inputs before running; do not read mutable simulation options from Swing while the simulation holds its locks.

Ordinary candidate simulation failures become rows and do not stop other candidates. Batch-wide invalid configuration or output-storage failure stops the batch with partial results preserved. Treat cancellation separately from simulation failure and do not swallow fatal JVM errors.

### FC logs and telemetry

Generalize the existing ensemble output tag into a run-output tag while keeping compatibility with the ensemble callers. Give every trial a unique tag containing batch ID, candidate ID, and delay-trial number. Apply it to both user-selected and automatic CSV/log destinations, so trials cannot overwrite one another or the baseline's files.

Keep the FC's existing detailed logging. Add `System.out.println` messages prefixed `MOTOR_BATCH` for batch settings, exclusions, trial start/end, selected delays, pass/fail reasons, cancellation, and all output filenames. Report the batch CSV destination after export. Tests must include two candidates using identical custom FC filenames and a cancelled trial with partially written telemetry.

## 6. What counts as a successful result

Keep execution status and criteria assessment separate. Example execution statuses: `SCREENED_OUT`, `COMPLETE`, `INCOMPLETE`, `ERROR`, `CANCELLED`, and `NOT_RUN`. Assessment values: `MEETS_CRITERIA`, `DOES_NOT_MEET_CRITERIA`, and `NEEDS_REVIEW`.

A qualified row needs a valid simulated ascent, rail clearance, apogee, configured recovery events, and a normal ground-contact ending. A time-limit or abort ending is incomplete. A result that never leaves the rail fails that criterion even if a partial trajectory exists. A missing/nonfinite required metric can never pass. Preserve stable reason codes alongside readable explanations, e.g. `NO_RAIL_EXIT`, `RAIL_SPEED_LOW`, `ASCENT_STABILITY_LOW`, `APOGEE_HIGH`, `DELAY_UNKNOWN`, and `TIME_LIMIT`.

| Metric | Definition for this feature |
| --- | --- |
| Apogee | Maximum altitude above the launch point; keep launch-site altitude separately. Verify the branch coordinate convention when extracting it. |
| Rail-exit velocity | Velocity at the engine's launch-rod-clear event, using event-time interpolation rather than a guessed sample offset. |
| Minimum ascent stability | Minimum stability in calibers from rail clearance through the earlier of first apogee or first recovery deployment. Empty interval is missing. Do not reuse the ensemble's current entire-flight minimum, which includes descent. |
| Maximum speed and Mach | Maxima before the first recovery deployment, with the interval stated in the column names/help. |
| Maximum acceleration | Maximum total translational acceleration before first recovery, using the existing engine channel; do not relabel it accelerometer specific force or opening shock. |
| Recovery deployment speed | Maximum speed at any configured recovery deployment, with per-device event values in details. Delay ranking uses the particular motor-triggered deployment described above. |
| Landing descent speed | Downward vertical speed at ground contact, exported separately from total ground-contact speed. |
| Average-thrust/launch-weight ratio | Sum of rated average thrust for motors ignited at launch, accounting for all instances, divided by candidate loaded mass times local gravity. Optional filter, disabled by default. This is not instantaneous launch thrust or rail-exit thrust. |

Also export time to apogee, flight duration, landing distance, and warnings. Warnings remain visible even for passing rows. Abort/no-liftoff/missing required recovery conditions prevent a pass; informational warnings alone do not automatically reject a motor. Use the active rocket's branch explicitly rather than taking an arbitrary branch index in future multistage support.

## 7. CSV, document storage, and plotting

### CSV contract

Use a file selector for folder and filename, with the normal overwrite check. Default filename: `<rocket>-motor-batch-<timestamp>.csv`. Default export: **All candidates**; offer **Meets criteria only**, **Selected rows**, and **All delay trials**. UI filtering must not silently change the chosen export scope.

Export typed results directly, including failed and unrun rows. The current simulation-table exporter skips rows with missing cells, so reusing it unchanged would lose precisely the failures this feature needs to explain.

Required column groups:

| Group | Columns |
| --- | --- |
| Identity | Schema version, batch ID, candidate ID, trial ID when applicable, manufacturer, designation, motor type, thrust-curve digest |
| Motor and mount | NAR/Tripoli motor level (L1/L2/L3, Below L1, or Needs review), impulse class, classification basis/note, diameter/length, total impulse, rated average/peak thrust, burn time, launch/burnout motor mass, target mount ID/name, instance count, selected delay or `plugged`, delay policy |
| Outcome | Execution status, assessment, reason codes/text, warnings, error message, trials attempted/planned |
| Flight | Loaded launch mass, apogee, time to apogee, rail-exit speed, ascent speed/Mach/acceleration maxima, minimum ascent stability, deployment speed, landing vertical/total speed, landing distance, flight duration, average-thrust/launch-weight ratio |
| Provenance | Baseline name/configuration ID, snapshot hash, application version, batch creation time, random seeds, launch altitude/rail settings/atmosphere/wind model and values, enabled thresholds, filter settings, FC definition hash and enabled state, FC CSV/log paths |

Use SI units with explicit header suffixes, e.g. `apogee_agl_m`, `rail_exit_speed_m_s`, `min_ascent_stability_cal`, and `motor_diameter_m`. Numeric values use decimal points independent of system locale. Blank numeric fields represent unavailable values; reasons explain why. Never substitute zero or infinity for missing flight results. Include the threshold-enabled flags so a blank limit cannot be mistaken for zero.

Write UTF-8, a header row, comma delimiters, and proper quoting of commas, quotes, and newlines. Serialize finite numbers without grouping. Protect formula-like text fields when opened in spreadsheet applications while keeping genuine signed numbers numeric. Use an atomic temporary-file replacement so a failed export does not truncate an existing file. Do not add non-tabular preamble lines; repeat the necessary batch metadata in each row.

### Saving and revisiting results

Store a versioned batch record in `.ork`: settings, detached rocket/options snapshot, the selected motor curves, FC file reference and input hashes, every trial summary, and chosen per-motor summaries. The rocket snapshot excludes other batch records and previous simulation results. Store shared input objects only once per batch, with rows referring to their IDs. Preserve incomplete statuses when saving a partial run. Give users an explicit way to remove old batches.

Extend the existing document saver/loader using a dedicated batch element rather than disguising candidate motors as ensemble members or ordinary simulation rows. A damaged or unsupported batch record must produce a loading warning without preventing the rocket itself from opening. Validate counts and values before allocating or accepting loaded records. Keep FC definitions external, as established by the [FC designer plan](05-flight-computer-designer.md); do not embed a hidden `.fc` copy in the batch record. Existing custom-code trust checks still apply to reruns.

Detailed time histories for every searched motor are outside the first implementation. **Create simulations from selected rows** adds only the selected motors/delays to ordinary simulations and runs them for the existing 2D/3D plots. Use the saved batch input snapshot: if the current rocket differs, open the saved snapshot in a separate document before creating those simulations. If the external FC file is missing or has changed, require the matching revision for a reproducible rerun; retain the ability to view/export the saved summaries. A deliberate rerun with an updated FC definition is a new comparison, labeled accordingly. Do not attach old summaries to changed geometry or changed FC inputs. Allow the normal ensemble mode to be enabled for further analysis.

## 8. Implementation map and order

All new classes below are proposed names. Keep candidate selection, grading, and execution independent of Swing.

| Existing source | Planned use/change |
| --- | --- |
| [SimulationPanel.java](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket-release-24.12.RC.01/swing/src/main/java/info/openrocket/swing/gui/main/SimulationPanel.java) | Add batch action, retain/reopen batch results, and create selected ordinary simulations. |
| [MotorFilterPanel.java](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket-release-24.12.RC.01/swing/src/main/java/info/openrocket/swing/gui/dialogs/motor/thrustcurve/MotorFilterPanel.java), [MotorRowFilter.java](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket-release-24.12.RC.01/swing/src/main/java/info/openrocket/swing/gui/dialogs/motor/thrustcurve/MotorRowFilter.java) | Reuse controls and extract domain predicates where useful; keep mount-fit screening separate from display-filter tolerances. |
| [ThrustCurveMotorSelectionPanel.java](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket-release-24.12.RC.01/swing/src/main/java/info/openrocket/swing/gui/dialogs/motor/thrustcurve/ThrustCurveMotorSelectionPanel.java) | Extract shared preferred-curve selection. |
| [Simulation.java](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket-release-24.12.RC.01/core/src/main/java/info/openrocket/core/document/Simulation.java) | Add explicit detached-document copy support and a general output tag. |
| [SimulationWorker.java](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket-release-24.12.RC.01/swing/src/main/java/info/openrocket/swing/gui/simulation/SimulationWorker.java) | Follow its background execution and cancellation conventions. |
| [FlightControllerSimulatorListener.java](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket-release-24.12.RC.01/core/src/main/java/info/openrocket/core/simulation/listeners/FlightControllerSimulatorListener.java) | Honor per-trial output tags and frozen FC inputs without changing hardware timing. |
| [SimulationTableCSVExport.java](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket-release-24.12.RC.01/swing/src/main/java/info/openrocket/swing/file/SimulationTableCSVExport.java) | Reuse file-dialog conventions; write a dedicated typed batch exporter that retains missing/failed rows. |
| [OpenRocketSaver.java](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket-release-24.12.RC.01/core/src/main/java/info/openrocket/core/file/openrocket/OpenRocketSaver.java), [OpenRocketContentHandler.java](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket-release-24.12.RC.01/core/src/main/java/info/openrocket/core/file/openrocket/importt/OpenRocketContentHandler.java) | Add versioned batch persistence alongside existing document data. |

Proposed core package: `info.openrocket.core.simulation.motorbatch`, containing `MotorBatchSettings`, `MotorBatchSnapshot`, `MotorCandidateSelector`, `MotorBatchRunner`, `MotorBatchMetrics`, `MotorBatchCriteria`, and immutable candidate/trial/batch result types. Proposed Swing additions: `MotorBatchDialog`, `MotorBatchWorker`, `MotorBatchTableModel`, and `MotorBatchCSVExport` in the corresponding existing GUI/file packages.

Implement in these reviewable steps:

1. **Input and result contracts.** Define immutable settings, identities, units, reason codes, delay policies, and schema version. Completion: validation covers invalid ranges, unavailable metadata, and unsupported configurations.
2. **Candidate filtering.** Share preferred-curve selection, build the catalogue snapshot, and implement fit screening. Completion: a small fixture catalogue produces the expected motors, identities, and exclusion reasons without running physics.
3. **Trial isolation.** Add the detached copy operation, seed initialization, frozen FC inputs, and unique output tags. Completion: replacing a motor changes only the trial and leaves the baseline and other trials unchanged.
4. **Runner and grading.** Execute parallel candidate flights with sequential delay trials within each candidate, support delay search, extract event-aware metrics, and classify incomplete/error/cancelled flights. Completion: batch and manually configured single-run results agree.
5. **Dialog and results.** Wire the action, filters, criteria, progress, cancellation, sortable table, and details. Completion: the UI stays responsive and a failed motor does not stop the remaining candidates.
6. **CSV export.** Implement the four export scopes, provenance, quoting, and atomic saving. Completion: an independent CSV parser reads all expected rows and numbers, including failed and partial results.
7. **Persistence and shortlist.** Save/load batch inputs and summaries; create ordinary simulations from selected rows. Completion: reopening the `.ork` preserves results and can recreate a selected flight even after the live catalogue changes.
8. **Integration verification and build.** Run the checks below, build the root `shadowJar`, and exercise the packaged dialog on macOS and Windows. Keep the existing version and `OpenRocket-MIT-v<version>.jar` naming unless a separate version/release change is requested.

## 9. Verification and completion criteria

Use deterministic small fixtures for edge cases, then the actual [Zephyrus model](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/examples/zephy_testlaunch-java-fc.ork) for integration.

- **Selection:** manufacturer combinations, nominal versus actual diameter, overhang/length boundaries, duplicate designations, multiple curves, unavailable flags, unknown dimensions, cluster/assembly motor counts, and unsupported staged rockets.
- **Physical equivalence:** include the original motor and at least two alternatives actually present in the loaded catalogue. Compare each batch result to an independently configured ordinary simulation using identical seeds, curve, delay, and FC settings. Establish tolerances from timestep/event resolution; record actual differences rather than only a pass flag.
- **Recovery:** unavailable/unknown delays, plugged/electronic recovery, multiple listed delays with different outcomes, no valid deployment, cancellation during a delay search, and ranking that selects an actual passing trial.
- **Metrics:** a descent stability dip must not reject otherwise acceptable ascent stability; missing rail exit, premature termination, missing recovery, nonfinite metrics, and incorrect landing-speed interpretation must not pass.
- **Isolation:** baseline motors, configurations, results, document modified state, and custom filenames remain unchanged by trial execution. FC state resets between candidates; seed repetition gives the same result within established engine reproducibility limits. Adding the completed batch record intentionally marks the document modified.
- **FC integration:** run a small Zephyrus batch with FC enabled; verify separate telemetry/log paths and no regression in its existing sensor-loop/PWM timing checks. Confirm cancellation closes output streams.
- **CSV and storage:** parse exports containing commas, quotes, newlines, Unicode, formula-like labels, numeric negatives, missing metrics, partial runs, and sorted/filtered views. Save/reopen results and verify curve/FC hashes, delays, statuses, criteria, and row counts. A malformed batch must not prevent loading the rocket.
- **GUI and scale:** inspect layout at macOS/Windows display scaling, cancel during a flight, and run a large fixture catalogue without accumulating every trajectory in memory. Record elapsed time and peak memory; show no arbitrary hidden candidate/export limit.

The feature is complete when a user can filter the installed motor catalogue for Zephyrus, run it with the selected launch/FC configuration, see a reason for every nonpassing candidate, export the complete comparison as CSV, reopen the saved summary, and turn a shortlist into normal simulations for plots and ensemble analysis.
