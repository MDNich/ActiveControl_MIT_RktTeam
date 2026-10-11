# Motor batch simulation: implementation and verification

Implemented 11 October 2026 in the existing OpenRocket MIT source tree. The existing source version is **6.4**; this work does not change it. Builds retain the requested `OpenRocket-MIT-v6.4.jar` naming. The [design](07-motor-batch-simulation.md) has been updated for the subsequently requested parallel execution, component clearance checks, and motor certification levels.

## Using it

1. Select exactly one editable simulation in **Flight simulations**, then click **Batch simulate motors…**.
2. Choose the active motor mount, nominal motor diameter, available length, manufacturers, motor filters, and desired flight limits. The settings start with **4 worker threads**, minimum rail-exit speed of 15 m/s, and minimum ascent stability of 1 caliber. Blank limits are disabled.
3. Optionally enable **Ignore component intersections** under **Rocket and workers** to simulate overlapping motors. It is off by default. Diameter and available-length limits still apply; ignored overlaps remain visible in the results and CSV. Choose **Keep baseline delay** or **Choose best listed delay**. The latter tests every listed finite delay and selects a passing result with the lowest motor-ejection deployment speed, then closest deployment to apogee.
4. Run the batch. Results appear while it runs; cancellation retains completed and partial results. Select a row for reasons, warnings, classification notes, and FC output paths.
5. Export CSV for all candidates, passing candidates, selected rows, or every delay trial. Saving the rocket also saves batch settings, inputs, curves, and summaries. The saved-batch menu restores the results view; the controls still describe a new run using the current baseline.
6. **Create simulations from selected rows** opens the recorded rocket snapshot in a separate document and runs those motor/delay combinations as ordinary simulations for the existing plots. Their configurations remain independent. Saved summaries do not retain every searched trajectory.

## Motor level classification

The filter, table, and CSV include the individual motor's category:

| Motor impulse class | Displayed level |
| --- | --- |
| H, I | L1 |
| J, K, L | L2 |
| M, N, O | L3 |
| Smaller classes without an identified high-power exception | Below L1* |
| Missing class or outside the usual H–O range | Needs review |

The manufacturer designation takes precedence over integrating a particular measured curve. If a usable designation is absent, the curve's integrated impulse provides the fallback; the CSV records that basis. Fractional A designations are preserved. For sub-H motors, available catalogue metadata promotes average thrust above 80 N, propellant mass above 125 g, or hybrid motors to L1. Sparking propellant and uncertain catalogue data require review because this database has no reliable structured sparking flag. The asterisk and row note make this visible. These are individual motor categories; whole-rocket/cluster requirements and current motor approval are separate.

Sources checked 11 October 2026: [NAR Level 1 procedures](https://www.nar.org/Level1HPRCertificationProcedures), [NAR high-power certification](https://nar.org/content.aspx?club_id=114127&module_id=668315&page_id=22), [NAR motor classification](https://www.nar.org/content.aspx?club_id=114127&module_id=669253&page_id=22), and [Tripoli certification levels](https://www.tripoli.org/content.aspx?club_id=795696&module_id=468541&page_id=22).

## Installed motor clearance

Every candidate is placed at `mount length + overhang - motor length`. Its cylindrical envelope is checked against active component instances, including repeated mounts and fixed motors. This catches forward interference even when the user increases the maximum-length field.

Checks cover hollow tube/ring bores, solid bulkheads, transition shoulders and caps, packed mass/recovery envelopes, through-wall fin tabs, and collisions between clustered motors. Surface contact is allowed with a 1 μm numerical tolerance. Component names and IDs are included in rejection reasons. Rail buttons use their outward-facing geometry rather than the larger selection bounds used by the drawing UI.

Known intersections produce `MOTOR_INTERSECTS` and a nonpassing assessment. Unusual components checked with bounding envelopes, including canted fins, produce `CLEARANCE_REVIEW_ENVELOPE`; those rows cannot be declared a fit. Packed-component envelopes reflect the dimensions in the rocket file, not necessarily a detailed physical arrangement. Retainers, unmodeled hardware, construction tolerances, and adapter engineering are outside the geometry check; select a modeled adapter mount when using one.

The batch-only **Ignore component intersections** option overrides `MOTOR_INTERSECTS` and `CLEARANCE_REVIEW_ENVELOPE` exclusions. It retains dimension, maximum-length, delay, and flight-criteria checks. Matching flights can meet the selected criteria with an explicit **intersection override** annotation; this does not assert physical clearance. The table and row details list the ignored intersections, the console prints `MOTOR_BATCH intersection_override`, and CSV exports include `component_intersection_override` and `ignored_component_intersections`. Saved `.ork` batches retain the setting and findings; older saved batches default to strict screening. Execution rechecks the captured rocket using the selected override, so stale preview results cannot dictate screening.

## Execution and storage

The core implementation is in `core/.../simulation/motorbatch`; Swing uses `MotorBatchDialog` from the existing simulation tab. Preferred thrust-curve selection is shared with the ordinary motor picker. No second simulator or motor database was introduced.

The coordinator captures the inputs and dispatches private document/rocket copies to a bounded executor. Each candidate tests its delays sequentially; separate candidates run concurrently. Options, wind generators, listeners, and FC runtime state belong to each trial. The actual wind generator is reseeded as well as the option field. The existing FC extension supports parallel execution; other extensions require one thread because their shared state has not been established. Java FC behavior retains the existing per-run class-loader handling; this is not a sandbox for arbitrary user Java programs.

Each FC trial gets a unique batch/candidate/trial suffix for both CSV and log paths, including custom destinations. `MOTOR_BATCH` console lines identify settings, exclusions, starts, outcomes, cancellation, shortlist creation, and CSV destinations. Closing the dialog cancels and waits for active trials to close their outputs. Incomplete delay searches cannot appear as optimized passing results.

Summary extraction uses launch-relative altitude and actual event times. Stability covers rail exit through the earlier of apogee or recovery. Speed, Mach, and acceleration maxima end at first recovery. Ground vertical descent speed is separate from total ground speed. Missing required events or enabled metrics cannot pass. Time-limited flights remain incomplete.

CSV includes failed and unrun rows, motor identity/dimensions, level/basis/notes, selected delay, all summary metrics, criteria, launch-condition XML, seeds, curve/input hashes, and FC paths. It is UTF-8, correctly quoted, locale-independent, and protects formula-like text fields while retaining signed numbers as numbers. Export uses a temporary file before replacement.

The versioned `.ork` batch record saves inputs and curves independently of the live motor catalogue. It includes an external FC reference and fingerprint, not an embedded `.fc` definition. Viewing/exporting saved summaries does not require that external file; rerunning requires its recorded revision. Input hashes, sizes, and key fields are validated. A damaged batch produces a load warning without preventing the rocket from opening. Saved snapshots use ordinary OR XML numeric precision, so reruns are compared with a tolerance rather than promised to be bit-for-bit identical.

## Verification

Automated checks cover one versus four workers, actual worker overlap, independent ordinary flights for three motors, cancellation during delay search, FC state/output isolation, motor level boundaries, missing optional catalogue metadata, forward bulkheads, hollow centering rings, fin tabs, radial mass offsets, exterior rail buttons, ascent-only metrics, report/CSV/`.ork` reloads, damaged-record handling, and recreating independent shortlisted simulations. Existing FC hardware-timing/output and simulation tests are also included.

The automated run passed **32 core tests and one native Swing test**. Independent ordinary flights agree within `1e-7` in the reported SI metrics; serial and four-worker batch summaries agree exactly. Saved XML reruns had apogee differences of A8: -1.51e-06 m, B4: -2.62e-07 m, C6: -8.73e-07 m, within the 1 mm reload tolerance.

The later intersection-override and startup correction passed **15 motor-batch core tests and 3 native Swing tests**, both in the working project and in an isolated copy of the staged source destined for GitHub. The override tests include a complete overlapping-motor flight, retained diameter/length/delay/flight limits, toggling back to strict screening, legacy record defaults, and JSON/`.ork`/CSV persistence. Swing tests cover the checkbox and main-panel startup with no selected simulation. The installed macOS launcher also completed startup using the rebuilt working-project JAR. The clean source build required building `:core:jar` before the root shadow task because the existing Gradle module transform resolves that artifact before it is produced on a fresh checkout.

The packaged JAR passed the dialog/run/CSV/reload/shortlist smoke test on **macOS and Windows 11**, both with Java 17.0.11. The three-motor desktop fixture reached three simultaneous flights with the four-thread default; the core fixture separately reached four. The desktop fixture deliberately includes nonpassing and time-limited results. Its seven compared metrics differed by at most 5.73e-11 across the two platforms, with identical execution statuses.

Packaged desktop smoke tests and numerical comparison results are recorded under [motor-batch-20261011](../../sim/FCsim/benchmarking/motor-batch-20261011/). The verification programs are [MotorBatchVerification.java](../../sim/FCsim/MotorBatchVerification.java) for the real model and [MotorBatchDesktopVerification.java](../../sim/FCsim/MotorBatchDesktopVerification.java) for the packaged desktop workflow.

### Zephyrus result

The unmodified `zephy_testlaunch-java-fc.ork` with `TestLaunch_v14.eng` produces **16 matching 152 mm catalogue candidates, all screened out by modeled clearance**, even with a deliberately generous 3 m length limit. The original N8406 also overlaps the modeled P-Motor, airbrakes module/casing, and some surrounding geometry. No candidate flight was run for this file, and this result is not a flight-performance comparison. The file was left unchanged.

See [candidate CSV](../../sim/FCsim/benchmarking/motor-batch-20261011/zephyrus-motor-candidates.csv), [saved batch JSON](../../sim/FCsim/benchmarking/motor-batch-20261011/zephyrus-motor-batch.json), and [clearance summary](../../sim/FCsim/benchmarking/motor-batch-20261011/summary.txt). These are strict-screening results. The intersection override now permits exploratory batch flights for overlapping candidates, with the recorded overlaps retained; increasing the length limit alone still does not suppress them.

## First-version boundaries

- One active stage, one varied mount; fixed motors and their ignition/recovery settings are retained.
- One preferred curve per catalogue motor and one nominal flight per motor/delay; ensemble analysis remains available for shortlisted flights.
- Best-delay search requires the varied mount to be the only motor-ejection source and requires motor-ejection recovery. Arbitrary drilled/adjustable delays are not inferred.
- Detailed trajectories are generated for selected reruns, while batch storage retains summaries and provenance.
- This build does not publish a GitHub release or replace platform installers.
