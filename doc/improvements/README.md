# OpenRocket MIT 6.2 improvements

Planned 20 September 2026 against the current source in `/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket-release-24.12.RC.01`.

Implemented in MIT edition 6.2. See the [implementation and verification report](03-implementation-report.md) for usage, output filenames, launcher compatibility, tests and limitations. The detailed plans remain below:

The latest [desktop build report](04-desktop-builds-6.2.md) covers the universal macOS DMG/ZIP and Windows x64 installer produced on 21 September 2026, with bundled Java, checksums and packaged launch/rendering checks.

1. [Flight computer controls and telemetry link settings](01-flight-computer-controls.md): a checkbox, three settings, and CSV/log file selectors below **Simulator options** in the left column, backed by the existing FC listener and OpenRocket extension system.
2. [Interactive 3D trajectory and rocket animation](02-trajectory-3d-and-animation.md): a 3D option in **Plot data**, equal spatial scaling, mouse rotation/panning/zooming, adjustable playback speed, a velocity arrow and a rocket cartoon whose attitude comes from the simulation.

## Chosen defaults

| Feature | Default |
| --- | --- |
| FC simulation | Off for a new simulation; recognize an already-installed FC listener as enabled |
| Downlink packet loss | 0% |
| Downlink delay | 0 ms |
| Repeatable random seed | 1 |
| Existing plot selection | 2D |
| 3D axes | Equal distance scale, launch-relative coordinates |
| Animation | Initially paused, playback factor 0.25× |
| Cartoon orientation | Full recorded simulation attitude, including roll |
| Recovery representation | Simple canopy when OpenRocket records recovery deployment |

Packet loss and delay describe the radio downlink received by the ground station. They do not change the FC's loop clock, onboard logging or control decisions. The additional controls are delay and random seed; the seed makes a loss pattern repeatable when comparing runs.

The playback factor scales the passage of simulation time: at 0.25×, one second of flight takes four seconds to display. Position and orientation share that clock. A separate display-size control keeps the cartoon visible across a kilometer-scale trajectory, and a velocity arrow communicates the actual simulated speed and direction.

## Implementation order

1. Implement and verify persistent FC settings, legacy-listener handling and editor behavior.
2. Add deterministic downlink loss/delay and verify the transmitted/received files.
3. Record full attitude and horizontal velocity components in normal simulation results, including `.ork` save/load.
4. Add the static interactive 3D viewer.
5. Add timeline playback, attitude animation, velocity display and recovery events.
6. Run the focused checks in both plans, build root `shadowJar`, and perform packaged GUI checks.

Each step has a concrete completion criterion in its detailed plan. This work extends the existing Swing/core application and FC classes. The previously identified FC hardware-timing corrections remain deferred as requested; these UI improvements must not be described as resolving that qualification work.

## Findings that shape the plans

- `SimulationOptionsPanel` hosts the FC box below **Simulator options** in the left column. The extension list stays on the right, per the final requested placement.
- Extension configuration already has `.ork` serialization and cloning support. The new box should use that storage, rather than add a second FC-settings mechanism.
- Simulation freshness currently checks rocket/options changes, not extension configuration. FC settings must explicitly invalidate prior results and participate in dialog cancellation and multi-simulation editing.
- Existing plot data stores position, scalar speeds and two angles describing the body axis. It does not store full roll attitude. Faithful animation requires saving the actual orientation quaternion.
- The application already includes JOGL and working 3D Swing components. The viewer can use those installed dependencies and rendering conventions.
- The landing and tumble steppers update translation but do not integrate attitude. Their saved orientation must be displayed honestly; the animation must not invent a hanging orientation or tumble motion.

The existing Swing/core framework is extended. The installed macOS launcher keeps its upstream-named JAR path, which now contains the same full application as `OpenRocket-MIT-v6.2.jar`.

## Subsequent planning

[External flight-computer files and graphical designer](05-flight-computer-designer.md), planned 7 October 2026: standalone `.fc` definitions, Zephyrus/Iris/Balius selection, graphical boards and sensors, linked behavior editing, and staged integration with the existing Java simulator. This document records the original plan; the implementation guide below describes delivered features and remaining limits. It supersedes embedding new flight-computer design definitions in `.ork`; project-specific references and simulation results remain in `.ork`.

[Flight-computer designer implementation and use](06-flight-computer-designer-implementation.md): managed `.fc` library, supplied Zephyrus file, graphical hardware/behavior editor, runtime integration, testing, and supported capabilities.

[Motor batch simulation and CSV export](07-motor-batch-simulation.md), planned and implemented 11 October 2026: manufacturer/motor filters, parallel flights (default four workers), installed-motor clearance checks, NAR/Tripoli levels, recovery-delay selection, flight criteria, cancellation, CSV export, and saved summaries. See [implementation and verification](08-motor-batch-implementation.md) for usage, tests, and the Zephyrus clearance findings.

[Idle memory investigation and fix](09-idle-memory-reclamation.md): live-object measurements, repeated FC runs, dialog lifetime checks, and automatic idle heap reclamation in the 6.4 startup path.

[Flight computer recovery coupling](10-flight-computer-recovery-coupling.md), planned 11 October 2026: exclusive FC recovery authority while FC simulation is enabled, channel-to-device mappings, timed physical deployment, plots/logs and implementation acceptance checks. OR deployment rules remain saved and resume when FC simulation is disabled.

[Flight computer recovery implementation and use](11-flight-computer-recovery-implementation.md): configuring channel assignments, physical deployment timing, saved/imported recovery plots, verification results and the rebuilt 6.4 application.

[Flight computer controller blocks](12-flight-computer-controller-blocks.md), planned 11 October 2026: graphical PID and velocity-weighted controllers, sensor/quantity selection, baseline capture, fin-tab/fin-cant/airbrake assignments, output ownership, virtual timing, plots and staged implementation checks.

[Controller blocks: implementation and verification](13-flight-computer-controller-implementation.md) — Signal flow designer, measured inputs, PID/velocity weighting, tab/cant/airbrake assignments and timing.

[Flight-computer API and architecture](14-flight-computer-api-and-architecture.md) — package rename to `edu.mit.rocket_team`, method documentation, [browsable Javadoc](javadoc/index.html), nine editable UML views and regeneration/verification instructions.

The architecture report is also available as a [17-page PDF](14-flight-computer-api-and-architecture.pdf) with nine native TikZ figures; the [standalone LaTeX source](14-flight-computer-api-and-architecture.tex) contains the complete report and every figure.

## Vanilla OpenRocket 24.12 export (13 October 2026)

Use **File → Export as → Vanilla OpenRocket 24.12 (.ork)…**. The conversion summary is shown before the file selector; the suggested filename ends in `-vanilla-24.12.ork`. The active MIT document, its saved state, source file and full simulation results are preserved. Exporting over that source file (including a symbolic or hard link) is rejected.

- Roll-control fin sets become ordinary trapezoidal fins, retaining their planform, thickness, mounting tabs, fillets, appearance and IDs. Movable tab deflection is removed.
- Triangular leading edges become **rounded** cross-sections. This keeps a plate-like fin and rounded leading edge without replacing the whole section by an airfoil. Calculated fin mass and aerodynamic behavior can differ; existing explicit mass/CG overrides remain.
- Fin cant outside vanilla's ±15° limit is clamped, with a conversion notice.
- Airbrakes become internal ballast with the same effective mass and axial CG. Their actuation, drag and detailed inertia are not represented.
- All simulation extensions are removed, including FC definitions/references, custom Java listeners and scripts. Saved standard recovery deployment rules remain and should be reviewed before rerunning.
- Simulations retain their configurations and ordinary launch/weather settings but export as unrun baselines. Flight data, external telemetry/log histories, FC provenance, ensemble settings/results, initial roll and motor-batch history are omitted. Ordinary motor configurations remain.
- Vanilla-compatible custom expressions and embedded textures remain. Expressions using MIT-only/extension variables, and expressions depending on those, are omitted.

The exporter streams a separate ZIP without cloning stored flight histories. It writes to a temporary file and replaces the destination only after success; cancellation and missing textures preserve an existing destination. Conversion details also appear on the console with the `VANILLA_EXPORT` prefix.

Validation used the [official OpenRocket 24.12 release JAR](https://github.com/openrocket/openrocket/releases/tag/release-24.12), SHA-256 `4959b72f52f5f607941e9722abbb7b7f0c4a38ebbbf84204a329db9f31c4f897`, downloaded separately from our MIT build. The official JVM (with no MIT classes on its classpath) loaded, saved and reopened the Zephyrus, demonstrator iteration 4, and mixed-feature example exports with zero warnings. The mixed-feature example completed a vanilla simulation through apogee and ground impact. The official desktop application also opened the demonstrator export. Seven export regressions plus eight existing saver tests passed; the two simulation-panel startup tests passed.

To reproduce after building `shadowJar` (run from the repository root with a Java 17 `JAVA_HOME`):

```sh
python3 packaging/verification/verify-vanilla24-export.py /path/to/design.ork
```

The script verifies the official binary's checksum and leaves exports, roundtripped files, logs and `verification.json` in `clone/openrocket-release-24.12.RC.01/build/vanilla-24.12/`. It also checks that input file hashes are unchanged. The optional `VanillaExportGuiAgent` checks that the desktop app opens the exported model and that the MIT build contains the enabled export menu item.

## Fin conversion preservation and percentage uncertainty (13 October 2026)

Adding or removing roll-control tabs now retains fillet radius **and material**, axial offset, mounting-tab offset, angular positioning mode, and the rotation used to calculate the CG of a single fin. The shared fin-copy routine previously omitted these fields. Appearance, inside appearance, visibility, material, finish, geometry, component identity, comments, and active mass/CG/Cd overrides are also covered by conversion regressions. The mounting-tab and roll-control-tab offsets now have separate editor/save/load accessors; changing one no longer changes the other. Existing file tags remain compatible. Previously lost values cannot be recovered automatically from files saved with the old conversion.

The **Ensemble simulation** box includes four additional controls, all initially zero:

| Input | Effect of one run's multiplier |
| --- | --- |
| Total mass σ (%) | Scales the active vehicle's structure, motor and remaining propellant mass, plus all moments of inertia; preserves its CG and nominal burn timing. |
| Overall Cd σ (%) | Scales axial drag and the drag coefficient breakdown, including user overrides, deployed recovery devices, and tumbling descent. |
| Roll forcing σ (%) | Scales aerodynamic roll forcing from fin cant and control tabs. Zero nominal forcing remains zero. |
| Roll damping σ (%) | Scales the aerodynamic roll-damping moment. |

Each is an independent Gaussian **fractional standard deviation**, not a hard bound: a 10% input draws a multiplier `1 + 0.10 × N(0,1)`, once per flight. That same multiplier applies throughout the flight and to separated stages. Percent uncertainty combines with the existing time-varying additive thrust noise and atmospheric uncertainty. Nonpositive multipliers stop the batch with an error and preserve previous results; they are not clipped or redrawn. The coefficient stream is seeded separately so older motor/weather realizations remain reproducible when these controls are disabled.

The existing 2D mean ±1σ bands, 3D mean trajectory and flight-summary probability distributions include all enabled variations. The console prints the four sampled multipliers for every flight. Saving simulation data retains each member's complete results **and** its sampled multipliers inside the `.ork`; old files default to zero added uncertainty and unit multipliers.

Verification covers physical mass/inertia/CG consistency, higher drag reducing apogee, recovery drag scaling, separate roll forcing/damping effects, fixed-seed repeatability, full-result save/reopen, old-file defaults, invalid-input handling, and the rendered Swing controls. Fin conversion tests cover one- and four-fin sets at four axial reference modes and independent mounting/control-tab persistence.

114 distinct focused regressions passed across core and Swing checks. The rebuilt v6.4 JAR opened the demonstrator in the desktop application. Vanilla 24.12 again loaded, saved and reopened all three compatibility fixtures without warnings. Logs and the current JAR checksum are saved under `clone/openrocket-release-24.12.RC.01/build/ensemble-verification/`.


## Fin profiles, wind direction, parallel ensembles and controller tab setup (13 October 2026)

Rounded fins now show curved leading and trailing edges in the rocket preview. Airfoil fins show a rounded nose and tapered tail; the fin editor includes an enlarged section sketch. The 3D renderer uses the same section geometry and retains control-tab deflection about the mid-thickness hinge, including ±90°. Dimensions in the rocket preview retain the specified thickness. These drawings are illustrative section categories, not a named or measured airfoil; this change does not alter the aerodynamic or mass model.

The ensemble panel adds **Wind direction σ (°)** and **Threads** (default **4**, range 1–256). Direction uncertainty is an independent Gaussian bearing offset drawn once per flight and applied to the existing wind profile at every altitude. It rotates clockwise from north, preserves nominal wind speed, and leaves the launch rod direction unchanged. A direction perturbation alone does not create wind when the baseline is calm. Component-wise east/north wind uncertainty is added after rotation. Direction draws and settings are retained alongside each member's complete saved results; older files default to zero direction uncertainty and four threads.

Parallel members use independent rocket copies, listeners and FC telemetry/log files. Random sampling and aggregation stay in run order, so changing thread count preserves results for the same seed. Only up to the worker count of flight histories is live at once; completed histories are compressed into the archive. Cancellation interrupts and closes all workers before returning, and failed batches preserve earlier results. FC extensions support parallel operation; other simulation extensions currently require one thread because their runtime isolation is not guaranteed. Each run's console output includes its bearing offset and output paths.

In **Flight computer designer → Controllers → Setup → Actuation: Fin tabs**, the physical-target list previously included only fins that had already been converted to roll-control fins. Through-the-wall mounting tabs are a different feature. The panel now explains this and offers **Add roll-control tabs** for active ordinary trapezoidal fins. This explicitly edits the rocket with Undo, preserving the fin's existing properties, and automatically selects the new target in the first unassigned row. New tabs occupy 25% of the trailing-edge length, centered along that edge, with chord equal to 20% of the smaller positive fin chord and an initial editable lift-slope approximation of 2π/rad. Review these values in the fin editor for the actual hardware. Existing targets with zero area or zero lift slope are labelled as having zero authority. Enable the controller, apply it, and save the FC design and rocket to retain the setup.

Verification covers tab conversion and UUID assignment, preserved fillets and mounting-tab positions, physical tab/cant coupling, wind-bearing convention, lateral dispersion, saved settings and sampled offsets, concurrent worker overlap, exact serial/parallel agreement, cancellation cleanup, and independent completed FC output files. Drawing checks cover curved profiles, true thickness, tab cutouts and ±90° deflection in both 2D geometry and the 3D rendering commands. UI screenshots are generated under `swing/build/ensemble-verification/`, `swing/build/fin-profile-verification/` and `swing/build/fc-controller-verification/`.


## Roll-control tab preview visibility (14 October 2026)

The 2D preview now resolves depth between the fixed fin and its deflected control tab. Previously, drawing their outlines independently exposed hinge edges buried inside the fixed fin and drew the cutout rim through the tab. This was particularly noticeable on a fin above the fuselage. Hidden segments are now removed, and the visible intersection of the two thickness profiles supplies a continuous hinge seam. Side, top and rear projections use the same visibility calculation. Selection hit-testing and drawing bounds are preserved; the actual tab geometry, mid-thickness hinge and simulation physics are unchanged.

Verification: the hinge-edge regression first failed on the previous renderer and now passes. All 41 focused drawing checks pass, covering positive/negative deflections, eight viewing angles above and below the body, ±90° travel, the analytic hinge seam, true thickness, curved profiles, cutout selection and fillets. Before/after images are in `swing/build/fin-profile-verification/roll-tabs-before.png` and `roll-tabs.png`.

The subsequent orientation audit used the user's `20267.demonstrator.iteration4.ork` without saving it. Its `MainFins` has four fins, zero cant, and a +40° control-tab angle. An independent Rodrigues rotation agrees with the tab corners and their world/side/rear projections to within 2.4 × 10⁻¹⁶ m across 9,800 coordinate checks. These cover ±90°, ±40°, zero deflection, all four fin instances, different view rotations, swept trailing edges and ±12° cant. The hinge remains at mid-thickness and parallel to the trailing edge. No angle-sign or rotation-order defect was found; the visibility repair does not change those rotations.

For this model's vertical trailing edge, a side-view tab chord projects as `(c cos δ, c sin δ sin φ)`, where δ is tab deflection and φ is the fin azimuth including view rotation. A fin directly above/below the body therefore shows a shortened chord (24.906 mm instead of 32.512 mm at 40°), while the near/far fins show +40°/−40° slopes. Its out-of-plane trailing-edge displacement is 20.898 mm. The visible outline is offset from the tab's centre plane by its 9.398 mm thickness and the hinge intersection. `swing/build/fin-profile-verification/roll-tab-orientation.png` overlays the preview with the independent centre-plane calculation; `roll-tab-orientation.txt` records the measured values. To rerun the file-specific audit, set `OR_TAB_CHECK_ROCKET` to the source `.ork` path and run `:swing:test --tests '*RollTabOrientationTest'`; the test checks that the source bytes remain unchanged.

The 3D rendering test also checks all eight thick-tab corners against independent hinge rotation for five deflections and three cant settings. Final verification comprises 43 drawing/3D tests, two orientation audits and 12 core tab tests, all passing. The built v6.4 JAR passed a desktop startup check; the installed `/Applications/OpenRocket_MIT.app` launcher resolves to that build on its next launch. Verification logs and the JAR checksum are in `build/roll-tab-visibility-verification/`.
