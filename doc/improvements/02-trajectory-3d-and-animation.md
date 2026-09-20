# Plan: interactive 3D trajectory and rocket animation

Implemented in 6.2. See [the implementation report](03-implementation-report.md) for actual test coverage and renderer measurements. Checkboxes describe implementation unless explicitly labelled verification.

## User-facing result

Add **Plot type: 2D graphs / 3D trajectory** at the top of **Edit simulation → Plot data**. Preserve the current 2D default. Selecting 3D replaces the irrelevant X/Y quantity controls with a brief trajectory description and branch selection; the existing **Plot** button opens a resizable 3D viewer. It works with normal simulation data whether or not the FC checkbox is enabled.

The viewer includes:

- A trajectory, launch marker, ground grid, labeled East/North/Height axes, optional ground projection and event markers.
- Drag to rotate, Shift-drag or middle-drag to pan, wheel/trackpad to zoom, plus **Fit**, **Reset**, **Top**, **Side** and **Perspective** controls.
- **Play/Pause**, **Restart**, a draggable time slider, simulation-time readout and playback speed. Initially paused; default 0.25×, presets 0.1×/0.25×/0.5×/1×/2×/5×, with editable range 0.01×–20×.
- A cartoon rocket with a nose, fins and an asymmetric colored stripe so pitch, yaw and roll are visible. A **Rocket size** control changes only its display size.
- A toggleable velocity arrow and speed readout in the selected units. Arrow-size adjustment changes only the visual arrow length.
- A simple canopy and tether after recorded recovery deployment, while retaining the rocket body's recorded orientation.

View changes, seeking and playback do not rerun the simulation or change its results. A PNG snapshot action provides a shareable view; movie export is not required for this first implementation.

## 1. Store the data needed for truthful orientation

Existing `SimulationStatus.storeData()` stores position and two angles computed from `orientation.rotateZ()`. These identify the body-axis direction but lose roll. Existing horizontal speed is also a magnitude, not a complete horizontal velocity vector.

- [x] Add built-in dimensionless flight data types for quaternion `w, x, y, z`, and speed types for `velocity_x` and `velocity_y`. Reuse the existing vertical velocity, positions and time.
- [x] Record these from the same `SimulationStatus` and timestamp as the other flight data. Use the existing `storeData()` path, not an FC-specific listener or intermediate Runge–Kutta evaluation. Do not change step sizes or the physics to obtain animation frames.
- [x] Define the convention explicitly: quaternion maps simulation body coordinates to the simulation world frame; body +Z is the nose direction. Verify against `getRocketOrientationQuaternion().rotateZ()` and cardinal launch directions before rendering.
- [x] Preserve the full-resolution attitude track even if the visible trajectory line is simplified. Keep all events and branch boundaries.
- [x] Register type names/units so the existing branch serializer and importer round-trip the channels. Test actual saved `.ork` files, not just in-memory values. Existing saved files without these channels must continue to load.
- [x] Confirm recording across powered, coast, recovery, tumble and final/aborted steps. Preserve only actual available samples; do not append invented post-flight motion.

For older results, allow the static 3D trajectory and a moving position marker. If full attitude is absent, display **Full orientation unavailable; rerun to animate the rocket**. Do not synthesize roll from the trajectory tangent or mistake the saved axis azimuth for roll. Do not automatically rerun a simulation just to open a plot.

The landing/tumble `AbstractEulerStepper` does not update orientation. A new recording will consequently retain its last attitude during those phases. Show **Orientation held by the simulation in this phase** when appropriate, based on the recovery/tumble event history for this engine. The viewer must not invent a hanging body or animated tumble. Faithfulness here means faithfulness to the simulation, not a claim that its recovery attitude is physically realistic.

Completion: a deliberately rolling flight retains the same position and full attitude before and after `.ork` save/load; a legacy file degrades clearly to position-only playback.

## 2. Prepare an immutable trajectory for the viewer

Add a small `TrajectoryData` adapter in the Swing plotting package. Consume completed `FlightDataBranch` values and events without modifying them.

- [x] Store time, local position, full velocity, normalized quaternion and the relevant events. Use the launch coordinate origin for all branches; do not reset the origin at stage separation.
- [x] Confirm and use OR's local east/north/up mapping. Height is simulation altitude above launch; do not introduce GPS altitude into this view. Labels and the speed readout use OpenRocket's unit preferences consistently.
- [x] Validate monotonic time and finite values. Collapse duplicate timestamps with a documented last-valid-sample rule while retaining events; split invalid segments rather than drawing across them. Scrubbing an unavailable interval should show missing data, not an invented trajectory.
- [x] Keep each branch separate and allow selecting the animated branch. A stage-separated branch must not inherit another branch's later recovery event. Existing branch-copy rules and event source IDs need a fixture.
- [x] Interpolate position and velocity between adjacent valid samples using their recorded timestamps. Use shortest-arc normalized quaternion SLERP; negate the second quaternion when the dot product is negative, and use normalized linear interpolation near identical quaternions. Keep exact recorded attitudes at sample times.
- [x] Do not interpolate across invalid data, branch changes or discontinuities. If source sampling cannot resolve rapid rotation, report the limitation; do not infer complete extra rotations from identical endpoint attitudes. Where available, angular-rate data can detect an interval spanning a potentially ambiguous half-turn or more.
- [x] Build arrays and trajectory line buffers away from the Swing event thread. Retain full data for animation; simplify only the displayed polyline when needed, preserving endpoints and events.

Completion: adapter tests cover vertical, tilted and rolling trajectories; quaternion sign flips; irregular/duplicate times; invalid samples; branch separation and missing attitude.

## 3. Add the static 3D plot

Use the JOGL dependencies already present in `swing/build.gradle`, following existing `RocketFigure3d`/`PhotoPanel` canvas initialization, theme and disposal conventions. Add a dedicated trajectory panel; do not repurpose the design-window camera or require Python, JavaFX or a new native plotting stack.

- [x] Add `Trajectory3DDialog` and `Trajectory3DPanel` under the existing Swing plotting area. Route `SimulationPlotPanel.doPlot()` to the new dialog when 3D is selected; the 2D “no Y quantities selected” check must not block 3D.
- [x] Use a single spatial scale: one meter occupies the same scene distance on every axis. Choose the camera fit from the largest dimension of the trajectory bounds, with padding. Handle a vertical or stationary track with a minimum nonzero span to avoid division by zero and camera clipping.
- [x] Label each axis and units; provide readable ticks/grid spacing across meter and kilometer extents. Use a uniform scene transform, never independent automatic stretching of the three axes. Top/side views and zoom make small horizontal drift inspectable.
- [x] Keep camera orientation independent of rocket orientation. Support mouse drag/rotate, pan, zoom and reset while paused or playing. An optional follow-position mode may translate the camera target without rotating it with the rocket.
- [x] Draw liftoff, burnout, apogee, recovery and ground-contact markers from stored events. Event selection seeks the timeline to that exact time.
- [x] If OpenGL is disabled/unavailable, give a specific message while preserving ordinary 2D plotting. Respect existing 3D preferences and initialization failure handling.
- [x] Stop animation and release GL resources/listeners when the dialog closes. Reopening should not accumulate render threads or stale listeners.

Completion: rotate, pan, zoom and fit the benchmark flight; verify a synthetic diagonal has physically equal axis scaling and a vertical flight remains visible and usable.

## 4. Animate time, velocity and attitude together

Maintain one playback clock independent of the simulation engine:

```text
displayedSimulationTime = anchorSimulationTime
                        + playbackFactor * elapsedMonotonicWallTime
```

Reset anchors on pause, seek and speed changes. A Swing timer requests display updates at approximately 30–60 frames/s, but elapsed monotonic time determines playback progress. Dropped rendering frames must not slow or accelerate the flight clock. Clamp to the actual recorded interval and stop at its end.

- [x] Evaluate position, velocity and attitude at the same displayed simulation time. At 0.25×, all recorded motion takes four times as long to view; recorded speed labels retain their true values.
- [x] Build the cartoon along body +Z with a visible roll marker. Apply the recorded quaternion, then position it at the interpolated trajectory point. Preserve the angle between the body axis and velocity, including ascent angle of attack and sideways descent. Never force the nose to point along velocity.
- [x] Keep cartoon display size independent of physical length and trajectory scaling. Use a sensible screen-visible default; changing it must not move the trajectory point or change orientation.
- [x] Draw the velocity vector from the rocket position using the recorded world components. Define arrow length as `arrowScaleSeconds * |velocity|`, with a display slider and clear legend; zero velocity yields no directional arrow. Do not confuse the arrow scale with the playback factor or the true speed readout.
- [x] Resolve recovery state from events at or before the current time, so seeking backward removes a later canopy and forward seeking restores it. Use actual `RECOVERY_DEVICE_DEPLOYMENT`, not apogee, ejection charge or a logged FC pyro command alone.
- [x] Use one simple canopy/tether depiction even if several devices are deployed, with event text identifying the device. Place the illustrative canopy above the body and retain the recorded quaternion for the body. The canopy shape need not be physically exact.
- [x] Pause at ground contact/end of data. Do not fabricate touchdown for an incomplete or aborted result.

Completion: an asymmetric rocket rolls independently of its flight path; slow motion scales position and orientation together; velocity remains physically labeled; forward/backward seeking reproduces the same recovery state.

## Source map

Source root: `/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket-release-24.12.RC.01`.

| Existing location relative to source root | Change/check |
| --- | --- |
| `core/src/main/java/info/openrocket/core/simulation/FlightDataType.java` | Register quaternion and horizontal velocity channels |
| `core/src/main/java/info/openrocket/core/simulation/SimulationStatus.java` | Capture new channels alongside each existing data point |
| `core/src/main/java/info/openrocket/core/simulation/FlightDataBranch.java` | Verify new values/events survive cloning and stage branching |
| `core/src/main/java/info/openrocket/core/file/openrocket/OpenRocketSaver.java` | Verify generic serialization of the additional channels |
| `core/src/main/java/info/openrocket/core/file/openrocket/importt/FlightDataBranchHandler.java` | Verify new-channel lookup and old-file compatibility |
| `core/src/main/java/info/openrocket/core/simulation/AbstractEulerStepper.java` | Reference for held recovery/tumble attitude; no new recovery dynamics in this work |
| `swing/src/main/java/info/openrocket/swing/gui/simulation/SimulationPlotPanel.java` | Plot-mode selector and 3D dialog entry |
| `swing/src/main/java/info/openrocket/swing/gui/plot/` | New trajectory adapter, playback controller, panel, dialog and simple rocket rendering |
| `swing/src/main/java/info/openrocket/swing/gui/figure3d/RocketFigure3d.java` | Reuse established JOGL/theme/lifecycle patterns |
| `core/src/main/resources/l10n/messages.properties` | Plot controls, axis names, data types and availability messages |

## 5. Verification and delivery

- [x] Unit-check quaternion interpolation at identity, 90° roll, sign-equivalent endpoints and near-180° changes; verify body/world coordinate conventions with known attitudes.
- [ ] Complete the expanded manual round-trip matrix (legacy files, separated branches, 2D plotting and export). Automated checks cover new-channel serialization, legacy data adaptation, and separated-branch attitude/recovery isolation individually.
- [x] Check that recording/viewing new channels leaves simulated trajectory and FC timing unchanged. Do not rerun the entire qualification campaign for a viewer change.
- [x] Validate playback with an injected clock: speed factors, pause/resume, arbitrary seeking, end conditions and recovery-event boundaries.
- [ ] Finish the complete manual gesture/playback sweep on each target platform. The actual renderer, equal-scale scene, roll/body basis, velocity vector, canopy, and PNG output have automated/visual checks; see the report.
- [x] Measure a roughly 100,000-sample trajectory: aim for at least 30 frames/s during playback and camera interaction on the test machine, recording machine/sample count rather than claiming a universal guarantee. Decimate the line if necessary, never the attitude data used for playback.
- [ ] Complete Windows GUI smoke testing. The root `shadowJar`, macOS launcher, native renderer, and reopening path have focused checks. Check 3D initialization, reopening/disposal, and the normal 2D path when 3D is disabled.
- [x] Save example screenshots and a short usage note in this improvements folder with the implementation results.

This is an OpenRocket results viewer. Full orientation comes from simulation truth, not the FC's integrated gyro estimates. The new FC checkbox and link settings are useful alongside it but are not prerequisites for plotting a trajectory.
