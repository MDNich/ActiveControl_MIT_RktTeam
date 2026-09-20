# OpenRocket improvement plans

Planned 20 September 2026 against the current source in `/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket-release-24.12.RC.01`.

The requested changes are split into two implementation plans:

1. [Flight computer controls and telemetry link settings](01-flight-computer-controls.md): a checkbox and three small settings directly below **Add extensions** in the simulation editor, backed by the existing FC listener and OpenRocket extension system.
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

- `SimulationOptionsPanel` already owns the **Add extensions** button and can host the requested box immediately below it.
- Extension configuration already has `.ork` serialization and cloning support. The new box should use that storage, rather than add a second FC-settings mechanism.
- Simulation freshness currently checks rocket/options changes, not extension configuration. FC settings must explicitly invalidate prior results and participate in dialog cancellation and multi-simulation editing.
- Existing plot data stores position, scalar speeds and two angles describing the body axis. It does not store full roll attitude. Faithful animation requires saving the actual orientation quaternion.
- The application already includes JOGL and working 3D Swing components. The viewer can use those installed dependencies and rendering conventions.
- The landing and tumble steppers update translation but do not integrate attitude. Their saved orientation must be displayed honestly; the animation must not invent a hanging orientation or tumble motion.

Only these planning documents were created for this request. No application code, FC behavior or simulation data was changed.
