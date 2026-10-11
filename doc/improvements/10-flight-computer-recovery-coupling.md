# Flight computer recovery: connection to OpenRocket physics

Planned 11 October 2026 against the MIT 6.4 source in `clone/openrocket-release-24.12.RC.01`. This is the original plan; see the [implementation and verification report](11-flight-computer-recovery-implementation.md) for the completed feature and usage.

## Required behavior

When flight computer simulation is enabled, **the FC exclusively controls recovery**. OpenRocket's automatic deployment rules and their deployment delays are ignored for every recovery device in that simulation. There is no automatic fallback to apogee, altitude, motor ejection, launch, or stage-separation deployment rules.

The saved OpenRocket configurations remain intact. Disabling FC simulation restores their normal behavior. This applies to old `.ork` files as well as newly configured simulations: an enabled FC with no recovery mappings produces no recovery deployment. Show that consequence explicitly in the Flight computer tab and at the start of the log.

This extends the existing FC listener, controller and OpenRocket event processing. It does not introduce another simulation listener or a replacement flight-dynamics framework.

## User interface and storage

Add a **Recovery** section to the existing **Flight computer** tab, with this explanation:

> Flight computer controls all recovery. OpenRocket deployment rules are ignored while FC simulation is enabled. Devices without a channel assignment remain stowed.

Show one row per physical parachute or streamer:

| Field | Behavior |
| --- | --- |
| Recovery device | Rocket component name, with stage/parent context to distinguish duplicate names |
| FC channels | Select any of channels 0–5, or leave unassigned |
| Deployment delay | Nonnegative delay in milliseconds from accepted FC firing to physical deployment; default 0 |
| Status | Assigned, unassigned, inactive in this flight configuration, or invalid reference |

Several channels may deploy the same device, representing redundant firing paths. One channel may deploy several devices. The first accepted firing schedules each device once; subsequent commands cannot restart its delay or deploy it twice. For example, a user can assign channels 0 and 1 to a drogue and channel 2 to a main. Do not infer those assignments from device names or from a particular FC model.

Store physical mappings and deployment delays in the simulation's existing FC extension configuration in `.ork`. Store stable component UUIDs, not names; a rename must preserve the connection. Resolve those UUIDs against the simulation's private rocket copy. Keep `.fc` files portable: they contain the logical recovery outputs, board programs and state-machine actions, not references to a particular rocket's parachutes.

Use a versioned optional recovery-settings value in the existing extension `Config`, with a typed `FlightComputerRecoverySettings` helper. Missing settings mean no assignments, **not permission to run OR recovery rules when FC is enabled**. Preserve mappings when toggling FC off. Edits participate in normal simulation invalidation, cloning, dialog Apply/Cancel, save/load, ensembles and motor batches.

Do not silently reconnect a deleted component by name. A configured UUID that no longer resolves produces an actionable validation error when starting an FC run. Inactive-stage assignments are shown and skipped for that flight configuration. Unassigned devices and absent logical recovery components are valid designs, with visible status and logged discarded commands. In particular, absence of recovery hardware must not recreate the earlier mandatory-actuator error.

All custom Java boards currently share the same six recovery channels. Label that clearly. Independent per-board pyro banks are outside this first implementation.

## What the current source already does

- `RTFC.fireChannels()` arms and fires the logical pyro controller. Custom Java `io.fireRecovery(channel)`, state-machine recovery actions, built-in flight phases and radio commands converge on `RTPyroController.firePyro()`.
- `RTPyroController` has a 250 ms output pulse and continuity/status monitoring, but no connection to a physical recovery device. Its `isFired()` value describes a live pulse, not permanent deployment.
- Java board outputs become available at their modeled execution-completion time. The FC listener already schedules CPU, board, GPS and servo deadlines on a virtual microsecond clock, with one second of pad warm-up before simulation time zero.
- `Simulation.simulate()` currently instantiates `ModifiedEventSimulationEngine`. Both it and `BasicEventSimulationEngine` process `RECOVERY_DEVICE_DEPLOYMENT` events, update the deployed-device set and switch to descent integration. `BasicLandingStepper` calculates drag from the deployed devices' areas and drag coefficients.
- The modified engine and descent stepper already contain FC deadline limits. The basic engine still has differences that must be covered rather than assuming the two paths behave identically.
- The 3D trajectory viewer already uses real deployment events to display the parachute. Log import currently retains event type and time but loses the recovery device's identity.

Source paths below are relative to `clone/openrocket-release-24.12.RC.01`.

## Runtime connection

Use this existing path:

```text
Board / state machine / built-in controller / received radio command
  -> accepted RTPyroController.firePyro(channel)
  -> per-run recovery bridge resolves channel -> recovery device(s)
  -> standard OpenRocket RECOVERY_DEVICE_DEPLOYMENT event
  -> existing deployed-device set, descent physics, event plots and 3D canopy
```

### 1. Capture accepted firing, not sampled pulse edges

Add a small immutable accepted-fire record at the successful branch of `firePyro()`, after its existing armed check. Include a sequence number, channel, exact `trace.bootUs()` and command origin where available. Preserve board ID through `RTFC`'s Java output callback; identify built-in, state-machine and radio origins separately when possible.

Do not detect firing by periodically polling `isFired()`: a rearmed command can occur while the previous pulse is still high. Do not change the firmware's arming, pulse-duration or continuity logic as part of this connection. A rejected unarmed command cannot schedule deployment; an absent recovery component still discards its commands.

Own the command queue/observer and a small `FlightComputerRecoveryBridge` in the existing listener's per-run state. Drain commands after each virtual-time dispatch, including startup. Bench-only execution has no physical bridge. Release pending events, callbacks and rocket references at completion/cancellation; no static run registry or retained second copy of the flight history.

### 2. Make deployment time explicit

Calculate:

```text
commandSimulationUs = commandBootUs - warmupUs
deploymentDueUs = commandSimulationUs + configuredDeviceDelayUs
```

Convert the displayed delay to integer microseconds once with documented nearest-microsecond rounding and range validation. Keep due-time comparisons on that clock; convert to seconds at the OpenRocket event boundary.

- A Java command acts at modeled board completion, not the earlier sensor-sampling time.
- The 250 ms pyro pulse is not an extra deployment delay.
- Recovery does not wait for a servo PWM latch. Changing PWM phase must not change its firing time.
- Zero-delay deployment is processed at the current accepted physical boundary, before the next positive physics step. A future deployment becomes an event deadline, including during parachute descent.
- Retain the FC's CPU/board deadlines separately from recovery-event deadlines. A deployment between board ticks must not create an extra board execution or cause the listener to report a skipped CPU deadline.
- Never evaluate a command at an intermediate/rejected Runge–Kutta stage or backdate deployment after stepping past its due time.
- Commands strictly before simulation time zero are logged as pad-warm-up commands and discarded physically; they are not replayed at launch. Commands at time zero are eligible. Commands after landing are logged and discarded.

Drain all same-time commands deterministically in the existing dispatch order. Schedule devices in stable UUID order; record the first accepted cause. At a ground-hit/deployment tie, treat ground impact as ending airborne deployment, regardless of priority-queue insertion order. Test these boundaries explicitly.

### 3. Override OR rules and reuse physical events

In both event engines, skip ordinary recovery-rule scheduling whenever the existing FC listener is active. This suppresses all automatic recovery triggers and their delays without changing the document's component configurations. Other events, including motor burnout/ejection, apogee and stage separation, retain their own meanings; do not suppress those events globally.

The bridge queues standard `RECOVERY_DEVICE_DEPLOYMENT` events whose source is the actual `RecoveryDevice` in the run's rocket copy. Track bridge ownership in per-run state. While FC is active, reject recovery-deployment events that were not authorized by that bridge, including automatic events already queued or injected by another extension. Avoid implementing FC firing by synthesizing a motor ejection event, which could activate unrelated devices.

Keep the existing physical deployment handling, high-speed/launch-rod warnings and deployment-under-thrust abort behavior. Recovery authority changes; the existing engine's validity checks remain. Add a small shared helper for authority checks, duplicate suppression and outcome recording so both engines use the same policy.

Only report physical deployment after the engine records it in the deployed-device set and flight events. A pre-event callback is not confirmation: another listener can veto it or the stage can be inactive. Record vetoed/skipped outcomes separately. Once a device has a pending or completed deployment, redundant firing cannot create another event. Do not retry a vetoed deployment automatically through later redundant pulses.

Check pre-apogee deployment carefully. The modified engine already removes the live FC listener from its auxiliary optimum-coast calculation; preserve that isolation and provide equivalent handling in the basic engine. An auxiliary trajectory must not advance the live FC clock, write into its telemetry or reuse its pending recovery events.

## Results visible to the user

Use the existing trace path, including its `System.out.println` output and `OR.log`, to record:

- Startup recovery authority, ignored OR rules, channel assignments, configured delays and unassigned devices.
- Accepted or rejected command, channel, origin, boot time and simulation time.
- Scheduled deployment with device UUID/name and due time.
- Actual recorded deployment time and scheduling lateness, or discard/veto reason.

Keep logical firing, continuity status and physical deployment distinct. The initial bridge assumes an accepted mapped firing operates its configured device after the stated delay; it does not establish electrical igniter success. Do not manufacture a continuity break or firmware `PYRO_SUCCESS` from a canopy event.

Add plot channels for each recovery channel's armed state and active pulse, and each mapped device's deployed state. Display these as steps. Log command and deployment events at their actual times so short pulses and transitions are not inferred from the existing nominal 10 ms plot sampling. For ensemble displays, identify an averaged deployment state as a fraction of runs, not a partially deployed canopy.

Extend plot metadata and log event records with device UUID/name and channel/origin. Update `.ork` result persistence and `FlightComputerLogReader` so imported logs retain device labels without requiring the original rocket. Keep old logs readable; old logical `pyro.fire` messages must never be interpreted as proof of physical deployment. Preserve the existing GS-compatible telemetry packet/CSV format; recovery plotting data belongs in the general simulation channels and structured action log.

Use actual accepted deployment events for the existing 3D canopy and show the device name/time in the event display. FC state labels continue to show the FC's state independently. The current descent stepper does not integrate hanging/swinging attitude; this feature does not invent that motion.

## Implementation sequence and completion checks

1. **Settings and interface.** Add `FlightComputerRecoverySettings`; wire it through `ZephyrusFlightComputer` and `FlightComputerPanel`. Implement the table, validation and exclusive-control explanation. Check save/load, renamed devices, deletion errors, switching flight configurations, Apply/Cancel and FC enable/disable. Keep all interface text independent of a particular FC model.
2. **Command capture.** Extend `RTPyroController` and the existing `RTFC` output path. Add focused tests to `RTPyroControllerTest` for accepted/rejected firing, repeated commands and timestamps. Update “logical output only” comments/help, including the Java board template.
3. **Physical bridge and rule suppression.** Add per-run bridge ownership to `FlightControllerSimulatorListener`; connect standard deployment events in both engines. Test a minimal rocket with one chute before adding dual deployment. Verify that firing changes drag, descent speed and the recorded event, while every OR automatic rule is ignored under FC control.
4. **Timing and descent.** Integrate due times with existing engine/stepper limits. Add tests for zero and sub-millisecond delays, board execution costs, built-in CPU timing, PWM independence, warm-up, ground impact, duplicate firing and coast-branch isolation. Verify that the FC still samples and executes after the drogue opens and can deploy the main later.
5. **Plots and logs.** Extend `FlightComputerData`, event metadata, result serialization and `FlightComputerLogReader`; connect the existing 2D/3D displays. Confirm an exported/imported run reproduces both deployment times and device names. Capture events on abort as well as normal completion without duplicating end-of-run records.
6. **End-to-end verification and build.** Run the targeted tests and existing FC actuation/designer regressions, build root `shadowJar`, and exercise the packaged app. Repeated and parallel runs must not share channel state, pending events or rocket references. Document the resulting user workflow and measured timing behavior before calling the feature complete.

## Required acceptance scenarios

| Scenario | Expected result |
| --- | --- |
| FC disabled | Original OR deployment rules and delays work unchanged |
| FC enabled, never fires; OR set to apogee/altitude/ejection | No device deploys through those rules |
| FC enabled, no recovery component or no mappings | Run remains valid; devices stay stowed; discarded commands/unassigned devices are reported |
| Custom board fires channel 2 at completion time 12.345 s | Mapped main deploys at 12.345 s for zero delay, or 12.34525 s for 0.25 ms delay; no 1 ms floor or PWM wait |
| Drogue first, main later | Both physical deployments occur; FC deadlines remain honored throughout descent |
| Two boards/channels fire the same device | One physical event, with a deterministic recorded first cause |
| One channel controls two devices | Both deploy once at their respective configured delays |
| 100% downlink packet loss | Autonomous local recovery timing is unchanged |
| Other extension queues deployment while FC is active | Unowned deployment is suppressed and logged |
| Early firing, high speed, motor still thrusting | Existing OR warnings/abort semantics remain visible; no silent successful-flight classification |
| Saved file has FC enabled but predates mappings | OR rules remain overridden; UI/log clearly report unassigned recovery |
| OR.log import and `.ork` reload | Physical event timing and identity survive; legacy logical-only logs do not acquire fictional deployments |

Use integer-microsecond expected times in tests and a tight floating-point tolerance only when checking OpenRocket's seconds representation. The timing report must separate modeled board completion, configured deployment delay and any numerical scheduling lateness.

## Initial scope

Physical parachute and streamer deployment using existing OR drag models; one active stage, matching the current FC restriction; six shared logical channels; explicit channel-to-device mapping. Separation charges, parachute reefing/cutting away, inflation dynamics, independent pyro banks, electrical firing-energy models and multi-stage FC execution remain future extensions. No requirement to rewrite existing `io.fireRecovery(channel)` programs.
