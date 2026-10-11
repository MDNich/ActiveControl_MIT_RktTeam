# Flight computer recovery: implementation and use

Implemented 11 October 2026 in the MIT 6.4 source and rebuilt application JAR. This implements [the recovery coupling plan](10-flight-computer-recovery-coupling.md).

## Using it

1. Open a simulation's **Flight computer** tab and enable flight computer simulation.
2. In **Recovery**, assign channels **0–5** to the rocket's parachutes or streamers. Each device has six channel checkboxes. Multiple channels can trigger one device; a channel can also trigger multiple devices.
3. Set the device's **Delay (ms)** if needed. The default is zero; fractional milliseconds are supported and rounded to the nearest microsecond. Enter commits an edited delay.
4. Have a custom Java board call `io.fireRecovery(channel)`, or use the FC's existing state-machine/built-in recovery actions. Existing programs need no API changes.
5. Run the simulation. Use **Plot data** to select the **Flight computer: recovery firing** or **recovery armed** preset, or select individual **FC: recovery deployed [device]** variables. The 3D canopy follows physical deployment events.

**With FC simulation enabled, all OpenRocket automatic recovery deployment rules and their delays are ignored.** Unassigned devices remain stowed. Missing logical recovery hardware and commands on unmapped channels are valid and logged as discarded; they do not trigger fallback deployment. Existing saved simulations need explicit channel assignments before an FC can deploy their recovery devices.

Disabling FC simulation restores the saved OR rules. Assignments remain saved in the simulation's `.ork` extension configuration. Portable `.fc` files continue to describe logical hardware and programs; they do not contain rocket-specific device references. Assignments use UUIDs, so device renaming is safe. Deleted references are reported at startup and can be removed with **Clear selected assignment**. Inactive devices are identified and skipped.

All Java boards currently share one six-channel recovery namespace. The first accepted firing schedules a device once; subsequent firings cannot restart its delay or deploy it again.

## Timing and physical behavior

Accepted `RTPyroController.firePyro()` calls emit immutable records with command sequence, channel, exact boot microseconds and origin. Java board IDs are preserved. Built-in, state-machine and received radio command paths use the same connection. Firmware arming, continuity and pulse monitoring are unchanged.

The listener's per-run bridge queues normal OpenRocket recovery events at:

```text
deployment time = accepted command completion time - pad warm-up + device delay
```

The 250 ms output pulse is not a deployment delay. Recovery does not wait for PWM refresh, and autonomous recovery does not depend on received telemetry. Negative simulation-time warm-up commands are discarded physically; time-zero commands are eligible. Commands after landing are discarded.

Both event engines suppress ordinary OR recovery scheduling while the FC listener is active. Only bridge-owned recovery events are accepted; other extensions cannot accidentally restore an automatic deployment path by queueing an ordinary recovery event. The existing parachute/streamer drag model, deployment warnings and under-thrust abort behavior remain in force. Drogue deployment switches to the existing descent integration while the FC continues executing and can deploy the main later.

Timing limits cover sub-millisecond events in both engine paths. The basic engine now shares the FC deadline behavior already present in the modified engine. Auxiliary optimum-coast calculations exclude the live FC, so an early deployment does not run the same controller a second time or corrupt its clock/log.

## Logs, plots and saved results

The existing console trace (`System.out.println`) and `OR.log` contain `recovery.authority`, `recovery.mapping`, `recovery.schedule`, `recovery.deploy` and `recovery.discard` records. Scheduling/deployment records identify the device, channel, origin, due time and actual time; deployment records include numerical lateness in microseconds. Rejections and listener vetoes remain separate from physical deployment.

Each channel has armed/pulse plot variables, and each configured device has a deployed-state variable. Digital variables use step plots. Exact transition records supplement nominal 10 ms FC log samples, and import merges those transitions at their recorded times. Ensemble averaging holds digital states between samples and labels recovery averages as fractions of runs.

Physical event metadata survives `.ork` save/load and log import, even without the original rocket. 2D event labels and 3D event navigation retain device names. Imported aborted runs preserve the abort event/cause. Legacy logs remain readable; historical logical firing messages do not acquire fictional physical deployment events. The GS-compatible radio packet/telemetry CSV format is unchanged.

## Verification

The broad regression run passed **87 core tests and 17 Swing/plot tests**, covering FC timing, actuation, designer/runtime behavior, telemetry, ensembles and motor batches. After strengthening descent and abort-import checks and adding two plot tests, the final targeted run passed **22 core tests and 3 Swing/plot tests**. The root `shadowJar` build succeeded.

Specific recovery checks cover:

- Accepted versus unarmed firing; retriggering during an existing pulse; microsecond timestamps and board origin.
- Board completion at 5.000250 s plus 0.250 ms device delay producing deployment at **5.000500 s**, unchanged by PWM phase and 100% downlink packet loss.
- Zero-delay deployment at **5.000345 s** in both engine implementations.
- Drogue deployment at **8.000000 s**, followed by main deployment during actual descent at **18.000250 s**. The FC clock continues across the descent-stepper transition; the resulting downward speed at 20 s is approximately **14.70 m/s** for the synthetic fixture.
- One channel deploying a parachute and streamer with separate delays; redundant channels producing one physical event.
- Exclusive FC authority, restoration of saved OR rules when disabled, absent hardware, unassigned channels, warm-up, ground-hit ties, listener vetoes and the existing under-thrust abort.
- Recovery mapping persistence, renamed/deleted components, exact imported deployment transitions and event metadata.
- Native recovery controls, step-plot rendering and the 3D canopy/name behavior of an imported event.

The [packaged desktop verification program](../../sim/FCsim/RecoveryDesktopVerification.java) exercised the real simulation dialog using only the finished JAR. It selected channel 2 and a 0.375 ms delay, then ran the same simulation twice. Both runs deployed at **4.000375 s**, each with **0.0 µs recorded scheduling lateness**, separate output files and matching imported device/event metadata. It also saved a synthetic `.ork`. These fixtures are software checks, not comparisons to actual flight data.

Local verification images, the packaged example and its logs are under `clone/openrocket-release-24.12.RC.01/build/fc-recovery-verification/`; Swing plot/control images are under `swing/build/fc-recovery-verification/` within that source tree.

## Build and scope

The artifact is `clone/openrocket-release-24.12.RC.01/build/libs/OpenRocket-MIT-v6.4.jar`. The compatibility JAR was refreshed by the build, so the installed `/Applications/OpenRocket_MIT.app` will use this implementation after restart. No release was published.

The bridge models an accepted mapped firing as an ideal physical deployment after the configured delay. It does not infer an electrical continuity break or pyro success from canopy deployment. Existing limitations remain: one active stage, one shared channel bank, the current OR recovery drag model and held descent attitude. Inflation dynamics, reefing/cutaway, separation impulses and electrical firing-energy models are outside this change.
