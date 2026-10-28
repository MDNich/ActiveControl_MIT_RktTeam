# Mission Simulation in OpenRocket MIT 6.5

Mission Simulation combines an OpenRocket flight, simulated telemetry receivers,
up to four ground antenna pointers, video slates and optional real Telemetry UI
workspaces. The result is one portable `.rktmission` ZIP bundle. This guide describes
the implemented software workflow; [the implementation ledger](18-mission-implementation-progress.md)
records the verification revisions. A development version number does not publish
an installer or a GitHub release.

## Start and save a configuration

Open **File → New Mission Simulation…** in OpenRocket. A local browser workspace
opens with six pages. OpenRocket owns the running jobs; refreshing the browser does
not restart them. Keep OpenRocket running while a mission executes.

**Load** opens a saved configuration or `.rktmission-setup`. **Save configuration**
saves editable JSON, including unresolved draft fields. **Portable setup** includes
the frozen rocket, selected motor curves and FC files, and any recorded-flight source.
Use a portable setup when moving between computers. Application locations and private
connection tickets remain preferences on the current computer.

1. **Site & stations:** the launch defaults to `18TUN2061530290` near URRG. Base and
   Away 1–4 use the Telemetry UI defaults. Enter a height and its MSL/ellipsoid datum
   for each enabled site. Heights are deliberately not guessed from imagery. Choose
   zero to four pointers; an Away receiver can also operate without a pointer.
2. Place sites by dragging map pins, clicking **Place on map**, entering latitude
   and longitude, or applying MGRS. Saved MGRS references retain their original
   point convention. Satellite, hybrid and street imagery require the internet;
   coordinates and the offline editor continue working without tiles.
3. **Rocket & flight:** use the current rocket or choose an `.ork`, then select its
   saved simulation. Configure motors, active stages, ignition/separation, recovery,
   guide, atmosphere, wind, solver, FC and extension settings. Inputs are frozen for
   the run. The open rocket document is not modified by these configuration edits.
4. Bind each logical vehicle to its stage for multistage missions. Before separation,
   attached vehicles follow the parent branch; after separation they use their own
   recorded branch. A single active stage can be bound automatically.
5. **Run & results → Validate configuration** reports missing or incompatible inputs.
   Start uses the validated snapshot; later edits belong to a later run.

## Flight, packets and ground links

| Selection | What executes |
| --- | --- |
| Ground Link Analysis, standard source | The ordinary complete OpenRocket flight runs first. Its branches drive standard legacy Zephyrus telemetry and the ground simulation. Ground commands cannot alter that completed flight. |
| Coupled Mission Simulation, custom FC | The existing FC listener runs with accepted physics steps, sensor acquisitions, controllers, PWM and FC recovery. The FC owns its transmission cadence. |
| Paced application rehearsal | The chosen source advances in real time and can connect Telemetry UI workspaces. Pause freezes mission time and simulated freshness while host services remain available. |
| New experiment from saved results | A selected member's recorded trajectory is reused, with either its original transmissions or an explicitly selected standard generator. No FC is rerun. |

Custom FC designs can declare a bounded telemetry format in `.fc` version 3.
Versions 1 and 2 still load. If no format is declared, choose legacy Zephyrus fallback
or require a format. Standard telemetry needs no custom FC; it marks unmodeled hardware
channels unavailable. Missing measured fields are never filled from trajectory truth
at the receiver.

Each station has its own packet loss, bit errors, latency, jitter, outages and optional
UART pacing. A baud setting of zero is software serial; a positive rate adds 8N1 byte
serialization. Receiver packet streams go through the Telemetry UI's decoder and
tracking policy. These are software connections; they do not require an OS virtual
COM port, a USB device or physical serial hardware.

Pointers have initial orientation, slew rates and antenna height above ground. The
geodetic policy uses absolute coordinates; the legacy policy reproduces the legacy
station convention. The default rates and directional beam-width gate are illustrative
models. Terrain obstruction, measured antenna gain tables and firmware/mechanical
qualification are not inferred from a map. Reference ZERO changes the encoder reference
without teleporting the physical mount. Stale data stops new tracking commands and
requires an explicit restart; it does not invent a firmware STOP command.

Timed actions can connect/disconnect stations, start tracking, hold new commands,
set reference zero, point manually, change targets or send supported FC commands
from Base. Base has no local pointer. Uplink is available only while an actual coupled
FC is running. The command menu follows implemented FC opcodes; `0x07` is unavailable.
Recorded command outcomes are simulator diagnostics, not wire acknowledgments.

## Connect the Telemetry UI

Headless policies require no separate Telemetry UI installation. OpenRocket packages
its pinned Python worker and runtime. For visible stations, use a Mission-capable
Telemetry UI checkout/package with the matching shared contracts.

In **Connections**, choose either the Telemetry UI source project plus its Python
interpreter, or its packaged executable. Selecting one launch method clears the
other. **Check selected application** checks capabilities and contract compatibility.
Choose **Interactive Telemetry UI** for the station that should issue pointer commands,
or **Observer UI only** for a read-only view. An observer leaves pointer control with
the headless policy. Base and each Away site can have a separate video application.

To use an application already open, choose **Mission Simulation → Prepare OpenRocket
connection** in that Telemetry UI, then choose its connection file for the desired
station role in OpenRocket. Each role needs its own waiting connection. The existing
application opens a separate SIMULATION workspace; its normal workspace is preserved.
Mission completion closes the simulated session without exiting the existing app.

All mission applications use isolated data directories, loopback network identities
and SIMULATION labels. Physical serial/camera connections and switching into a separate
replay or virtual trajectory are rejected in that workspace. Imported configurations
cannot choose executables or attach applications automatically.

Interactive ownership requires a paced clock. While paused, **Transfer to…** revokes
the old controller before granting the new one, clears pending actions and advances
the ownership epoch. Resume and explicitly restart tracking. Commands queued before
that transfer are not replayed.

A lost mandatory owner pauses the mission and offers **Reconnect**. Restore its waiting
application or choose a replacement, reconnect, then resume and restart tracking.
A new session instance preserves the original session alongside it. Uncertain commands
are not resent. A detached read-only observer does not stop the mission. A failure of
the canonical headless worker fails the member and preserves the committed prefix.

## Video and ensembles

The Balius/Iris presets set stream identities and receiver routing; custom configurations
can choose analog/digital counts and routes. Each stream gets one PNG slate identifying
its analog/digital ordinal, mission, run/member, vehicle and `.ork` filename. Logical
frames refer to that stored image rather than copying it into every frame record.

Configure overall or per-Away-receiver loss, delay and outage windows. Analog visual
interference is an illustrative image effect. In paced connected missions, delivered
slates pass through real H.264/MPEG-TS/SRT services into the Base video wall. A logical
outage longer than the wall's watchdog interval produces an actual stale tile and
recovery. Pause carriers keep host-time leases alive and are excluded from mission
frame counts and optional recordings.

Headless logical video supports up to 64 streams. A connected publisher is limited
to two preview and two main sessions total, so route at most two simultaneously
shown streams to each connected video publisher. Enable **Also save encoded video**
to include delivered video segments in native sessions; all selected enabled video
receivers then need their video applications. Recording failures are surfaced.

Flight ensembles use OpenRocket's existing sampling order and distributions for
motors, atmosphere, wind, mass and aerodynamic coefficients. Each member has separate
FC, link, receiver, pointer and output state. Parallel execution is bounded to eight
workers and available host capacity. Select one application member for visible paced
rehearsal; the remaining members run headless. Failed members are retained alongside
completed siblings unless fail-fast is selected.

## Results, recovery and further experiments

Results are stored under `~/OpenRocket/Missions/jobs/<job-id>/` by default; the workspace
shows the exact bundle and staging paths. A `.rktmission` contains the resolved mission,
frozen inputs, branch truth, original transmitter bytes, per-station receiver/pointer
records, FC diagnostics when applicable, video assets/timelines, native station sessions,
metrics and an SQLite index. Every inventory entry has a length and SHA-256 hash.
Private credentials and application-launch preferences are excluded.

**Open result bundle** is read-only and starts no applications. Select a member and
channel, then read a bounded time window. Plots show recorded measurements or truth
with their stated units. Export a selected station or native session to `.rktflight`
for the Telemetry UI's existing paused replay. The export includes an omission report;
it does not fabricate UTC, physical pointer feedback or hardware acknowledgments.

**Create experiment from selected member** makes a new configuration with a new
mission identity. Choose original recorded transmissions or fresh standard Zephyrus
packets explicitly. The launch, flight settings and vehicle bindings remain fixed;
ground-site positions, links, pointers and video can change. Portable setups include
the selected source history, so the original result file is not needed afterward.
Choose a new rocket to return to a new physics/FC simulation. Recorded-source loading
has a one-million accepted-truth-sample limit and reads TX chunks incrementally.

**Recover interrupted run** opens its staging folder and writes a separate incomplete
bundle using only hash-verified committed prefixes. It ignores uncommitted tails and
leaves the original staging intact. Recovery does not resume an FC or reopen transports.
The bundle can be inspected and exported through the same results controls.

Recording rotates canonical chunks at 16 MiB, forces raw data before indexes and
publishes checkpoints during active recording. Final ZIP publication is atomic after
verification. Limits include 100,000 entries, 64 GiB uncompressed per bundle, 100,000
pending receiver frames/64 MiB of receiver payload, and bounded video/command queues.
Capacity or disk errors fail the member rather than silently dropping canonical logs.

## Scope and qualification

This implementation covers ground-based pointers. Coupled FC execution retains the
existing one-active-stage, one-FC, pad-start restriction. Ordinary multistage flights
use precomputed branch history. Timing, byte streams and deterministic sampling have
software fixtures; these do not establish measured RF, USB, motor-driver or antenna
accuracy. Cross-platform equality applies to declared deterministic streams, not
encoded video bytes, UTC, GUI repaint timing or arbitrary floating-point solver output.

The package qualification builds native application images and runs standard and custom
FC fixtures with the image's Java and pinned Python on Windows x86_64, Linux x86_64,
macOS ARM and macOS Intel. Desktop/attachment/video acceptance additionally runs the
actual Telemetry UI locally. See [the ledger](18-mission-implementation-progress.md)
for run IDs and the final validation record. Publishing a release is a separate step.
