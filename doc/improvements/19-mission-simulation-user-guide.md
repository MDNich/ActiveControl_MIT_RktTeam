# Mission Simulation in OpenRocket MIT 6.5

Mission Simulation combines an OpenRocket flight, simulated telemetry receivers,
up to four ground antenna pointers, video slates and discoverable network stations
for the Telemetry UI. The result is one portable `.rktmission` ZIP bundle. This guide describes
the implemented software workflow; [the implementation ledger](18-mission-implementation-progress.md)
records the verification revisions. A development version number does not publish
an installer or a GitHub release.

## Start and save a configuration

Open **File → New Mission Simulation…** in OpenRocket. A native Java window opens
with six tabs. Closing that window leaves its run active; reopen it from the same
menu. Keep OpenRocket running while a mission executes. No browser is involved.

**Load** opens a saved configuration or `.rktmission-setup`. **Save…**
saves editable JSON, including unresolved draft fields. **Portable setup** includes
the frozen rocket, selected motor curves and FC files, and any recorded-flight source.
Use a portable setup when moving between computers. No Telemetry UI application path, Python path or connection ticket is needed.

1. **Site & pointers:** the launch defaults to `18TUN2061530290` near URRG. Base and
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
5. **Run & results → Validate** reports missing or incompatible inputs.
   Start uses the validated snapshot; later edits belong to a later run.

## Flight, packets and ground links

| Selection | What executes |
| --- | --- |
| Ground Link Analysis, standard source | The ordinary complete OpenRocket flight runs first. Its branches drive standard legacy Zephyrus telemetry and the ground simulation. Ground commands cannot alter that completed flight. |
| Coupled Mission Simulation, custom FC | The existing FC listener runs with accepted physics steps, sensor acquisitions, controllers, PWM and FC recovery. The FC owns its transmission cadence. |
| Real-time network rehearsal | The chosen source advances in real time and publishes simulated Away stations. Pause freezes mission time; network heartbeats and video leases remain available. |
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

OpenRocket includes the station decoder, HTTP/WebSocket publisher, mDNS discovery,
FFmpeg and SRT runtime. It does not launch or locate a Telemetry UI desktop.

1. In **Network**, enable sharing and choose **Local network** or **This computer
   only**. **Advertise now** lets Base join before the flight starts. Starting a
   shared run also starts its stations automatically. The panel lists addresses for
   networks where multicast discovery is unavailable.
2. Open the updated Telemetry UI normally in **Base** mode, choosing the same
   Balius/Iris vehicle layout. Open **Station network**, select an Away entry marked
   **SIMULATION**, and choose **Join selected station's mission**.
3. In the telemetry role, joining a telemetry publisher selects it as the display
   source. Other simulated receivers can be selected from the display-source list.
   In the video role, the Base video wall adopts the advertised stream layout and
   receives the slates using its ordinary leased SRT connections.
4. Start the mission in OpenRocket. Use OpenRocket's timed actions or **Pointer / FC
   command…** controls for simulated commands. Base receives observations and video;
   it does not send hardware commands to the simulator.

Each enabled Away site advertises a telemetry service and, when routed video exists,
a video service using `_rt-gs._tcp.local.`. Every service has a persistent node ID, a
fresh boot ID when restarted, a distinct HTTP port, and the mission UUID. Only the
selected ensemble member is published. Packet data is the actual post-link receiver
output, with the original received bytes; it is not copied from trajectory truth.

Joining a simulation is explicit and never relabels it as live. Normal live stations
can remain discoverable on the same LAN; station-slot conflict detection is scoped
by mission, vehicle and source mode. Simulation acceptance is not restored silently
on application restart. **Stop sharing** withdraws the services after a run ends.
An active run must first finish or be cancelled before changing its publishers.

The runtime uses HTTP/WebSocket control, mDNS discovery and leased H.264/MPEG-TS over
SRT. These are trusted-LAN services; do not expose their ports directly to the public
internet. SRT session tokens bind the requested media stream. No file chooser or
application-launch endpoint is exposed on the LAN. The old private application
attachment protocol remains available for compatibility tools, but it is no longer
the native configuration workflow. Imported application bindings are retained as
migration provenance while native station control moves to OpenRocket.

## Video and ensembles

The Balius/Iris presets set stream identities and receiver routing; custom configurations
can choose analog/digital counts and routes. Each stream gets one PNG slate identifying
its analog/digital ordinal, mission, run/member, vehicle and `.ork` filename. Logical
frames refer to that stored image rather than copying it into every frame record.

Configure overall or per-Away-receiver loss, delay and outage windows. Analog visual
interference is an illustrative image effect. In shared real-time missions, delivered
slates pass through real H.264/MPEG-TS/SRT services into the Base video wall. A logical
outage longer than the wall's watchdog interval produces an actual stale tile and
recovery. Pause carriers keep host-time leases alive and are excluded from mission
frame counts and optional recordings.

Headless logical video supports up to 64 streams. A connected publisher is limited
to two preview and two main sessions total, so route at most two simultaneously
shown streams to each network video publisher. Enable **Also include encoded video
in the result bundle** to save delivered video segments under the selected member
without opening video applications. Network sharing and video feeds must be enabled.
Recording failures are surfaced and committed simulation data is retained.

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

**Open this run’s result** or **Open result…** is read-only and starts no applications. Select a member and
channel, then read a bounded time window. The record table and detail view show the saved values; native time-series plots can
also be opened from a station export in Telemetry UI. Export a selected station or native session to `.rktflight`
for the Telemetry UI's existing paused replay. The export includes an omission report;
it does not fabricate UTC, physical pointer feedback or hardware acknowledgments.

**New experiment** makes a new configuration with a new
mission identity. Choose original recorded transmissions or fresh standard Zephyrus
packets explicitly. The launch, flight settings and vehicle bindings remain fixed;
ground-site positions, links, pointers and video can change. Portable setups include
the selected source history, so the original result file is not needed afterward.
Choose a new rocket to return to a new physics/FC simulation. Recorded-source loading
has a one-million accepted-truth-sample limit and reads TX chunks incrementally.

**Recover interrupted run…** opens its staging folder and writes a separate incomplete
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
macOS ARM and macOS Intel. A separate Telemetry UI matrix exercises actual desktop
software connections and repeated window rebuilds on those four platforms; native
SRT/video-wall, outage and encoded-recording acceptance additionally runs locally.
See [the ledger](18-mission-implementation-progress.md)
for run IDs and the final validation record. Publishing a release is a separate step.
