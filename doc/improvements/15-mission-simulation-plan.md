# Mission Simulation: flight computers, ground stations, antenna pointers and telemetry

Implementation status (28 October 2026): this document retains the reviewed plan and contracts. See [the implementation ledger](18-mission-implementation-progress.md) for completed increments and evidence, and [the user guide](19-mission-simulation-user-guide.md) for the implemented workflow and limits.

Proposed 27 October 2026; final consistency review 28 October 2026. This is an integration plan, not an implementation report. It builds on the OpenRocket MIT source now versioned 6.5 and the RT Telemetry UI source, including a read-only review with the **Telemetry UI** chat. Ground-based antenna pointers are the agreed scope; onboard steerable antennas are outside this first design. The version bump does not imply this feature is implemented.

The proposed feature is **Mission Simulation**: run the rocket, flight computer, radio links, receiving stations and antenna pointers as one experiment. **Ground Link** is the part that models reception and pointing. Save the experiment as a **Mission Bundle**, a versioned ZIP containing configuration, flight truth, FC logs, station observations, pointer commands and motion, and reproducibility information.

The implementation should sit above the existing OpenRocket flight simulation. It needs a new orchestration layer and result model, not another aerodynamic solver. Existing ordinary simulations, ensembles, FC timing, recovery authority and `.ork` files remain the foundation.

The subsequent [configuration workspace and video design](16-mission-configuration-ui.md) specifies a new UI independent of the existing flight dialog, saved configurations, map/Lat-Lon/MGRS site placement, standard legacy telemetry without a custom FC, and configurable analog/digital identification-video streams. It extends the first scope below to include video transport testing; RF video waveform modeling remains outside that scope.

The [final implementation contracts](17-mission-simulation-contracts.md) resolve runtime ownership, supported branch/FC combinations, clocks, codec semantics, height references, pause behavior and storage. Read these three documents together; document 16 owns the work-package sequence and document 17 owns precise cross-application behavior.

## 1. Intended experience

In OpenRocket, choose **New Mission Simulation**, load a mission configuration or choose a rocket and saved flight settings, then select standard telemetry or a custom FC. New drafts start at the URRG launch, Base and four Away positions in document 16; heights remain unset. Place sites on the map or enter latitude/longitude or MGRS. Each station has a receiver profile and each Away station can have a pointer. Advanced launch-relative east/north/up entry declares its origin and altitude reference.

Choose whether to inspect an existing flight, run a repeatable coupled simulation, or connect the Telemetry UI for an interactive rehearsal. The station selector identifies the exact role, receiver, target vehicle, transport endpoint and mission. Connecting and starting reception remain separate actions. Show **Simulated** throughout the UI, including in station/network views and exported observations.

The result view combines the rocket trajectory, station locations, antenna boresight, received position, commanded target and actual simulated mount motion. A shared timeline shows packet loss, stale telemetry, pointer saturation, FC events and command delivery. Select a station to compare its reception and pointing error with another station. Save the complete experiment in one portable file; reopen it paused, with all transports disconnected.

The first headless demonstration uses standard telemetry and Base plus four Away receivers. The subsequent custom-FC fixture uses one active-stage rocket and one offset Away station. In both, the real Telemetry UI decoder consumes simulated receiver bytes, its shared tracking policy produces ordinary pointer-command bytes, and a simulated mount follows those commands. The archive must make the whole chain inspectable.

## 2. What already exists

The audit used OpenRocket source commit `7f0b67ee1c755ec89f154b0f0d2f4e23c843aa87` and Telemetry UI commit `7b5031554c10c535ea722774ad3c92985b02dd14`, together with the working files present during review. These revisions are reference points, not a claim that either working tree was clean. No application code, builds or devices were changed for this plan.

| Existing component | Reuse and integration gap |
| --- | --- |
| OpenRocket `Simulation` and its physics, environments, flight configurations and separated branches | Supply flight truth. Preserve the selected configuration and enabled extensions when constructing a mission run. |
| `FlightControllerSimulatorListener`, `RTFC`, board/controller scheduling and PWM/recovery coupling | Supply the FC execution path. Mission events must obey the same virtual deadlines and output timing. |
| `RTTelemetryEngine` | Produces 128-byte FC payloads and deterministic packet loss/delay logs. It is a byte queue, not an RF model, and currently has one received-output path rather than independent ground receivers. |
| `EnsembleRunArchive` | Already compresses completed member results on disk and retrieves members individually. Reuse this bounded-memory approach for mission members. |
| Telemetry UI `TelemetryDecoder` and `SerialWorker` | Decode framed bytes, record raw data and accepted observations, and deliver through bounded worker/UI queues. A reusable software transport factory is missing. |
| `VirtualPointer` and `VirtualFlight` | Provide a command-consuming mount model and launch-ENU → ECEF → antenna-ENU geometry. Current use is reference-trajectory rehearsal: normal `start_tracking()` rejects the virtual pointer. |
| Telemetry UI station networking | Discovery, HTTP/WebSocket telemetry, snapshot/resume, bounded journals, independent reception archives, video and Base→Away configuration updates are implemented. Some older README/guide descriptions are stale. Remote hardware commands still return 501. |
| `.rktflight` and `SessionRecorder` | Already provide checksummed ZIP export, atomic replacement, SQLite indexes and raw byte capture. The current archive represents one mission and one recording session, not a multi-station FC experiment. |
| `simulation_bridge/src/RocketBridge.java` | Runs a fresh nominal trajectory simulation, clearing extensions. It is not a coupled FC bridge and cannot be used unchanged for Mission Simulation. |

Two concrete prerequisites emerged from the Telemetry UI review. `VirtualPointer` emits `virtual_pointer_tx`, but `recording.py` does not define that raw role; processing it with a recorder attached would fail the role lookup. Continuous mount pose is also not currently recorded. Add a focused regression and settle the recording representation before using virtual-pointer recordings as evidence.

## 3. Simulation classes and ownership

Use three run modes plus replay. These are different execution contracts, rather than different names for playback.

| Mode | Purpose | Timing and limitations |
| --- | --- | --- |
| **Ground Link Analysis** | Compare station placements, pointer parameters and reception against a saved flight and, preferably, saved FC transmissions | Reuses immutable flight/TX history. Can run quickly because no uplink changes the flight. With trajectory alone, label synthetic telemetry or truth-following results explicitly. |
| **Coupled Mission Simulation** | Run physics, FC, links, station software and pointers together | One virtual clock; deterministic for the same configuration, input events, model versions and seeds. Use a headless station controller that shares the UI's decoding/tracking code. |
| **Interactive Mission Rehearsal** | Exercise the actual Telemetry UI and operator workflow through simulated station boards | Paced execution. Host scheduling and human actions affect the first run; record their admission times for repeatable subsequent replay. Accelerated operation needs an explicit lockstep-capable endpoint. |
| **Mission Replay** | Inspect saved experiments and compare stations/runs | Read-only, initially paused. Creates no serial, radio or pointer-control transport. |

Introduce proposed responsibilities such as `MissionDefinition`, `MissionRunner`, `MissionClock`, `GroundStationModel`, `RadioLinkModel`, `PointerPlant`, `StationTransport` and `MissionResultStore`. Names are provisional; the ownership boundaries are the decision. OpenRocket owns rocket truth and FC execution. The mission scheduler owns causal event order. Each station owns its receiver, controller and pointer state. The UI renders or supplies operator input; its repaint rate must not advance flight physics.

```text
Frozen rocket + selected simulation + FC + mission configuration
                              |
                         MissionRunner
                     virtual clock / events
                              |
                OpenRocket flight <--> FC runtime
                       |                 |
                    truth             TX payloads
                       |                 |
                       +------> Ground Link model <---- pointer pose
                                         |
                              station receiver / trailer
                                         |
                             duplex software serial
                                         |
                   telemetry decoder -> station tracking policy
                                         |
                               pointer command bytes
                                         |
                                   pointer plant

Base simulation commands -> selected modeled uplink -> FC input queue
All participants -> append-only result store -> Mission Bundle
```

Truth is available to propagation, mount evaluation and result analysis. A tracking controller receives only its delivered, decoded observations unless the user selected the explicitly named truth-following benchmark. This separation is essential to testing stale data and missed packets.

Coupled custom-FC runs start with one explicitly identified transmitter and one active stage from the pad, matching the current listener's limits. Ordinary standard-telemetry flights can be precomputed with all separated branches, then consumed together on the mission clock. Every payload and observation carries explicit transmitter/branch routing metadata; a legacy packet itself has no vehicle ID. Document 17 defines Iris branch bindings and routing. Multiple coupled FCs and mid-flight FC startup remain outside the first delivery.

## 4. Station locations, coordinate systems and pointer models

A mission definition stores the launch origin, Base and Away station IDs, receiver and pointer locations, target association, antenna mounting orientation, pointer initial pose, controller policy, transport profile, radio profile and independent random-stream identifiers. Use the existing wire IDs `launch` and `gs1`–`gs4`; display the current Base/Away names. Allow receiver and antenna phase-center offsets within one station rather than assuming that the USB board's GPS antenna occupies the pointer pivot.

For every location, retain its source and altitude datum. Canonical geometric positions use WGS84 latitude/longitude and ellipsoidal height. MSL heights need a declared geoid conversion; launch-relative heights need a declared origin. A missing datum is unresolved input, not implicitly zero. Preserve both surveyed truth and the reported/noisy ground position seen by the station software. Model position errors independently from the physical location used in propagation.

Convert rocket and station positions into ECEF and subtract there, then transform the line of sight into the station's local east/north/up frame. Define azimuth clockwise from true north and elevation above the local horizon:

```text
azimuth   = atan2(east, north)
elevation = atan2(up, hypot(east, north))
error     = acos(clamp(dot(unit_boresight, unit_line_of_sight), -1, 1))
```

Use consistent degree/radian conversions at boundaries. At coincident locations the direction is undefined; near zenith retain a defined previous azimuth or mark it undefined, rather than commanding an arbitrary full turn. Include tests across north, the dateline, the poles and the launch-origin offset. Place the pointer firmware's azimuth sign inversion in its protocol adapter, not in the shared geometry.

Offer two separately versioned tracking policies. **Legacy UI tracking** reproduces its frozen receiver latitude/longitude rounded to five decimal places, ground altitude zero and rocket filtered barometric altitude. **Geodetic tracking** uses explicitly compatible target/station height references and configured offsets. The receiver-height firmware/UI discrepancy must remain visible: current documentation describes receiver heights in metres while the legacy decoding/CSV convention divides by 1000. Do not silently reinterpret old files or claim the two policies are equivalent.

The pointer plant records target angles, actual simulated boresight, software zero offsets, travel limits and any saturation. Begin with a documented rate-limited model, then add measured acceleration, backlash, calibration errors and limit behavior when data exists. Existing virtual rates of 90°/s azimuth and 60°/s elevation are illustrative defaults, not qualified hardware specifications. Physical cable-wrap constraints and mount-specific ranges belong to explicit profiles.

Distinguish three quantities everywhere: **requested target**, **simulated physical pose**, and **measured pose**, which is unknown for the present physical protocol. Its ZERO opcode resets software counts at the current position; it does not home the hardware. The protocol supplies no pose acknowledgment, home switch, motion watchdog or stop command. A simulator's internal hold/freeze operation must not be presented as an available physical firmware command.

The present `VirtualPointer` turns ZERO into a target of `(0, 0)`, so it can move the modeled mount back toward that pose. A firmware-faithful profile must instead reset the coordinate offset while preserving physical pose. Reuse the existing model's structure, but test this semantic difference and the firmware deadband rather than assuming its illustrative behavior already matches the hardware.

## 5. Ground Link and closed-loop reception

Introduce an immutable transmission event at `RTTelemetryEngine.send(byte[])`, before the current loss/delay filter. Preserve the existing 128-byte payload encoder and standalone telemetry behavior. A mission link router fans each transmission out to independent receiver links. In mission mode one component owns loss and delay; applying both the old single-link filter and a new station filter would count impairments twice.

Each transmission has a stable ID, FC identity, original bytes and TX time. Each receiver outcome refers to that transmission and adds station identity, modeled arrival time, loss reason, signal estimate and receiver metadata. Keep FC-reported RSSI distinct from receiver-side RSSI. One station's missing packet must not erase another station's successful observation.

Provide increasing levels of fidelity:

1. **Scripted link:** exact loss windows, fixed/variable delay, corruption, disconnection and receiver restart. This is the first deterministic integration fixture.
2. **Statistical link:** independently seeded receiver loss and delay distributions, plus optional declared common outage processes. Stable participant IDs keep random streams unchanged when another station is added.
3. **Directional link:** geometry, ground antenna boresight/pattern and configured propagation/link-budget assumptions influence reception. Store frequency, powers, antenna patterns, receiver thresholds and model version. Treat unmeasured patterns or obstruction models as assumptions, not validated RF coverage.

For each packet, evaluate reception using the mount pose and rocket geometry at the relevant modeled transmission/arrival intervals, as defined by the link profile. A packet cannot improve the pointing that determined its own reception. Decoded positions then drive the next target command; finite slew and future reception close the feedback loop. This permits useful tests of acquisition, stale-position hold, loss bursts, horizon masks, reacquisition and competing station locations.

Measure angular error, time within a configured beam region, first acquisition, longest outage, accepted/rejected/missing frames, age of information, command latency, time at mechanical limits and recovery after outages. Include metric definitions, thresholds and denominators: reception rate is relative to eligible transmissions, and undefined pointing intervals are reported separately rather than counted as zero error.

## 6. Shared time and causal scheduling

Use integer mission-time microseconds to match existing FC traces, encoded as decimal strings in JSON as specified in document 17. Keep mission time, FC boot uptime, wrapped wire milliseconds, host monotonic time and UTC as distinct fields. The current FC warmup/boot offset must be recorded explicitly. Wire clock rollover and packet sequence rollover are protocol events, not a restart unless independent evidence says so.

Integrate mission deadlines with the listener's existing next-deadline mechanism. Preserve accepted physics steps, sensor acquisition, board/controller completion ordering, PWM latching and recovery coupling. Socket or serial worker threads enqueue inputs; only the simulation thread admits them to the FC. Do not mutate flight state or `RTFC` from a UI callback.

Define a stable event key such as `(missionTime, phase, participantId, sequence)`. At a boundary, advance each plant to that time using previously accepted commands; process scheduled transmissions and receiver/serial completions; publish accepted observations; evaluate station control; then admit resulting commands and schedule their effects. Same-time zero-delay events use explicit later phases and cannot recurse indefinitely. Existing FC phase ordering remains authoritative within its adapter, including the current completion/PWM tie behavior. Verify all cross-boundary ties rather than assuming a Java/Python thread order.

For interactive runs, record when each external action entered the mission, its source host time, selected route and accepted/rejected result. Define a late-input rule: admit at the next available boundary and record lateness, or reject after a configured timeout. Do not retroactively change accepted flight steps. A disconnected/slow UI must cause a declared pause, input timeout or observable data gap, not unbounded buffering.

Pause stops virtual deadlines and virtual command expiry. The headless mode can accelerate within resource bounds. A normal Qt/OS-serial UI uses host time today, including a 0.3-second playback step clamp and wall-clock command expiry; production simulation therefore requires clock injection even for paced runs if pause is enabled. Network health and video leases remain on host time. An external UI is not automatically deterministic merely because the physics is seeded. Document 17 defines worker barriers, late actions, paused media and the bounded delivery tail.

At flight completion, stop new FC transmissions according to the declared run policy and optionally advance a bounded station-delivery tail against the recorded final truth state. Classify remaining packets as delivered, dropped or still pending at termination. The current `RTTelemetryEngine.finish(true)` drains with `Long.MAX_VALUE`; carrying that behavior unchanged into a causal station simulation would deliver future packets without advancing the other participants correctly. Cancellation and failure also need explicit partial-result status and queue cleanup.

## 7. Telemetry UI and simulated serial integration

The desired path is a simulated FC board communicating through a selected simulated Base/Away receiver, then into the normal telemetry application. It should exercise bytes, framing and worker behavior, not inject perfect values directly into display fields.

| Boundary | Existing contract to preserve |
| --- | --- |
| FC → receiver | 128-byte FC payload; last byte is the sum of payload bytes 0–126 modulo 256. |
| Receiver → telemetry application | 144-byte frame: `AB AB`, the 128-byte payload and a 14-byte receiver trailer. Trailer data is station-specific and has no independent checksum. |
| Application → pointer | 11-byte command: `AA`, opcode, little-endian float azimuth/elevation and checksum. Preserve absolute/jog/ZERO semantics, firmware sign convention, deadband and shortest-rotation behavior. |
| Application → simulated FC | Existing 16-byte legacy command with reserved bytes, argument/opcode and big-endian checksum; support only commands verified against the selected FC parser and state rules. |

The existing FC `packets.bin` cannot be sent directly to `TelemetryDecoder`: it lacks the two-byte framing prefix and receiver trailer. Add a receiver-board adapter that builds these bytes with an explicit firmware/profile version. It must also model partial reads, multiple frames per read and optional corruption. Preserve numeric flight-state codes and identify the interpretation profile; legacy state names and provisional Balius network names are not interchangeable. Gyro integrals are not attitude, and unknown recovery/motor states must not become fabricated observations.

Refactor transport creation behind a common duplex-byte interface supporting open, close, read, write, polling, connection identity, bounded queues, expiry and error reporting. Preserve `SerialWorker` raw capture and its accepted-observation callback before GUI delivery. Calling `Controller.accept()` directly is suitable for isolated display tests but bypasses the complete receiver/recording path.

| Transport | Proposed use |
| --- | --- |
| In-process duplex bytes | Unit fixtures within one runtime. This is not a way for Java to execute the Python station policy in-process. |
| Local software connection | Java mission process ↔ packaged Python headless worker or external Python UI over the versioned loopback IPC in document 17. Cross-platform default; supports deterministic barriers and paced rehearsal. |
| POSIX pseudo-terminal pair | Optional end-to-end exercise of the existing serial implementation on macOS/Linux. Register its endpoint explicitly: the present port picker relies on device enumeration. |
| Windows virtual COM pair | Optional serial-driver integration test with a separately installed, selected adapter. The ordinary software connection must work without a virtual-COM driver. |

A software serial endpoint reproduces the application-visible byte protocol; it does not by itself emulate USB device enumeration, descriptors, DTR/RTS behavior or electrical timing. Python's [PTY API is Unix-only](https://docs.python.org/3/library/pty.html). pySerial's [socket URL transport ignores serial port settings/control lines](https://pyserial.readthedocs.io/en/latest/url_handlers.html#socket); enabling it would require changing the current `serial.Serial` construction. Its loopback transport echoes bytes and is not a simulated receiving station.

If a profile models UART serialization, declare 115200 baud, 8N1 and ten bit-times per byte; a 144-byte frame then occupies 12.5 ms on that modeled link. Charge each modeled segment once. Actual USB CDC delivery and OS buffering are separate phenomena. Retain frame-generation, radio-arrival, serial-completion, host-read and UI-delivery times so delays can be attributed correctly.

Add an explicit simulation source/capability to the controller and station workspace. Today, the LAN runtime recognizes `simulation`, but the controller's operational modes, live-only publisher and peer acceptance rules do not form a complete simulation workflow. Do not label simulated frames LIVE to bypass those checks. Provide named endpoints such as `Mission / Away 2 / Telemetry` and `Mission / Away 2 / Pointer`, and keep pointer/receiver ports distinct.

## 8. Base/Away selection, networking and uplink

Normal Base profiles have one local telemetry board and no local pointer. Away stations own downlink and pointer transports and have no rocket-command authority. Preserve these roles. The existing virtual pointer can be reused behind a new simulation capability, but enabling telemetry-driven virtual tracking is an intentional new path, not a relaxation of physical-mode checks.

Separate three choices in the mission UI: **which receiver's observations are displayed**, **which station's pointer is controlled**, and **which modeled radio path carries an authorized Base uplink**. Selecting an Away observation source must not grant that station uplink authority. The current Away-route selector is a placeholder, not a working remote-command path.

Initially implement receive-only station integration and pointer control. Then add simulation-only uplink using the existing 16-byte commands that the selected `RTFC.readTelem()` actually supports. Base sends bytes to a modeled ground radio path; delayed surviving commands enter `RTTelemetryEngine.enqueue()` and are consumed at the next eligible FC receive opportunity under document 17's event ordering. Preserve checksum, reserved-byte, state, arming and channel-mask validation. Record send, radio admission, delivery and FC acceptance/rejection separately. A host write completion is not an FC acknowledgment. Standard/precomputed flights have no FC command consumer, so their rocket-uplink controls are unavailable.

The physical Base enable/disable switch and Iris target-selection firmware contracts are undefined. Do not invent those bytes, claim firmware compatibility or unlock LIVE commands. A simulation session exposes declared capabilities through its separate handshake and can acknowledge a simulation-only route selection. It tests only known legacy FC commands; any modeled Away relay must be an explicit experimental topology. Iris attribution in simulated sessions comes from configured routes and metadata; physical packet attribution remains unknown without a qualified identity/switch contract.

Reuse the implemented LAN observation and configuration machinery where its semantics fit. Give every local station instance its own data directory, node/boot identity and service port, sharing only the selected mission UUID and vehicle definition. Add explicit simulation provenance and peer subscriptions that keep simulated and physical observations separate. Remote observations remain a distinct display/archive source and do not refresh local receiver freshness or enable local tracking. Testing a network-fed tracking policy would be a separately selected controller mode.

Preserve bounded journals, reconnect/resume barriers, archive gap records and durable configuration acknowledgments. Freeze location changes into the event tape with their effective time; require the same tracking hold/restart policy as the UI. Video transport already exists; plan 16 adds static identification slates, custom stream configuration and simulation-aware transport testing. RF video waveform modeling and hardware-in-the-loop operation remain outside the first delivery.

## 9. Mission Bundle and efficient results

Use a new outer **`.rktmission` Mission Bundle, schema 1**, using ZIP64 and the validation/snapshot techniques already in `.rktflight`. This names the larger experiment honestly and avoids silently changing the current single-session schema. Add a mission-bundle reader to both applications; the Telemetry UI selects a run and station for inspection. Document 17 fixes the layout, chunk/index records and recovery contract; `.rktmission-setup` is the separate portable-input format.

Keep `.rktflight` schema 1 fully supported. Provide an on-demand export of one selected station/run as a valid `.rktflight`, with a clear report of mission-only data omitted. Do not append unlisted files to existing `.rktflight` archives: current inventory validation rejects them. If an unchanged UI must consume the first prototype, optionally include a compatible `.rktflight` export in the outer ZIP, but do not make nested duplicate sessions the normal storage layout.

Archive overview (the expanded authoritative layout is in document 17):

```text
example.rktmission                     # ZIP64 schema 1
  manifest.json                        # versions, inventory/hashes, completion
  mission.json                         # stations, links, execution policies
  assets/<sha256>/...                   # frozen .ork, .fc, behavior sources, motors
  configurations/<id>.json             # resolved options, mappings, model profiles
  runs/<run-id>/
    run.json                           # seed streams, sampled inputs, clock mapping
    truth/schema.json
    truth/chunk-000001.csv              # bounded chunks, declared types/units/frame
    events/chunk-000001.jsonl           # event order and accepted external inputs
    fc/<fc-id>/OR.log                   # original FC diagnostics
    transmitters/<id>/tx/chunk-000001.bin   # exact standard/custom payloads
    transmitters/<id>/tx/index-000001.jsonl # IDs, lengths, offsets, TX times
    stations/<station-id>/receiver/...  # delivery outcomes, raw frames, times
    stations/<station-id>/pointer/...   # command bytes, target, pose, model state
    stations/<station-id>/sessions/<role>/<instance-id>/... # native snapshots
    stations/<station-id>/network/...   # received network observations and gaps
    video/<stream-id>/source.json       # slate asset, cadence, codec/profile
    video/<stream-id>/events-000001.jsonl # per-receiver delivery/freeze/drop
    metrics.json
    validation.json
  index.sqlite                         # finalized, rebuildable time/chunk index
```

Every channel declares its producer, units, frame/datum, interpolation rule and whether it is truth, measured, decoded, estimated or commanded. Discrete states and actuator latches use step interpolation. Record model/code revisions, resolved FC definitions and behavior source hashes, supported protocol mappings, seeds, station IDs and sampled ensemble parameters. A bundled behavior source is an input asset, not permission to execute it when the file is opened.

Separate compact canonical data from convenience exports. Store a transmission's exact payload once, then reference it from receiver outcome records when unchanged; store station trailers and corruption deltas or complete changed frames as appropriate. Preserve the exact raw byte chunks and timing when protocol/serial behavior is under test. Native UI raw logs may legitimately duplicate bytes needed for independent replay; list that cost rather than quietly discarding evidence. Generate redundant CSV views on demand where existing consumers do not require them.

For the initial interoperable schema, chunked CSV truth with explicit column types and round-trip numeric precision, JSONL events and binary wire records are sufficient. JSON null/validity fields must distinguish absent or non-finite values. Benchmark a typed binary/columnar encoding later against actual Java/Python readers before committing to another dependency. ZIP compression alone does not reduce runtime object retention.

During a run, append to a staging directory with bounded chunk sizes and queues. Flush raw data before committing indexes; checkpoint manifests with committed lengths. Snapshot SQLite through its backup mechanism and committed prefixes, as the recorder already does; do not copy a live database while ignoring its WAL. At finalization, close chunks, compute the inventory, write a temporary ZIP and atomically replace the chosen destination. Retain recoverable staging data if finalization fails.

Use ordinary interoperable Deflate, initially level 3, and store already compressed `.ork`/video/nested archives without unnecessary recompression. Benchmark the chosen defaults. A time index identifies a chunk, not a byte seek into one giant compressed stream. Readers load only the selected station, channels, member and time window; extracting a small finalized index is acceptable. Validate entry paths, duplicates, file types, counts, uncompressed sizes and hashes using bounded reads. Importing a bundle must not execute code or construct transports.

Mark runs and participants `complete`, `cancelled`, `failed` or `incomplete`, with last committed time and loss records. A crash-recovered prefix must never appear as a complete run. Disk-full and archive errors are visible failures with preserved prior output. Reading imported `.rktflight` sessions preserves their original host/UTC timestamps; clock mappings belong in additional metadata, not rewritten native raw headers.

Keep `.ork` as the rocket/design and conventional simulation format. Mission bundles contain frozen input assets and mission results; avoid another full duplicate history inside the bundled `.ork` when a supported design/configuration-only export can preserve all required settings. An optional `.ork` reference to an external mission result needs a content hash and missing-file behavior. It must not quietly make old self-contained saves incomplete.

## 10. Ensembles, placement studies and performance

Support a matrix of flight cases and station layouts, with explicit flight-member, layout, link-repetition and mission-run IDs. A Ground Link Analysis may reuse one immutable flight/FC TX history across many layouts when no uplink changes the vehicle. Coupled missions with FC-affecting commands require a new flight run. Label reused histories so a layout comparison does not masquerade as independent flight uncertainty.

Retain the existing ensemble sampling order and independently seeded model streams. Adding a station or changing worker count must not perturb unrelated FC noise, weather or another station's loss sequence. Parallel workers own independent FC, receiver, mount, recorder and result-store state. Keep only bounded active histories; write completed results promptly and merge summaries in stable run order.

Before choosing performance targets, measure the existing single-flight/ensemble baseline and representative one/four-Away missions. Record peak memory, runtime, bytes per flight minute, compression time, first-open latency and time-window read latency. Acceptance requires bounded growth in resident memory as completed members accumulate and lazy reopening without loading every participant. Publish measured tradeoffs; do not promise a compression ratio before testing real logs.

## 11. Verification strategy

Tests should inspect behavior across boundaries, using independent golden byte vectors and geometry expectations where possible. Round-tripping an encoder through its own decoder is insufficient evidence of a correct protocol or coordinate convention.

| Area | Required evidence |
| --- | --- |
| Geometry | Cardinal directions, launch/station offsets, height datums, dateline/poles, zenith/coincident cases, mount sign conventions and legacy/geodetic policy differences. |
| Byte protocols | Exact 128/144/11/16-byte fixtures; fragmentation/coalescing; bad checksum/resynchronization; unchecked trailer labeling; uint32 uptime and uint16 sequence rollover; numeric state mapping. |
| Causality | A packet cannot improve its own reception; delay changes information age; command arrival respects the next FC receive opportunity; same-time ties preserve FC/PWM behavior. |
| Pointer behavior | Rate/limit/deadband/ZERO semantics; distinction between sent target and pose; stale/disconnect stops new commands and requires explicit restart, without inventing a physical stop opcode; partial-write uncertainty and no automatic retry. |
| Recording | Reproduce and fix the `virtual_pointer_tx` role mismatch; preserve continuous pose and raw commands; retain committed prefixes across cancellation/crash. |
| Serial/software endpoints | Simultaneous receiver and pointer endpoints, independent polling/close, queue bounds, clock injection, pause/acceleration and reconnect identity. Exercise POSIX PTYs separately from the portable software adapter. |
| UI/role isolation | Base has no local pointer; Away has no uplink; simulated observations never appear as LIVE; remote display does not refresh local tracking; replay opens disconnected. |
| Network | Multiple local identities/ports, selected mission, reconnect/resume, gaps, delayed configuration, receiver provenance and independent station reception. |
| Archive | Both applications select and replay the same run/station; v1 `.rktflight` still opens; selected-session export is valid; hashes, truncation, bad paths, disk-full and incomplete manifests are handled. |
| Repeatability | Same inputs yield identical canonical events/metrics, including serial/parallel runs; interactive input-tape replay reproduces admitted actions. Host timestamps and ZIP metadata are excluded from byte-for-byte claims. |
| Regression | Ordinary OpenRocket and FC simulations, timing/recovery/controller behavior, nominal trajectory bridge and physical UI connection rules retain existing semantics. |

After the standard-telemetry fixture, the custom-FC acceptance fixture is one single-active-stage flight and one offset Away station: genuine FC payloads become receiver frames; the real UI decoder feeds the selected tracking policy; resulting 11-byte commands move the modeled mount; a scripted dropout makes data stale and stops new tracking commands; explicit restart resumes tracking. A previously accepted movement can still finish because the physical protocol has no stop command. Reopen the bundle in both applications and compare payload IDs, timestamps, targets, poses and metrics. No physical device is required for these fixtures.

## 12. Delivery sequence

Use the single A–G sequence in [specification 16](16-mission-configuration-ui.md#11-work-packages-and-definition-of-done): shared contracts, complete configuration service/workspace, headless standard mission, custom FC/application integration, static video, ensembles/results and packaged qualification. Document 17 adds precise acceptance gates to those packages. This supersedes the earlier separate 0–6 sequence so two implementation orders do not compete.

Integrate persistence while each source is added. The first end-to-end headless run uses standard telemetry; the genuine FC fixture and video qualification follow. Packaged qualification covers macOS, Windows and the available Linux distribution, including the bundled Python worker. PTY/virtual COM and measured mount/RF profiles are separate qualifications.

The nominal `RocketBridge` remains a separate supported request path. Add a versioned mission request/response protocol rather than changing nominal requests to load executable FC extensions unexpectedly. Freeze the chosen simulation's environment, flight configuration, FC definition and physical assignments; apply existing extension trust rules before execution.

## 13. Decisions and remaining measurements

The recommended architectural decisions are Mission Simulation as an orchestrator, ground pointers only, a virtual-clock core with an optional paced UI, byte-level receiver/serial adapters, explicit Base/Away roles, and a versioned `.rktmission` ZIP with `.rktflight` compatibility exports. Existing source and the Telemetry UI review support these boundaries; they do not establish hardware fidelity.

Measurements still needed are mount rates/acceleration/limits/backlash, calibration conventions, antenna patterns and boresight, radio settings and receiver behavior, ground-height conventions, and any future firmware enable/target/acknowledgment contract. These do not block the scripted software fixture. They determine which later simulation profiles can be described as representative of real equipment.

Do not treat a successful mission rehearsal as qualification of RF range, mechanical pointing or FC hardware timing. The useful claim is narrower and testable: the declared software, models and protocols produced a traceable result under the recorded assumptions.

## 14. Source map for implementation

Paths below are relative to this plan so it remains useful in the repository. The Telemetry UI links assume the two projects remain sibling directories.

| Area | Starting points |
| --- | --- |
| Whole application and FC documentation | [OpenRocket architecture source](../main/main.tex), [FC API and architecture](14-flight-computer-api-and-architecture.md) |
| Flight orchestration | [Simulation.java](../../clone/openrocket/core/src/main/java/info/openrocket/core/document/Simulation.java), [FlightControllerSimulatorListener.java](../../clone/openrocket/core/src/main/java/info/openrocket/core/simulation/listeners/FlightControllerSimulatorListener.java) |
| FC timing and uplink parsing | [RTFC.java](../../clone/openrocket/core/src/main/java/edu/mit/rocket_team/FC/RTFC.java), [FlightComputerTimingSettings.java](../../clone/openrocket/core/src/main/java/edu/mit/rocket_team/FC/FlightComputerTimingSettings.java) |
| FC packets and output | [RTTelemetryEngine.java](../../clone/openrocket/core/src/main/java/edu/mit/rocket_team/telemetry/RTTelemetryEngine.java), [TelemetryLinkSettings.java](../../clone/openrocket/core/src/main/java/edu/mit/rocket_team/telemetry/TelemetryLinkSettings.java), [FlightComputerOutputSettings.java](../../clone/openrocket/core/src/main/java/edu/mit/rocket_team/telemetry/FlightComputerOutputSettings.java) |
| Existing result archive | [EnsembleRunArchive.java](../../clone/openrocket/core/src/main/java/info/openrocket/core/simulation/ensemble/EnsembleRunArchive.java) |
| Wire/role documentation | [PROTOCOL.md](../../../RT_Telemetry_UI/docs/PROTOCOL.md), [V0A_STATION_MODES.md](../../../RT_Telemetry_UI/docs/V0A_STATION_MODES.md) |
| Serial and decoding | [devices.py](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/devices.py), [protocol.py](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/protocol.py), [rocket_protocol.py](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/rocket_protocol.py) |
| Station control and geometry | [controller.py](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/controller.py), [virtual_pointer.py](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/virtual_pointer.py), [domain.py](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/domain.py), [station_profile.py](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/station_profile.py) |
| UI and network integration | [station_workspace.py](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/station_workspace.py), [network_bridge.py](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/network_bridge.py), [network/runtime.py](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/network/runtime.py), [network/observation.py](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/network/observation.py) |
| Recording and portable files | [recording.py](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/recording.py), [flight.py](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/flight.py) |
| Nominal bridge | [RocketBridge.java](../../../RT_Telemetry_UI/simulation_bridge/src/RocketBridge.java) |
| Existing regression entry points | [test_serial.py](../../../RT_Telemetry_UI/tests/test_serial.py), [test_virtual_pointer.py](../../../RT_Telemetry_UI/tests/test_virtual_pointer.py), [test_recording.py](../../../RT_Telemetry_UI/tests/test_recording.py), [test_flight.py](../../../RT_Telemetry_UI/tests/test_flight.py), [test_network_runtime.py](../../../RT_Telemetry_UI/tests/test_network_runtime.py) |

This plan was checked against source and the other chat's read-only review. Implementation tests, application builds and hardware qualification remain work for the phases above.
