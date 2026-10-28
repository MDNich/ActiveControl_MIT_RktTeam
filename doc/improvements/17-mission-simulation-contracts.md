# Mission Simulation: implementation contracts and final review

Reviewed 28 October 2026 against the OpenRocket MIT and RT Telemetry UI working trees. **This is the implementation handoff, not an implementation report.** OpenRocket's application version is now 6.5; that version bump does not mean Mission Simulation is implemented.

Implementation evidence is maintained in [18 — progress and qualification](18-mission-implementation-progress.md).
The delivered connection mechanism uses a private, locally selected connection file
from Telemetry UI's **Mission Simulation → Prepare OpenRocket connection** command.
It opens an isolated simulation workspace in the existing process, checks its boot
identity and exact contract hash, and never converts an existing LIVE workspace.
Ownership transfer is available between a headless controller and an already
connected observer/interactive application while paused. A failed ownership handshake
leaves the member paused for cancellation/restart, without granting a second owner.
An ensemble opens applications only for its explicitly selected member; all others
record a headless ownership override. Optional encoded recordings cover each selected
Away video receiver's delivered frames and retain host-clock/mission-clock sidecars.

Read [15 — architecture](15-mission-simulation-plan.md) for the system, [16 — configuration specification](16-mission-configuration-ui.md) for the user interface and work packages, and this document for the resolved cross-application contracts. These contracts take precedence over illustrative values in the browser prototype. Changes during implementation must update the affected contract and its shared fixture together.

The final review resolves runtime ownership, single-stage FC limitations, packet identity and height semantics, deterministic timing, station startup, video pause behavior and recoverable storage. Hardware measurements remain inputs to qualified profiles; they are not prerequisites for the explicitly illustrative software models.

## 1. Supported execution and vehicle identities

| Configuration | Execution contract for the first delivery |
| --- | --- |
| Standard telemetry, ordinary single/multistage rocket | Run the complete selected OpenRocket flight first, then run Ground Link Analysis or paced rehearsal against its immutable branches. Generate standard packets from those branches. Ground commands cannot change this precomputed flight. |
| Saved flight and saved FC transmissions | Ground Link Analysis against recorded truth/TX events. Missing TX history requires the explicitly selected standard generator; do not imply that FC code was rerun. |
| Custom FC, one active stage, pad startup | Coupled Mission Simulation or paced rehearsal with the existing FC listener, accepted physics steps, controller/PWM/recovery ownership and optional known-command uplink. |
| Custom FC with multiple active stages, multiple concurrent FCs or mid-flight startup | Preserve the draft, but reject this run configuration with a targeted explanation. This requires a later multi-body FC scheduler and is not silently reduced to one stage. |
| Result replay | Read-only and initially paused, with no connected transports. A new experiment derived from a result gets a new run identity. |

The restrictions above come from `FlightControllerSimulatorListener.startSimulation()`, which currently rejects anything other than one active stage and time-zero startup. `BasicEventSimulationEngine` processes separated branch histories sequentially. The mission layer must not mistake its branch callbacks for a simultaneous multi-vehicle clock. Precomputing ordinary flights preserves staging and recovery while allowing synchronized ground-link playback of all branches.

Define `vehicleBindings` in the configuration: logical vehicle ID, rocket asset hash, selected flight configuration ID, component/stage UUID, branch lineage and active interval. A display name or branch list index is not identity. Before separation, a child vehicle uses its attached parent's trajectory with its declared physical offset; afterward it uses its own branch. Bind exact event boundaries to one side and record that choice. A branch ending in an abort has no valid future truth; a landed branch can hold its final pose. Never extrapolate an aborted trajectory to finish a mission.

For precomputed flights, retain accepted solver samples and discrete events. Interpolate position in canonical ECEF and continuous scalar observations linearly between samples, use quaternion interpolation for orientation, and hold discrete states until their event boundary. Do not interpolate across separation, deployment or another discontinuity: record both sides, with the post-event state authoritative at the event time. Store this interpolation profile and maximum sample spacing. High-rate synthetic packets cannot imply more physical accuracy than their underlying flight samples.

Iris presets define Sustainer/Booster telemetry routes and the three video channels in specification 16. They do not instantiate two FCs. A full two-vehicle standard-telemetry rehearsal requires both branch bindings. Base receives one selected vehicle at a time; each Away receiver has one configured telemetry target. Broadcast each vehicle's packet to every receiver tuned to that vehicle. Balius has one vehicle, so all enabled receivers receive that source. Simulation-only Base target changes are recorded control events with acknowledgments from the simulator, not invented firmware bytes. Flush partial receive frames on a switch and identify the new receiver stream epoch.

A received legacy frame has no vehicle ID. Use configured routing and explicit simulation session metadata for attribution; never infer vehicle identity from values, state numbers or a stream label. Keep unknown attribution unknown for physical/imported sessions. One pointer has one selected tracking target and one physical pose, even when several video channels share its antenna.

## 2. Runtime boundary and controller ownership

Java owns flight/FC execution, mission time, truth sampling, modeled links, serial serialization, pointer plants, validation and the canonical result store. Extract the Telemetry UI's decoder and station tracking policy into a Python module usable without Qt. Both the UI and the headless station worker import that same module. Do not maintain a separate Java approximation of the UI policy.

Launch one isolated Python worker per active ensemble member, containing that member's station controllers. Package a pinned worker and its Python runtime with OpenRocket; a headless mission must not require an external Telemetry UI installation or a developer Python environment. The packaged application must locate this worker through its installation manifest. An explicitly selected developer override is recorded in the run provenance. This packaging dependency is part of packages C and G.

Each station has exactly one command owner: `headless-policy` or `interactive-ui`. Observer UIs may display the same data but cannot also run automatic pointing for that station. Changing owner requires a paused run, queue resolution, an ownership epoch and explicit tracking restart. The initial UI-connected fixture uses the interactive UI as owner; the deterministic fixture uses the headless worker. Both consume the same bytes and use the same policy implementation.

The station policy receives decoded delivered observations, reported station position, clock, settings and operator events. It never receives truth pose, packet-loss decisions or a noiseless target through a convenience API. A separately named truth-following benchmark can access truth; it must be labeled in results and cannot pass the telemetry-tracking acceptance fixture.

Use the current policy's 5 Hz tracking limit, 2-second freshness threshold and 500 ms pointer-target queue expiry as the initial profile, all evaluated in mission time. A checksum failure, repeated old sample or unrelated remote observation does not refresh the local tracking source. Reacquisition restores usable data but does not automatically restart tracking after a stale/disconnected stop.

Store headless operator actions in an explicit scenario tape. The default fixture connects/polls at prelaunch start and requests tracking once at mission time zero, after the warmup observations; a failed request is recorded, not retried silently. Dropout fixtures include an explicit later restart action. In an interactive run the operator or a visibly selected startup script owns these actions. Loading a draft never executes the tape; starting the configured mission does. No default startup script sends FC arming, firing or state-change commands.

## 3. Clock, ordering and termination

Use signed 64-bit integer microseconds. Encode these as decimal strings in JSON to avoid JavaScript integer truncation. Mission time zero is physical flight start. The existing one-second FC warmup occupies mission time −1,000,000 through 0 with the vehicle on the pad; FC boot microseconds equal mission microseconds plus the recorded boot offset. Standard sources use the same nominal prelaunch interval. Preserve FC-generated uptime/state behavior rather than rewriting it to match a synthetic clock.

At a deadline, use this stable order, with participant ID and sequence breaking ties:

1. Advance continuous plants/flight to the boundary using previously accepted inputs. Respect the FC adapter's existing internal acquisition/completion/PWM order.
2. Complete existing FC work and collect TX/flight events. Integrate mission deadlines with accepted physics steps; do not interpolate an FC decision from a rejected solver step.
3. Evaluate transmission eligibility and receiver outcomes with the pose available at that instant; complete scheduled radio/serial delivery.
4. Decode delivered chunks, publish accepted observations and run due station-policy ticks.
5. Admit station commands and operator actions, apply pointer targets, and queue FC-bound input. An uplink first delivered at this boundary is visible at the next eligible FC receive opportunity, not retroactively to the FC work in step 2.
6. Commit ordered events and advance to the next deadline. Zero-duration follow-on events use later phases; reject a model that generates an unbounded same-time cycle.

In lockstep, Java sends each worker its next deadline and delivered bytes, waits for its acknowledged actions, then advances. Socket arrival order does not define virtual order. The worker must not run wall-clock timers for tracking, freshness or expiry. An independent host watchdog detects a dead worker without changing simulated timeout semantics.

In paced interactive mode, admit external input at the next unprocessed boundary. Record host arrival, admitted mission time, route, event ID and lateness. Default operator-command lifetime is 500 ms of active mission time; reject expired commands and report why. Do not accumulate operator commands while paused: configuration can be edited as a new draft and explicit paused control actions can be recorded, but flight-affecting commands wait for a fresh user action after resume. Partial/uncertain writes are never retried automatically.

Pause freezes mission timers, pointer motion, simulated freshness and virtual expiry. Real sockets, process health and SRT leases still use host time. A lost mandatory participant pauses a rehearsal and requires explicit reconnect/restart; a headless worker failure fails that member. A detached read-only observer does not stop the run. While running, station dropout may be a configured scenario and produces stale data normally.

Default end policy: stop TX generation when all selected valid branches end, then advance a **2-second delivery tail**, configurable from 0 to 60 seconds. Hold only valid landed final poses; do not generate fresh synthetic measurements in the tail. Run queued station/link events on the clock and classify remaining items as pending-at-end. Never use `finish(true)`/`Long.MAX_VALUE` to deliver all future packets. Cancellation stops generation immediately, records pending items and finalizes a committed partial result; it does not run the normal tail.

## 4. Coordinates, heights and pointer semantics

Retain the exact six URRG coordinates and original MGRS strings from specification 16 and `mission-ui/urrg-sites.json`. Production manual MGRS entry uses the **grid reference point**, matching Telemetry UI. Imported cell-center coordinates retain that convention and their existing canonical position; do not silently reinterpret them. The browser preview's cell-center input is a known migration difference. Preserve entered precision independently of survey accuracy. Reject unsupported polar MGRS with a latitude/longitude alternative.

Canonical geometry is WGS84 latitude/longitude and ellipsoidal height `h`. Retain orthometric/MSL height `H` and geoid separation `N` separately: `h = H + N`. Use a pinned EGM96 grid/interpolator for the normal offline conversion, with model/data hashes in the resolved configuration. A manually supplied constant separation is an explicitly labeled local approximation with a stated area of use, not an implicit zero. The conversion convention follows [GeographicLib's GeoidEval documentation](https://geographiclib.sourceforge.io/C++/doc/GeoidEval.1.html).

The OpenRocket adapter supplies the solver's launch/atmosphere altitude in its established MSL convention and records the conversion into canonical geometry. Do not feed ellipsoidal heights into atmosphere settings without conversion. Every active physical site needs a height and resolved conversion; disabled sites may remain incomplete. Map clicks/MGRS supply no height. The default presets continue to have null heights. Missing grid data or a missing declared approximation blocks the dependent run, not draft editing.

Receiver-reported position, survey position, pointer pivot and antenna phase center remain distinct. A mount offset is local ENU metres. Default offset is zero except for the explicitly displayed mount height. Convert the target's *reported* height according to its codec profile before geodetic tracking. Existing FC `RTGPS.getHeight()` subtracts a mutable zero reference; record zero events and the corresponding datum/reference in simulation metadata. An imported legacy session lacking that reference cannot silently enable geodetic GPS tracking; use the explicitly selected legacy/barometric policy or resolve the reference.

The initial illustrative mount uses rates 90°/s azimuth and 60°/s elevation, unlimited continuous azimuth and physical elevation 0–90°, with configured initial pose and zero offsets. Its profile has no acceleration/backlash model. If acceleration limits are selected, use a separately versioned acceleration-limited plant with defined stopping behavior; do not display ineffective controls. Clamp travel targets and report saturation. Choose the shortest allowed azimuth path; break an exact 180° tie in the positive direction. Near zenith, retain the last azimuth; at zero line-of-sight distance record undefined pointing.

Keep exact firmware-byte parsing/sign/deadband behavior in a pointer adapter with golden vectors, separate from geometric coordinates. ZERO changes the software reference at the current physical pose. Stale tracking stops new tracking commands and requires explicit restart, matching the UI; the plant may still finish a previously accepted movement because the physical protocol supplies no stop/watchdog. An optional simulator freeze is a separate experiment event. Internal pose is simulated truth, never an acknowledgment or measurement from real hardware.

## 5. Standard and custom telemetry

Freeze `legacy-zephyrus.v1` as a named **wire layout**, separate from the source and semantic profile. The 128-byte payload checksum, 144-byte receiver frame, 11-byte pointer commands and 16-byte uplinks remain exactly as documented in Telemetry UI's [PROTOCOL.md](../../../RT_Telemetry_UI/docs/PROTOCOL.md). Do not apply the provisional Balius network state mapping to legacy Zephyrus state numbers.

For the standard source, default to TX at 20 Hz, GPS sampling at 10 Hz and synthetic barometric sampling at 20 Hz. Hold a sensor sample until its next acquisition; sending another packet does not resample its noise. These are standard-source defaults, not overrides to an FC's timing. A default standard source emits throughout prelaunch/flight and stops at the terminal policy above.

| Legacy field | Standard-source contract |
| --- | --- |
| GPS fix, latitude/longitude | Modeled fix state (default 3), signed E7 degrees from the latest noisy observation. Invalid fix retains wire placeholders but makes position unavailable to tracking. |
| GPS height, float at byte 40 | Noisy MSL height above the launch MSL reference, metres; semantic profile `gps_agl`, with that fixed reference in metadata. Convert through the declared geoid for geodetic tracking. |
| GPS accuracy/satellites | Unavailable unless a selected sensor model supplies them; zero wire placeholders, invalid in the semantic field mask. |
| Filtered barometric height, float at 59 | Synthetic barometric height above launch in metres. Initially ideal height plus configured bias/noise; identify this as an observation model, not an executed hardware filter. |
| State at 63 | 255, unknown hardware state. Record flight phase from OpenRocket events separately; do not manufacture FC state transitions. |
| Uptime/sequence at 80/84 | Unsigned 32-bit boot milliseconds and unsigned 16-bit transmitted-packet sequence, wrapping naturally. Starts from zero at source boot; increases even when every receiver loses a packet. |
| Velocity at 122 | Synthetic vertical velocity in m/s, positive up, with validity/provenance. It is not an actual accelerometer integration result. |
| Other sensor, battery, current, pyro, servo, gyro-integral and maximum fields | Zero wire placeholders and unavailable semantic values in the initial standard profile. No synthetic attitude from gyro integrals. Later sensor models explicitly version any newly valid field. |
| Receiver trailer | Own receiver's modeled position/fix and RSSI, with the firmware's metre height encoding. Preserve raw value and separately expose the legacy divide-by-1000 display discrepancy. The trailer has no independent checksum. |

Carry field validity, `simulation` source, transmitter identity, vehicle/branch, height reference, model version and observation ID beside the bytes through decoding, recording and network publication. Apply this before consumers see the decoded sample; the current decoder hardcodes `LIVE`. Byte compatibility alone is insufficient for the full mission feature. A legacy-only external viewer can be tested as a transport sink, but cannot claim correct unavailable-field/provenance presentation. Preserve custom FC packet values and explicitly describe its existing zero placeholders instead of substituting the standard profile's mask indiscriminately.

Add `.fc` **version 3** with optional `telemetryFormat`; continue reading versions 1 and 2 unchanged. Migrate the controller-block version check explicitly rather than accidentally rejecting v3 controllers. If absent, obey the saved legacy-fallback/require-format choice. In v3 the declaration identifies a codec schema/version, immutable layout asset and bindings to named FC snapshot values. Capture values at the FC's transmit point and preserve its rate ownership.

Initial custom codecs are bounded declarative fixed frames: 1–4096 payload bytes, explicit field offsets, integer/IEEE float types, byte order, scale/offset, units, validity and clock semantics; byte framing and receiver trailer are separate declarations. Support no checksum, sum8 and a fully parameterized CRC16; each requires independent vectors. Reject overlapping/out-of-bounds fields, unknown source bindings, non-finite integer conversions and unsupported checksum/framing schemes. Do not evaluate arbitrary expressions/code from a codec file. More complex existing formats require a reviewed adapter and advertised capability, not guessed interpretation. Both applications must advertise the exact codec and semantic-profile hash before starting a UI-connected run.

## 6. Noise, loss, serialization and random streams

The baseline profile is an ideal link: zero packet/frame loss, zero corruption, zero delay/jitter and zero sensor noise. The preview's nonzero values are illustrative, not production defaults or measured URRG behavior. Retain loaded values without resetting them. Standard GPS/barometer bias and noise are explicitly configured; FC sensor noise remains owned by its FC definition.

GPS horizontal σ means the standard deviation of each independent local east/north axis; vertical σ applies to up. Barometric σ is metres. A per-run bias is drawn once; white noise is drawn per acquisition. A shared transmitted sample is identical for every receiving station. Receiver position noise has its own source clock/seed; it is not another rocket-sensor noise pass.

Independent link loss is a Bernoulli probability per eligible transmission. Jitter is Gaussian added to fixed delay and clamped at zero; FIFO delivery is on by default. A two-state burst option uses declared good/bad loss probabilities and per-transmission transition probabilities, initially good. Scheduled outages use half-open mission intervals `[start,end)` and take precedence over random loss. Save attempted/eligible/dropped counts separately. Corruption independently flips payload bits after encoding/checksum and never recomputes the checksum; header/trailer corruption is a separately selected serial-fault test.

Processing order is route eligibility → outage → directional/link-budget qualification (when selected) → random loss → byte corruption → radio delay → receiver framing → optional UART serialization → host delivery. Downlink, uplink, pointer serial, station LAN and video impairments have distinct profiles; charge each delay/loss stage only once. Initial direction-dependent reception samples geometry/physical boresight at TX time. Record that assumption; optional packet-duration/interference models are separate versions. A scripted profile needs no fabricated RSSI: mark its wire placeholder unavailable. The first directional profile uses a supplied gain-vs-off-axis table and sensitivity threshold; absent measured data, label the selected table illustrative. Frequency, power, gains, losses, sensitivity and horizon-mask assumptions must all be explicit, with no implicit terrain model.

Default software byte delivery has no physical UART delay. An explicit `uart-115200-8n1` profile adds ten bit-times per byte on each selected segment (144 bytes = 12.5 ms), preserves per-endpoint FIFO order and records completion time. Do not count the receiver's framing bytes as an additional RF payload unless its radio profile says so. Fragmentation/coalescing changes read chunks, not transmitted bytes.

Reordering is an advanced fault profile. The current decoder treats a small backward uptime as a restart; fix the simulation decoder adapter so recorded transmitter epochs distinguish delayed packets from reboot/rollover. A delayed older observation remains in history but cannot replace the controller's newest accepted target or refresh freshness. Without that capability, reject reordering tests rather than silently reporting false reboots.

Use the versioned `sha256-counter-v1` sampler for new mission models. Hash a sequence of UTF-8 fields, each prefixed by a four-byte unsigned big-endian byte length: sampler version, unsigned master seed in decimal, member sampling ordinal, stable participant ID, phenomenon, model version, TX/sample/frame ordinal and draw ordinal. IDs are used exactly as stored; generated run UUIDs, host times and worker indexes are excluded. Take the digest's first 52 bits as unsigned `n`, and set `u = (n + 0.5) / 2^52`, strictly between zero and one. Normal draws use Box–Muller with two consecutive uniforms, `sqrt(-2 ln u1) cos(2π u2)`. Commit independent hash/uniform vectors and tolerance-based normal vectors in package A; do not use language-native default RNGs. Event-addressed draws prevent skipped deliveries from shifting later samples. Existing flight/FC ensemble sampling remains unchanged and independent. A link repetition has an explicit sampling ordinal; layout comparisons can reuse it intentionally for common random inputs, and record that relationship.

## 7. Software serial and application handshake

Use a loopback TCP connection for Java↔Python mission IPC, distinct from station LAN HTTP/WebSocket/SRT. Protocol name is `rt-mission.v1`: a four-byte unsigned big-endian length followed by UTF-8 JSON, at most 1 MiB per message; byte chunks are base64, at most 64 KiB decoded. Packet limits are separate from chunk limits. No newline parsing or ad hoc shell protocol. The JSON envelope contains protocol, type, message ID, session/mission/run/member/participant/connection IDs, sequence and decimal-string mission time.

Messages cover `hello/capabilities`, `configure/ready`, `advance/advanced`, `rx-bytes`, `tx-bytes`, `action/result`, `pause/resume`, `close/closed` and `error`. In deterministic mode, one `advance` batch includes deliveries/actions for one deadline and `advanced` acknowledges the last consumed sequence plus produced writes. Per-session credentials are passed in a private launch manifest, never archived. Capability mismatch is a preflight error. Reconnect creates a new connection epoch and resets framing state; it does not resend uncertain commands.

Use bounded channel queues, initially 256 messages or 4 MiB, whichever is reached first. Record effective limits. A lockstep producer waits before advancing. In paced mode, overload pauses with a visible reason unless a declared test deliberately drops display/network observations with explicit gap records. Never silently drop FC inputs, canonical events or raw recorder data to make a run appear successful. Host watchdog is 10 seconds by default and does not count as a simulated outage.

The launch manifest identifies the configured executable/interpreter, role/site, private data directory, mission IDs, IPC endpoint, codec/profile hashes, streams, clock mode and requested LAN bindings. Construct process arguments as an argument list. Add an actual documented manifest option to Telemetry UI; do not claim current CLI support. Attached applications exchange the same manifest through a capability-checked control endpoint. Check `/api/v1/info` for existing station identity, then complete the mission handshake before enabling start.

Launch Telem/Video as separate instances with distinct directories, ports and boot identities. The normal four Away sites remain `gs1`–`gs4`; receiver-only means a site with its pointer disabled. Do not create a fifth wire station ID. Additional receivers/channels within a site need declared endpoint IDs and capability support. Base remains `launch`, has no local pointer and alone owns simulated uplink. A `.rktmission` replay cannot open these endpoints.

The current `RTFC.readTelem()` supports the following command families; advertise the actual runtime's capability set, not the longer physical protocol wish list:

| Opcode, hexadecimal | Current simulator behavior to retain |
| --- | --- |
| `01`, `06` | Arm/fire selected channels, with existing mask, connection and pyro-state checks. |
| `02` | Request an existing FC state; retain the FC's state-transition rules. |
| `03`, `05` | Manual airbrake/roll angles, ground state only; finite −180° through +180° values. |
| `04`, `08`–`0B` | Ground-only roll control, controller reset and sensor-zero operations as implemented. |
| `10`–`13` | Existing emergency channel groups; `13` excludes channel 5. |
| `14`–`16` | Ground-only VTX/converter/protection settings with the current argument handling. |
| `07` | **Not implemented in this FC simulator**, although the telemetry protocol document lists disarm. Hide/disable it for this runtime; do not invent a successful response. |

The FC consumes at most one received packet at the end of each loop. Record queued, delivered, parsed, ignored, rejected and applied outcomes separately, deriving FC outcomes from simulator diagnostics. Those diagnostics are not a wire acknowledgment. Unknown opcodes, reserved bytes and state-gated commands retain their current behavior. Adding any missing opcode is a separate explicit implementation change with a source-grounded fixture.

## 8. Video, pause and recording

Logical streams describe onboard sources; receiver copies describe reception. Default Balius is one digital plus one analog. Iris is Sustainer digital to GS1–4, Sustainer analog to GS1–3 and Booster analog to GS4; GS4 digital remains Sustainer. Zero streams of either type is valid. Arbitrary supported counts require a generalized registry and resource admission, not a hardcoded two-channel UI.

Generate one image asset per logical stream/run/member using a bundled font and template version. Include the requested mission title, run/member ID, `.ork` basename, vehicle and prominent Analog/Digital ordinal. Save the PNG/JPEG bytes, hash and metadata, so replay need not reproduce platform-dependent font rendering. No real camera is required.

In accelerated/headless mode, record logical frame generation/delivery/drop/freeze events against mission time and the slate asset. No H.264/SRT encoding is required. Full media-path qualification runs paced through the existing source/encoder/SRT/Base-wall chain. Encoded bytes and host delivery timing are not bitwise deterministic; do not compare them as canonical simulation output.

Distinguish **content freeze** (unchanged image with fresh frames) from **delivery outage** (no new frames, hence stale). Ordinary static slates always have unchanged pixels, so a content freeze alone cannot demonstrate lost reception. Default stale threshold is the UI's five seconds; outage fixtures must stop delivery longer than that and test recovery. Analog noise is a labeled illustrative pixel effect with seed/version, not a waveform or codec model.

On mission pause, freeze logical frame time and mission freshness. The transport may repeat a paused slate to maintain an SRT session; mark those as carrier/paused frames, excluded from mission delivery and freshness metrics. Keep lease renewals on host time (currently 30-second lease, 10-second renewal). If the receiver cannot advertise paused-clock support, disable pause for that connected run or disconnect it explicitly; never silently test wall-clock stale behavior as though it were mission pause.

Allocate the current **two preview and two main sessions per publisher total**, not per stream. Validate available ports and actual encoder capacity, report refused promotions and retain the previous usable preview. Include received-frame timing/status and `capture-timing.jsonl` for local capture and remote wall views when recording them. Shared link-outage IDs can correlate telemetry and video explicitly; otherwise streams have independent impairments. Save encoded recordings only by request or for media qualification.

## 9. Configuration files, service and immutable runs

Use production JSON `format: openrocket.mission`, `version: 1`, extension `.rktmission-config.json`. Use **`.rktmission-setup`** for a portable configuration ZIP with manifest, mission JSON and content-addressed assets; it contains no result history. Reserve **`.rktmission`** for results. The preview's similarly suffixed JSON retains its distinct `openrocket.mission-configuration-preview` identity; a dedicated import adapter reports placeholder/missing fields and cannot turn it into a runnable configuration by changing its format string.

Asset precedence is bundled hash-verified content → explicit resolved path → original relative reference. Mismatched hashes require selecting a new draft revision; never silently replace frozen assets. Executable/interpreter locators are machine preferences, not executable content to auto-launch from an imported bundle. Preserve unknown optional data on save; unknown required capabilities or major versions block runs. Unknown optional settings are not considered implemented merely because they round-trip.

The local Java HTTP API uses `/mission-api/v1/`: `capabilities`, `drafts` (create/load/import), `drafts/{id}` (read/update), `drafts/{id}/save`, `drafts/{id}/resolve`, `drafts/{id}/validate`, `jobs` (start), and `jobs/{id}/{pause,resume,cancel,events,results}`. Job status is `GET jobs/{id}`. State-changing requests carry the launch token and allowed origin; draft mutations require the expected revision. Files are selected through Java-owned dialogs/asset handles. Use no arbitrary shell-command endpoint.

Validation returns field path, stable code, severity, explanation and the required correction. Start requires a validated resolved snapshot hash and an idempotency key; retries return the original job. Validation changes (missing assets, changed file hashes, occupied bindings, unavailable decoder/runtime) invalidate readiness. Pollable job status and sequenced bounded events survive a browser reconnect without restarting execution.

The state machine in specification 16 applies. Finalization records participant outcomes and either creates a complete/partial bundle or preserves staging with a visible error. An ensemble's member failure does not erase successful members: default is continue independent members, with parent outcome `complete-with-failures`; explicit cancel stops scheduling new members and cancels active ones. A user-selected fail-fast policy is recorded in configuration. A runner restart cannot claim to resume a custom FC mid-flight; recover readable committed results and rerun a member from startup under a new run ID.

## 10. Bundle layout and durability

Use ZIP64 schema 1 with the layout below. IDs in paths are validated opaque identifiers, never user titles or raw external paths. The same layout serves standard and custom-FC sources.

```text
manifest.json                              # schema, inventory, statuses
mission.json                               # immutable resolved configuration
assets/<sha256>/<basename>                  # rocket/FC/motor/codec/slate/geoid inputs
runs/<run-id>/run.json                      # member/layout/seed/model/clock identities
runs/<run-id>/truth/{schema.json,chunk-*.csv}
runs/<run-id>/events/chunk-*.jsonl
runs/<run-id>/transmitters/<id>/tx/chunk-*.bin
runs/<run-id>/transmitters/<id>/tx/index-*.jsonl
runs/<run-id>/fc/<id>/...                    # original FC logs, if present
runs/<run-id>/stations/<id>/receiver/...
runs/<run-id>/stations/<id>/pointer/...
runs/<run-id>/stations/<id>/network/...
runs/<run-id>/stations/<id>/sessions/<role>/<instance-id>/...
runs/<run-id>/video/<stream-id>/source.json  # asset hash, cadence, codec/profile
runs/<run-id>/video/<stream-id>/events-*.jsonl
runs/<run-id>/{metrics.json,validation.json}
index.sqlite                               # finalized, rebuildable lookup
```

TX indexes contain transmitter/packet ID, mission/boot time, length, byte offset and payload hash. Variable custom payloads are not assumed to be 128-byte records. Receiver outcomes reference the original TX plus trailer/corruption data, while protocol-test raw captures preserve actual byte chunks. Video events include receiver identity. Namespaced UI sessions avoid collisions between Telem/Video and restarted processes; preserve original native host/UTC clocks and add sidecar mappings rather than rewriting `RGM1` headers.

Resolve the existing `virtual_pointer_tx` recorder mismatch by recording its exact transmitted command bytes under the existing pointer-TX raw role 3, with simulation provenance in session metadata. Continuous modeled pose belongs in the mission pointer channel, not a fabricated physical pointer-RX record. Keep the native raw-role numbering and `.rktflight` schema unchanged.

Rotate canonical chunks at 16 MiB uncompressed, and checkpoint at least once per active host second; preserve record boundaries. Large runs may contain multiple files for one second. Chunks have first/last mission time, sequence range, schema and committed byte length; readers do not scan every member to find a selected time window. The limit is a starting storage default, recorded and benchmarked, not a promise about compression ratio. Use Deflate level 3 initially and ZIP store for already compressed media/archives.

Write/flush raw records before committing references, fsync at checkpoints, and atomically replace checkpoint manifests on the same filesystem. Snapshot SQLite using its backup API. A crash exposes only committed prefixes and an incomplete status; the most recent uncommitted interval may be lost and is identified. Finalize into a temporary ZIP beside the destination, verify inventory, then atomically replace it. Failure leaves staging and any previous valid output intact. Do not clean recoverable staging automatically before the user has a valid result or explicitly discards it.

The inventory lists every entry except its own manifest, with path, length, SHA-256 and type. Reject duplicate/escaping paths, symlinks, unexpected entries, unsupported schema and inconsistent indexes using bounded reads. Initial import ceilings match the existing `.rktflight` reader: 100,000 entries and 64 GiB uncompressed, reported before extraction. Larger ensembles require explicitly raised resource policy or partitioned bundles; never silently omit members. Check free space for staging plus final ZIP; disk-full is a failed/incomplete run, not a successful run with dropped logs.

Keep `.rktflight` schema 1 export on demand for one selected station session, with a report of omitted mission channels. Nested compatibility exports are optional duplicates, not the default layout. Reopening a result does not require original machine paths or execute its FC/behavior assets.

## 11. Metrics and reproducibility claims

Record the numerator, denominator, interval and profile for every metric. Eligible packet count excludes untuned/not-yet-active receiver intervals but includes in-range scheduled TX attempts lost to modeled impairments. Byte-accepted reception rate uses decoder-accepted frames divided by eligible transmissions, with duplicates reported separately. Report routing exclusions and raw RF success separately from valid decoded reception.

Information age is mission time minus the acquisition time of the newest usable target observation. Wire uptime is not a substitute for sensor acquisition time; source metadata supplies it in simulation. Outage duration is time without a usable accepted update, with the freshness threshold stated. Tracking error uses the actual plant boresight and truth LOS only in analysis; report undefined geometry separately. First acquisition requires usable telemetry and the configured tracking/beam criterion, not merely an encoder write. Saturation time and commands admitted/expired/uncertain are distinct metrics.

For identical versions, inputs and input tape, deterministic mode must reproduce canonical event ordering, wire bytes and sampled model outcomes independently of worker count. Freeze floating-point tolerances for trajectory/angle comparisons; do not promise identical floating-point values across all platforms/JDKs. Exclude UTC, IPC chunking not explicitly modeled, ZIP metadata, video encoder output and GUI repaint timing from bytewise claims. Record code/runtime/codec/model versions and the baseline OpenRocket/FC seeds alongside new mission streams.

## 12. Implementation gates

Packages A–G in specification 16 are the one delivery sequence. Add the following fixtures to their acceptance checklist; these are **required future tests**, not tests performed by this document review.

| Gate | Fixture and passing evidence |
| --- | --- |
| A — contracts | Shared JSON schemas; independent legacy 128/144/11/16-byte vectors; custom-codec bounds/checksum vectors; FC v1/v2/v3 migration; all six URRG coordinates; MGRS point/center preservation; geoid/relative-height conversions; seed derivation vectors. |
| B — complete configuration | Inventory-driven round trips of every flight/FC/ensemble field; missing assets/heights remain draftable; preview migration flags placeholders; immutable run hash; idempotent start; offline bundled UI and worker lookup. |
| C — first headless mission | Standard telemetry, Base plus four Away stations; one selected pointer controller per station; no truth leak; independent receiver loss; stale stop/restart; ZERO without physical jump; two-branch Iris replay and explicit custom-FC multistage rejection. |
| D — FC/application | Actual custom-FC single-stage flight through the normal decoder/policy/recording path; FC/PWM same-time ties, warmup, bounded tail, supported uplink acceptance/rejection and unavailable `07`; unknown/relative altitude; reordering without false reboot; queue pressure, pause and reconnect. |
| E — video | Balius/Iris plus Digital 2; GS4 routing; zero-video mission; legible slates; capacity rejection; paused clock versus leases; delivery outage longer than five seconds; received-video timing and optional encoded capture. |
| F — durable results | Serial/parallel member agreement; failed member plus completed siblings; standard source without `fc/`; pointer-TX role regression and continuous pose; multiple role/restart sessions; lazy reads; kill during writes/checkpoints/finalization; disk-full; malicious/truncated ZIP; both applications reopen complete and recovered prefixes. |
| G — packaged acceptance | macOS, Windows and the available Linux distribution run the same standard and custom-FC fixtures with bundled worker, no developer environment and no physical devices. Qualify software IPC on each platform; PTY/virtual COM are separate optional checks. |

Do not begin package C with undocumented stand-ins for the Python policy, FC timing or archive contract. Package A materializes these decisions as schemas and shared fixtures; package B supplies the complete editable configuration. Storage is integrated as C–E add streams, not deferred until the end. Once these gates pass, implementation can be described as delivered; measured RF/mount fidelity still requires the corresponding real-world profile data.

## 13. Source evidence for the final decisions

This review inspected the following additional boundaries directly, beyond the references in documents 15 and 16:

- [FC startup guard and sensor adapter](../../clone/openrocket/core/src/main/java/info/openrocket/core/simulation/listeners/FlightControllerSimulatorListener.java), [sequential branch engine](../../clone/openrocket/core/src/main/java/info/openrocket/core/simulation/BasicEventSimulationEngine.java), [FC format version validation](../../clone/openrocket/core/src/main/java/info/openrocket/core/simulation/flightcomputer/FlightComputerDesign.java).
- [FC GPS zero-reference behavior](../../clone/openrocket/core/src/main/java/edu/mit/rocket_team/instrument/RTGPS.java), [legacy packet construction and transmit timing](../../clone/openrocket/core/src/main/java/edu/mit/rocket_team/FC/RTFC.java), [queued delivery and finalization](../../clone/openrocket/core/src/main/java/edu/mit/rocket_team/telemetry/RTTelemetryEngine.java).
- [Decoder reboot/rollover heuristic and LIVE label](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/protocol.py), [tracking/freshness](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/controller.py), [pointer model](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/virtual_pointer.py), [archive limits](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/flight.py).

No simulator, telemetry transport or media integration was implemented during this review. The next step is package A: turn the frozen decisions into executable schemas, migrations and independent fixtures, then proceed through the configuration and first headless mission.
