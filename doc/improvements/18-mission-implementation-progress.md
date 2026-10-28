# Mission Simulation implementation progress

28 October 2026. Implementation follows the A–G packages in [the configuration specification](16-mission-configuration-ui.md) and the [final contracts](17-mission-simulation-contracts.md). Each tested increment is committed and pushed separately. This ledger distinguishes working code from remaining package work.

## A1 — shared contract foundation

Implemented in OpenRocket core and RT Telemetry UI:

- Versioned mission draft envelope and `rt-mission.v1` message schema, bundled identically in both applications' source packages with SHA-256 inventory verification. The OpenRocket resource directory is authoritative; Telemetry UI has an explicit sync/check tool.
- Exact URRG default coordinates, original MGRS strings and four pointer assignments. Elevations remain unresolved. Creating a draft assigns a fresh mission UUID; loading a saved draft preserves its identity and values.
- Java immutable draft objects and Python draft read/write functions. Both validate the shared schema, route references, unique IDs, pointer assignments, station command ownership, seed bounds and delivery-tail bounds. Unknown optional data round-trips. Atomic saves preserve the previous file on validation/write failure.
- Bounded JSON parsing: 1 MiB, depth 64, duplicate-key rejection, finite numbers and strict UTF-8. Preview configurations and unknown production versions are rejected instead of being silently upgraded.
- Identical event-addressed SHA-256 sampling, with shared independent hash/uniform/normal vectors. No global random stream or dependency on worker count.
- Explicit geoid-separation height conversion and WGS84 ECEF primitives with analytic vectors. This is not an installed geoid grid, terrain-height lookup or a Java MGRS decoder.
- Bounded length-prefixed IPC frame read/write, signed 64-bit mission times represented as strings, exact byte chunks, canonical base64, fragmented/coalesced reads and truncated-frame rejection. This opens no sockets, serial devices or applications.
- Legacy 128/144/11/16-byte fixtures. Java preserves exact payloads/trailers, including deliberately corrupted packets; Python exercises the actual decoder, pointer encoder and uplink encoder against the stored vectors.

The schemas intentionally validate the **draft envelope** at this increment. Full flight snapshots, custom codecs, link models and output policies are retained objects but are not yet interpreted as executable settings. No API claims that an accepted draft is ready to run. The validator implements only the published bundled keyword subset and fails on unknown schema keywords; it never resolves external schema URLs.

### Verification

The first check set passed 49 Java tests (27 new mission checks plus existing FC-design and ensemble regressions) and 58 Python tests (mission contracts plus existing protocol/location checks). Python lint and byte-for-byte synchronization of all ten shared contract resources pass. The tests exercise both actual implementations against stored independent vectors, rather than comparing one encoder with only its own decoder.

The repository-wide Java style task reports existing non-ASCII warnings in other source/test files. The new mission source/tests have no Checkstyle findings; this increment does not modify the unrelated files to hide those warnings. Test dependencies missing from the local cache were fetched through the normal Gradle resolver, after which the targeted checks ran successfully.

### Locations and commands

Authoritative resources: `clone/openrocket/core/src/main/resources/mission/v1/`.

Java APIs: `info.openrocket.core.simulation.mission` (`MissionDefinition`, `MissionJson`, `MissionSchema`, `MissionRandom`, `MissionGeodesy`, `MissionWire`, `LegacyTelemetryEnvelope`).

Python APIs: `rocket_gnc_monitor.mission_contracts`. Importing this package needs only the Python standard library and starts no Qt application. Existing Telemetry UI controllers and LIVE transport paths are unchanged.

From the OpenRocket source directory:

```sh
./gradlew :core:test --tests 'info.openrocket.core.simulation.mission.*' \
  --tests 'info.openrocket.core.simulation.flightcomputer.FlightComputerDesignTest' \
  --tests 'info.openrocket.core.simulation.ensemble.EnsembleTest'
```

From the Telemetry UI repository:

```sh
.venv/bin/python -m pytest -q tests/test_mission_contracts.py tests/test_protocol.py tests/test_location.py
.venv/bin/python scripts/sync_mission_contracts.py --check \
  ../ActiveControl_MIT_RktTeam/clone/openrocket/core/src/main/resources/mission/v1
```

The sync tool requires an explicit source directory and verifies the source inventory before copying. Run without `--check` only when deliberately updating the reviewed shared contract bundle. Recompute the authoritative inventory when a contract changes; never regenerate expected vectors automatically as part of a runtime test.

## A2 — immutable flight inputs, codecs and coordinates

Implemented complete simulation-option companions, native rocket/extension snapshots, frozen motor curves and exact FC file copies with an inventory. Restoring a snapshot uses its own motor data and FC files even after the original library file is deleted. All flight, inactive wind, manual bearing, FC extension and ensemble settings remain distinct. Loading does not run an FC; existing Java-board approval checks still apply when execution begins.

The FC reader now accepts v3 with an optional declarative telemetry format. The Java/Python codecs share a bounded schema, fixed numeric layouts, explicit scaling, optional-field availability and checksum parameters. Independent byte-layout, sum8 and published CRC check-string fixtures pass. A custom-format design is explicitly rejected by the legacy standalone runner until Mission Simulation supplies its receiver path; existing v1/v2 behavior remains available. A shared command capability declaration excludes unsupported opcode `07`.

Production MGRS conversion uses NGA MGRS Java 2.1.3, with explicit grid-point/cell-center conventions and unchanged-notation preservation. All six URRG defaults are retained exactly; fresh conversions agree with the independent Telemetry UI library within 3 cm. UTM MGRS is supported; polar locations use latitude/longitude. The offline, hash-pinned GeographicLib EGM96 15-minute grid uses bilinear interpolation; its published interpolation error is 1.152 m maximum / 0.040 m RMS, not surveyed ground accuracy. Seven independent PyGeodesy/Karney values cover URRG, poles, the longitude seam and mountains. Terrain/station heights remain user inputs.

Preview import preserves all original fields and exact stored coordinates, with field-level review findings for placeholder settings and machine paths. It cannot be made runnable just by changing its format identifier. The service in B will enforce those findings.

Verification: 77 Java mission/FC/controller/ensemble checks passed before the additional full-flight snapshot comparison; the six snapshot checks then passed, including ordinary-flight solver samples matching within `1e-5` in SI units (native XML numeric serialization tolerance). Python passed 69 contract, codec, protocol and coordinate checks, plus lint. Fourteen shared contract files are identical between repositories. Packaged execution remains package G work.

## B1 — configuration service and portable input storage

Implemented a Java-owned, token-authenticated loopback API, revision-checked draft updates, typed validation findings, immutable resolution hashes and idempotent job dispatch. Job execution is supplied by the runner interface; without a runner the API explicitly advertises that execution is unavailable. Native file selection is an injected desktop operation, not an arbitrary browser path or shell endpoint.

The new flight editor model covers every option companion field plus active stages, separation events/delays/altitudes, motor selection and ignition/ejection settings, recovery events/delays/altitudes, and all installed extension configuration values. Editing works on isolated snapshots. Its tests uncovered two native serialization omissions (inactive top-stage separation values and the ignition override flag); the snapshot companion now preserves these too. `MotorConfiguration.useDefaultIgnition()` also now clears its override flag after the setters that mark it overridden.

Portable `.rktmission-setup` export/import carries the draft and frozen native inputs. A fixture deletes the original machine's entire input store and restores the setup on a fresh store. Missing elevations still produce draft findings. Loading retains the mission identity and advances an already-open draft's local revision to invalidate stale edits.

The shared Java archive foundation writes ZIP64 with SHA-256 inventory, Deflate level 3, verified temporary publication and atomic replacement. Reads verify selected entries lazily; import rejects escaping paths, Windows device names, case collisions, symlinks, truncation, inconsistent inventories and oversized contents. These storage primitives are not yet a running result recorder, crash-recovery implementation or replay UI; those remain C–F work.

Verification: 67 Java mission and motor tests passed, including loopback authentication/origin rejection, immutable resolution, repeated-start idempotency, full editor round trips, portable recovery and corrupt-archive rejection. The browser workspace and desktop entry point are B2, next.

## B2 — connected configuration workspace

The File menu now opens **New Mission Simulation** in a browser, backed by the Java service. All six pages are bundled in the application, including Leaflet 1.9.4 (publisher hashes verified, license included). No Node.js or CDN script is required by users. USGS imagery/topographic labels, Esri imagery, OSM streets and a coordinate-only offline view are selectable; imagery failure leaves editing available. Visible attribution and browser caching are retained.

The workspace supports exact saved-MGRS reapply, fresh grid-point conversion, map dragging/placement, explicit ground/mount heights, pointer count and receiver-only sites; native rocket/simulation selection; flight, motor, stage, recovery, extension and ensemble editors; per-receiver impairments and outage windows; standard/custom FC selection; Balius/Iris routes, independent digital/analog counts and labeled slate previews; station command ownership and machine-local application selectors; JSON save/load and portable setup export/import. Choosing a source project or executable records a machine preference, outside portable mission files.

Validation and input resolution are connected. Run controls remain disabled when the runner is unavailable; no simulated progress or dummy completion is shown. C supplies the actual runner and result views. Existing extension configuration values are editable in the advanced section; specialized FC assignment controls can be improved as D wires those paths.

Verification: 49 core mission tests pass, and the application JAR builds. Browser checks against the actual Java server passed with all internet requests blocked: six sites, unchanged MGRS, height editing, flight/link changes, Iris GS4 routing, Digital 2, validation, save/load and identity preservation. All six pages fit a 390-pixel viewport in dark mode. Site and video screens were visually inspected. The repeatable host is `MissionWorkspacePreview` in test sources; the browser check is `packaging/verification/mission-workspace-browser.cjs` and requires developer Playwright only for testing.

## C1 — shared station worker and illustrative link/mount models

The Qt-free Python station uses the actual Telemetry UI decoder and a shared receiver-only tracking policy. The desktop controller now calls that policy for its legacy pointing calculation. The worker receives serial bytes plus explicit acquisition/height-reference metadata; it has no trajectory access. It supports bounded custom-codec framing, stale hold with explicit restart, reordered observations without false reboot or target refresh, station connection epochs, exact pointer command bytes and declared command expiry.

OpenRocket bundles a reproducible source ZIP of that worker, verifies its SHA-256, and launches one isolated Python process per member over authenticated loopback IPC. Credentials use a private launch manifest deleted after handshake. The production Python runtime locator is separate from imported setup data; installer distribution remains G work. A developer override is explicit and recorded. Lockstep calls verify identities, sequences, mission time and acknowledgments with a ten-second host watchdog.

Java link models implement routing/outage/directional qualification, event-addressed independent loss/corruption/jitter, FIFO or explicit reordering, and optional 8N1 UART pacing. The illustrative mount uses physical slew limits and encoder reference offsets: ZERO does not teleport the mount, and loss of new commands does not fabricate a firmware STOP.

Verification: 24 Python station/controller/protocol tests and three Java worker/link/mount tests passed. The Java test launches the bundled Python ZIP and exchanges actual legacy packets and pointer bytes over a socket. Complete flights, recording and job controls are C2 work, next.

## C2 — complete standard flights, station replay and committed result streams

The Java runner now computes the complete ordinary flight before station playback, identifies branches by stage UUID, preserves accepted solver samples/events and uses ECEF/continuous-value interpolation with quaternion slerp. Attached vehicles use the parent trajectory until separation, then their own stage trajectory. Explicit body offsets and active intervals are supported by the binding model. Invalid/missing multistage bindings are findings; a single active stage can be resolved automatically.

The standard source acquires held GPS/barometric observations at independent cadences, uses event-addressed noise, sends exact legacy packets, records acquisition/height-reference metadata and marks unmodeled hardware unavailable. Base plus four Away receivers run independently through the shared worker. The mission clock includes pad warmup, plant advance, transmissions, ordered link deliveries, policy, pointer commands and a bounded delivery tail. Large same-time delivery batches are drained without evaluating policy before the final batch. Truth is used for analysis/beam qualification, never as a policy input.

Canonical recording now rotates raw/index/event/truth chunks at 16 MiB, forces raw data before index checkpoints, and publishes committed-prefix manifests at least once per active host second. Results carry their resolved configuration and frozen flight assets; failed/cancelled jobs preserve a verified partial bundle or readable staging. The desktop runner supports pause/resume/cancel, persisted status and browser reconnection. The browser retains an uncertain start's idempotency key and shows receiver counters/results instead of raw job JSON.

Verification: 54 mission Java tests passed, followed by four full-run/branch/lifecycle tests. The full mission uses the packaged Python worker, receives at five stations, completely drops only GS2, generates no Base pointer commands and reopens its verified ZIP. A real multistage rocket retains shared pre-separation positions and divergent separated trajectories. A paced job freezes mission time while paused and cancellation produces a hash-verified partial result. Python's 24 station/controller/protocol regressions pass.

This increment admits standard headless missions with zero video and one flight member. Other configured modes are rejected explicitly at start pending D–F; the workspace retains their settings. SQLite/recovery/replay views and broader crash/packaged qualification remain F–G work.

## D1 — coupled flight computer, declared packets and command tape

The existing FC listener now accepts a per-run mission adapter at accepted scheduler boundaries. Mission deadlines participate in physics step limiting; PWM/controller/CPU ordering remains in the existing listener. The same station/link recorder handles custom-FC transmissions, pad warmup and the bounded post-flight delivery tail. Mission runs use an explicit telemetry finalization bound and never drain at `Long.MAX_VALUE`.

Declared codecs encode held measured FC device values. Missing fields remain unavailable at the receiver; a test codec lacking longitude/fix receives successfully but cannot track, confirming that truth does not fill those gaps. GPS reference changes carry their actual baseline and epoch. Variable-length packets exposed the legacy flash logger's fixed 128-byte assumption; custom designs now use a separately identified, bounded illustrative byte-stream page model, while the legacy logger remains unchanged. Native FC diagnostics are retained alongside canonical variable-length TX/index records; native CSV metadata explicitly identifies the external mission transport.

Timed station actions and live pointer controls support connect/disconnect, explicit tracking restart, hold, ZERO, manual pointing and target changes. Base-only uplink supports the actual implemented opcodes, with `07` unavailable in the normal command selector. Explicit raw-byte experiments can still test rejected/unknown packets. Commands pass through a separately modeled uplink and enter the FC queue for its next normal one-packet loop read. Command identities and parsed/ignored diagnostics are recorded as simulator evidence, not wire acknowledgments. Queue admission is bounded; switching targets clears partial framing and old target observations, advances the connection epoch and requires tracking restart.

Verification: 72 Java mission/FC timing/output/telemetry/flight checks passed, including a full coupled v3 flight with 14-byte payloads, GPS-zero command and unsupported raw opcode. Python passed 25 station/controller/protocol checks, including reconnect during a fragmented frame. The application JAR builds. The resolved asset inventory now follows the final frozen flight hash and branch records include parent lineage.

D2 still needs the actual Telemetry UI process/manifest adapter and application handshake. Video, ensembles, full recovery/replay and installer qualification remain E–G. The build continues to reject those unfinished execution paths rather than silently skipping configured features.

## Remaining work

**C–G:** deterministic station worker/scheduler and actual simulation, FC/software-station integration, media transmission, ensembles/bundles and packaged qualification remain as specified. No installer or GitHub release should describe Mission Simulation as delivered on the basis of A1 alone.

## D2 — real Telemetry UI participants (2026-10-28)

Implemented the `--mission-manifest` entry point using the actual StationWindow,
with an injected MissionController and explicit SIMULATION source. It cannot open
physical serial connections or switch into LIVE. Receiver bytes, decoded samples,
and pointer writes use the same MissionStation policy as the bundled worker.
Qt acknowledges every boundary before the producer continues; failures terminate
the participant rather than silently losing canonical data. Interactive owners
replace the corresponding headless command owner; observers cannot issue commands.
Base uplinks return through the ordered simulator queue; opcode 0x07 remains disabled.

Each role gets a private data directory, connection identity, OS-assigned loopback
LAN port, simulation-only network acceptance, a native session, and a mission/host
clock mapping. The result bundle imports closed native sessions separately by
station/role/instance. Credential manifests are deleted after handshake and excluded
from results. Source-project/Python or packaged-app paths come only from machine
preferences. Interactive job admission requires paced execution.

Validation: 57 Java mission tests passed; a further Java integration test launched
the real Telemetry UI, ran a complete flight with GS1 owning the pointer, and verified
its archived native SQLite session and clock mapping. The Python application test
verified handshake, software receipt, tracking writes, fixed mission time during an
idle boundary and complete session shutdown; 44 station/controller/protocol/network
observation regression tests passed. No serial devices were opened.

Remaining work includes mission video, ensemble execution/result indexing and crash
recovery, runtime packaging and platform qualification. Existing-process attachment
and paused ownership transfer still need their integration gates before final completion.

## E — logical video and real station video transport (2026-10-28)

Added deterministic per-stream, per-member 960×540 PNG slates with mission title,
run/member, vehicle, stream ordinal and rocket filename. The bundled DejaVu font,
license, template version and hashes make the asset provenance explicit. Logical
frame capture, delay, independent loss, outages and delivery are recorded separately;
one static image is stored per stream. Analog interference is an explicitly
illustrative deterministic static pattern, not an RF demodulation model.

The Telemetry UI MissionProfile now supports arbitrary stream IDs and zero inputs
without changing normal Balius/Iris profiles. Mission capture feeds the existing
MediaService → H.264 Baseline/YUV420P MPEG-TS → leased native SRT → Base VideoWall
path. Mission publishers and viewers accept SIMULATION explicitly. Missing logical
frames stop the carrier; the existing five-second watchdog detects the outage.
Pause carriers keep host-time leases alive and are labeled/excluded in the timing
sidecar. Native capture and remote wall receipt mappings are retained with sessions.

Connected video requires paced execution. A publisher supports two preview and two
main sessions total; preflight rejects connected routes requiring more than two
simultaneously displayed channels at a publisher. Headless logical video supports
up to 64 total streams independently of that real-time encoder capacity.

Validation: actual native SRT delivery to a Base wall, a >5-second outage and recovery
passed using the Mission capture implementation. Zero/arbitrary registry and pause
carrier checks passed. Existing video-wall/media regression checks passed after a
compatibility fix for the test runtime's optional source-mode property. Java tests
verify logical outage/recovery, two distinct slate hashes, dimensions, and a reopened
archive containing only one PNG per stream. Full Java mission regression is recorded
with the stage commit. A visual review caught and removed a residual LIVE label on
Mission wall tiles. Receiver RNG addressing now includes transmitter identity, and
same-time delivery ordering includes it as well.

## F1 — ensembles and receiver analysis (2026-10-28)

Mission ensembles share the native EnsembleSampler and EnsembleRunListener, retaining
the existing atmosphere, wind, thrust, mass and aerodynamic coefficient distributions
and historical draw order. Each member owns its flight, FC, receiver process, links,
video timeline and output directory. Work is bounded to eight members and the host's
available processors; paced application ensembles run serially. Independent member
failures are recorded while other members continue, unless fail-fast is selected.
Cancellation waits for member workers to stop. Paced clocks restart for each member.

Receiver metrics now separately record RF/frame counts, freshness coverage, acquisition
age, first usable acquisition, outage duration, tracking time, saturation time, and
time-weighted pointing error with explicit observation intervals and denominators.
Cancelled/failed coupled runs retain the accepted truth prefix and native FC files.

Validation: all 26 native ensemble and Mission ensemble tests passed. The new parity
test compares every canonical per-member stream from serial and parallel execution
byte for byte. A separate analytic test verifies information-age and freshness
normalization. Result indexing, committed-prefix recovery and replay/export interfaces
remain the next increment.

## F2 — indexed results, recovery and station export (2026-10-28)

Canonical checkpoints now include incrementally computed SHA-256 prefix hashes and
support bounded 32 MiB inventories. ZIP/ZIP64 central-directory bounds are inspected
before entry allocation; non-regular Unix entries are rejected. Recovery copies and
verifies only committed file prefixes, builds a fresh SQLite index, publishes an
incomplete bundle atomically and leaves the original staging untouched.

The bundled stdlib utility creates `index.sqlite` through SQLite backup, checks raw
index bounds and per-record hashes, and supports bounded member/channel/time-window
queries. The workspace can open a result, recover a staging folder, inspect an event
or measurement channel, and plot the selected time window without running endpoints.
Ensemble status summaries and active/queued jobs have explicit bounds.

Selected stations export to existing `.rktflight` schema 1 with an omission report.
A selected native application session preserves its original clocks and uses SQLite
backup. A headless station export reconstructs receiver samples and existing RGM1
raw roles, explicitly labels the synthetic elapsed-time mapping and unavailable UTC,
and never invents physical pointer feedback or hardware acknowledgments.

Validation: 63 Java mission tests passed and the distribution JAR built. Recovery
tests reject altered committed bytes, ignore uncommitted tails, preserve the source,
and reopen/query an archive after deleting its original staging. Cancellation still
produces an indexed readable prefix. The exported station fixture opens successfully
with the unmodified Telemetry UI flight reader.

## G1 — bundled runtime and package qualification (2026-10-28)

The macOS and Windows packaging recipes now include hash-pinned Python 3.13.18
runtimes and their upstream license files. Runtime discovery works beside the
distribution JAR without a developer interpreter. A common application-image
qualification script and CI matrix cover Linux x86_64, Windows x86_64, and both
macOS architectures. The probe uses the image's Java and Python, exercises standard
and custom-FC missions, then verifies slates, archive hashes and indexed queries.

The native macOS ARM image passed: 61 standard frames and 50 custom-FC frames
accepted at GS1. Other platforms require the CI evidence; adding a matrix is not
itself a passing platform qualification. The final integration increment is still
closing attachment, ownership transfer and recording controls.
