# Mission Simulation implementation progress

28 October 2026. Implementation follows the A–G packages in [the configuration specification](16-mission-configuration-ui.md) and the [final contracts](17-mission-simulation-contracts.md). Tested increments are committed and pushed in batches. This ledger distinguishes working code from remaining package work.

**Current delivery:** configuration, standard/custom-FC execution, ground pointers,
Telemetry UI connections, video, ensembles, portable results, recovery and saved-flight
experiments are implemented. Use [the user guide](19-mission-simulation-user-guide.md)
for current behavior and limits. Earlier sections below are dated checkpoints;
their unfinished-work statements are superseded by the later acceptance entries.

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

## Work remaining at the D1 checkpoint (historical)

At this checkpoint, **C–G** still required deterministic station execution, FC/software-station integration, media transmission, ensembles/bundles and packaged qualification. The later entries below record their implementation and acceptance; this is not the current outstanding-work list.

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

## D3 / E2 / F3 — integration closure (2026-10-28)

An already-open Telemetry UI can explicitly prepare a private Mission connection.
OpenRocket selects its connection file per role, checks authenticated loopback
readiness and matching contract hashes, then opens a separate simulation workspace
inside that process. Completing a mission closes its session without terminating
the existing application. Fresh launches also have a capability check.

Paused ownership transfer revokes both controllers before granting one, advances
ownership/receiver epochs, clears pending operator actions and requires an explicit
tracking restart. The browser exposes the actual owner. Paused commands are rejected;
active commands carry a 500 ms mission-time expiry. Native session closure completes
before its acknowledgment, and participant/recording shutdown errors fail the result.

Requested encoded video now records delivered frames into native session segments,
with bounded queues and surfaced errors. Pause carriers are excluded. Video outage
editing, queue admission, disabled-site filtering and portable result filenames are
effective. Ensembles connect applications only to the selected member; other members
use recorded headless ownership overrides. Results verify index channel/time/record
boundaries, and receiver outage intervals end at reacquisition rather than the next
policy tick.

Validation: 64 Java mission tests passed, including fresh and existing-process GUI
connections and two ownership transfers. Eleven Python station/application/result
tests passed; native SRT delivery, outage recovery and optional encoded recording
passed in the video tests. The selected application and pinned worker now exchange
the exact shared-contract inventory hash. Windows checkout byte preservation and
explicit SQLite closure address the next platform failures. All other matrix
platforms passed the preceding qualification revision; final Windows and updated
revision qualification remain pending.

## G2 — recovery and acceptance closure (2026-10-28)

The four native package-image jobs all passed at `08e3a988` (Actions run
40746926431): Windows x86_64, Linux x86_64, macOS ARM and macOS Intel.
Windows required writable handles for durable file flushes; that fix is in the
pinned worker. This verifies bundled Java/Python execution and both standard and
custom-FC probes, rather than merely compiling platform packages.

Mandatory application loss now pauses the mission for an explicit reconnect,
creates a distinct restart session and preserves the prior session. Commands with
uncertain outcomes are recorded and never retried. Tracking needs an explicit
restart after reconnection. Read-only observer loss does not interrupt the member.
Application readiness, selected ensemble member bounds and unique scenario action
IDs are checked before execution. Receiver backlogs have explicit memory/count
limits; an over-capacity member fails with its committed data preserved.

The configuration adds Base video and independent receiver video impairment
profiles. Native station sessions receive the configured mission/site metadata.
Mission controllers reject physical camera/serial, replay and unrelated virtual
trajectory starts. Browser acceptance exercises offline map operation, edited
flight/link/video settings, save/load, a paced run, pause/browser reconnect/resume,
result naming, indexed plots and a compatible station export.

Separate-process tests force termination during writes, checkpoint updates and ZIP
finalization, then verify and query the recovered prefixes. A Linux `/dev/full`
fixture exercises actual ENOSPC; other hosts skip that OS-specific fixture. Index
queries reject hash-valid indexes whose channel, record boundary or time disagrees
with the canonical stream. The final saved-flight experiment workflow and updated
final qualification are the remaining closure work.

## F4 / G3 — saved-flight experiments and final workspace (2026-10-28)

Saved-result experiments now use a new mission identity and a versioned, hash-pinned
selected-member source. The immutable source includes truth/TX/events and original
flight/codec inputs. Users explicitly choose original transmissions or new standard
packets; neither path runs an FC. Portable setups retain the source, fixed launch and
flight inputs are protected, and receiver/link/video settings remain editable. Packet
identity, time, channel, bounds and hashes are checked during incremental replay.

All 70 Java mission checks passed (the Linux-only ENOSPC fixture is skipped on this
Mac). Tests cover standard and custom packet replay, exact original TX bytes, standard
regeneration, portable-source import and flight-edit rejection. The browser acceptance
passed offline setup, normal run, pause/browser reconnect, complete and recovered
station export and creation/execution of a new recorded-flight experiment. It found
and fixed a result time-window field interfering with configuration validation.

The native macOS ARM image passed standard/custom-FC missions and both recorded-packet
experiments using its bundled runtimes. Report:
`build/mission-package/macos-arm64/report/20261028T191709.945923Z/report.json`.
The preceding recovery revision passed all four platforms in Actions run 40749647820;
the final source revision will receive the same matrix before completion. Sixty-two
Python contract/codec/station/result/application checks passed. A combined desktop
video run exposed an intermittent PySide native object-construction crash; final
video stability investigation is still in progress. This entry does not count that
failed run as a pass.

## G4 — desktop runtime stability and final qualification (2026-10-28)

The final OpenRocket implementation at `f7041648` passed all four native image jobs
in [Actions run 40753227393](https://github.com/MDNich/ActiveControl_MIT_RktTeam/actions/runs/40753227393):
Linux x86_64, Windows x86_64, Mac ARM and Mac Intel. Those jobs exercise bundled
Java/Python execution, standard and custom-FC missions and both recorded-TX experiments.
The recovery revision and the final revision both passed the matrix.

The desktop crash was narrowed to stale native layout wrappers: a repeated station
rebuild returned `QWidgetItem` from `menuBar()` on PySide 6.11.2; 6.11.3 crashed in
the same rebuild workload. Qt 6.12 includes the relevant layout-ownership fix
([PYSIDE-3469](https://github.com/pyside/pyside-setup/commit/72a3b4d7802203d53d2989fa61db39322b852864)).
Telemetry UI `5b737ce` pins PySide/Shiboken 6.12.0 in both lockfiles, checks the desktop
runtime before Mission connection/launch, and reports an actionable update instruction
for older environments. Its bundled headless worker remains Qt-free.

The qualified runtime passed 98 Mission/station/media checks and 59 additional
existing desktop-control/layout checks. These runs used a source copy excluding the
unrelated local table-cell and recorder edits. The regression rebuilds Base and Away
workspaces 30 times in a separate process, destroys their layouts and allocates new
Qt objects to exercise address reuse. The video fixture uses actual H.264/SRT delivery to the Base wall,
outage/recovery and an encoded recording. Three OpenRocket integration cases also
passed against that source/runtime: fresh launch, existing-app attachment and observer
operation, including ownership transfers and mandatory-participant reconnect.

The separate [Telemetry UI desktop matrix](https://github.com/MIT-Rocket-Team/RT_Telemetry_UI/actions/runs/40759027040)
passed on all four operating-system/architecture targets at `9797bfe`: **65 checks
per platform**, including real software application connections, offline attachment
startup, native window rebuilds and recorder shutdown. Both the OpenRocket native-image
matrix and the Telemetry UI desktop matrix are green for the delivered implementations.
Its first passes exposed platform test issues: native MGRS differs in the last floating
bit, a large unnamed JSON fixture exceeded Windows' environment-variable limit, and
the Intel runner needed more time for all 30 rebuild cycles. Stored defaults remain
exact; the independent converter comparison now allows 0.1 mm and malformed fixtures
have short explicit names. Telemetry UI `0216841` also removes HTTPServer's unnecessary
reverse DNS lookup from the private loopback attachment listener. A DNS-disabled
regression fails before that fix and passes afterward. Fresh launch, attachment and
the entire rebuild workload passed locally again, as did the three Java/application
integration cases using the final runtime and listener.

The final rebuild test explicitly checks that every native recorder thread has
stopped, has no error and has completed its manifest. It retains scratch recordings
until pytest cleanup so Windows file-sharing locks do not interrupt the Qt-lifetime
test itself. CI retains child logs, including partial logs from timeouts. This test
adjustment changes no application behavior.

The browser acceptance already passed configuration/save/load, offline coordinates,
paced execution, pause/browser reconnect, complete and recovered result export, and
creation/execution of a new saved-flight experiment. Both exported `.rktflight` files
opened in the existing Telemetry UI reader.

The main working environment has been updated to the pinned desktop runtime. Release
publication remains a separate operation; these commits do not publish an installer
or claim hardware/RF qualification. Ground-only pointers, one-stage/one-FC coupled
execution and the illustrative beam-width model retain the limits in the user guide.

## G5 — actual Mac launcher acceptance and packaging repairs (2026-10-28)

User reports exposed a gap in G4: the package fixtures invoked Java on a flat
classpath, and browser acceptance used a development host. Neither exercised the
native launcher's isolated class loader or the welcome dialog. The earlier green
matrix therefore did not establish full desktop startup acceptance.

Three repairs address the reports:

- `processResources` now bundles `ReleaseNotes.md` directly. The previous Copy task
  configured its source inside `doLast` and left the file out of the distribution.
  The development fallback handles a filesystem root without a parent and reports
  missing notes as an ordinary I/O error. Notes are read as UTF-8.
- The isolated application loader delegates platform classes to Java's platform
  loader. This exposes `jdk.httpserver` and `java.net.http`, required by Mission,
  while retaining isolation from the system application classpath. A bootstrap-only
  parent caused `NoClassDefFoundError` when the File menu created `MissionApi`.
- Native images explicitly include the existing platform OpenRocket icon. The Mac
  package verifies that its plist-selected icon matches the source artwork byte for
  byte. The previous qualification image silently used the generic Java icon.

Local acceptance launched the rebuilt Apple Silicon `.app/Contents/MacOS/OpenRocket_MIT`
from `/`, with its bundled Java/Python and no shell working-directory dependency.
The disposable `MissionDesktopSmokeAgent` isolates preferences in memory (macOS does
not honor `java.util.prefs.userRoot`), observes and closes the real welcome dialog,
opens the bundled model rocket through the normal example loader, and clicks the
real **New Mission Simulation** menu item. Production `MissionWorkspace` owns the
service and native file dialogs; no replacement host is injected.

The browser then binds that current rocket and passes offline coordinate editing,
flight/link/video configuration, native save/load dialogs, a paced run, pause and
browser reconnect, named result creation, indexed queries, complete and recovered
station exports, and a new recorded-flight experiment. It records native-window and
browser screenshots. The agent exits only its own disposable test process. Local
results are retained in `build/mission-package/macos-arm64-desktop-fix/report/desktop`.

Three welcome regression tests passed. Standard and custom-FC packaged fixtures,
including recorded-TX reruns, also passed through `JarInJarStarter` from `/` (61 and
50 accepted frames respectively). The new launcher probe rejects the prior image.
The native Mac workflow is now part of package qualification for both Mac targets;
all four targets run the isolated-loader probe and packaged-resource checks. These
new CI results must be assessed independently of the earlier G4 green matrix.

`packaging/verification/mission-desktop-smoke.py APP REPORT` reproduces the full Mac
test with `JAVA_HOME`, Node and Playwright available. `PLAYWRIGHT_MODULE`, `NODE` and
`BROWSER_EXECUTABLE` can select installed tools; the report directory must be new.
`packaging/mission/qualify.py --output DIRECTORY` builds a separate application image
so a currently running user copy need not be overwritten. Release publication is
still separate from this repair.
