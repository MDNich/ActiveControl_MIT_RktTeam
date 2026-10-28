# Mission Simulation configuration: implementation specification

Implementation status (28 October 2026): this document retains the reviewed plan and contracts. See [the implementation ledger](18-mission-implementation-progress.md) for completed increments and evidence, and [the user guide](19-mission-simulation-user-guide.md) for the implemented workflow and limits.

Implementation specification finalized 27 October 2026; final consistency review 28 October 2026. This extends the [Mission Simulation architecture plan](15-mission-simulation-plan.md) with a new configuration interface and the requested standard telemetry and video simulation. The [final implementation contracts](17-mission-simulation-contracts.md) fix runtime, clock, packet, height, storage and application-boundary details and are part of this handoff. The existing flight configuration dialog is not the layout template: reuse its data and behavior behind a new mission workspace. The configuration prototype is delivered; application integration is the remaining work specified below. Application version 6.5 is metadata, not a claim that the feature already exists.

An [interactive design prototype](mission-ui/index.html) accompanies this specification. It has real map imagery, draggable sites, latitude/longitude and MGRS conversion, configuration import/export, editable settings, variable stream counts, Balius/Iris presets and downloadable PNG slates. It is a configuration preview, not a working mission runner. It does not parse `.ork` internals, execute FC code, connect the Telemetry UI or transmit video. Its file format is deliberately named as a preview format rather than claiming compatibility with an implemented simulator.

## 1. Workspace and navigation

Keep the mission title and run label visible above six freely accessible sections. The top bar always offers **Load configuration**, **Save configuration** and, in the implemented application, **Save as**. A compact left navigation replaces the existing flight dialog's tabs and grouped boxes. Navigation does not discard unapplied changes.

| Section | Primary task |
| --- | --- |
| **Site & stations** | Choose launch coordinates, Base, 0–4 pointers and other receiver sites on a large map or through coordinates. |
| **Rocket & flight** | Choose the `.ork` and flight configuration, import a saved simulation, and edit launch, weather, atmosphere, motors, staging, recovery and solver settings. |
| **Telemetry** | Choose an FC/format or standard Zephyrus telemetry; configure transmission, measurement noise and independent receiving links. |
| **Video** | Choose Balius, Iris or Custom; set analog/digital counts, vehicle identities, receiver assignments, image slates and impairments. |
| **Connections** | Locate the Telemetry UI, choose Base/Away instances and roles, and assign simulated serial/video endpoints. |
| **Run & results** | Select execution mode, ensembles, reproducibility and result location; inspect and resolve validation findings. |

Use a large map and one adjacent site inspector in the first section. Other sections use compact, labeled forms with units and expandable advanced settings. Keep one clear primary action per view. In the real application, **Run mission** appears only after validation and shows the selected execution mode; the prototype uses **Check configuration** to avoid suggesting a working runner.

Drafts are editable without a rocket or installed Telemetry UI. Validation distinguishes missing requirements for a run from optional features. For example, an unavailable map service must not prevent numeric coordinate entry, and an unavailable UI installation must not prevent a headless mission.

## 2. Saved configurations and asset handling

Introduce a versioned mission configuration separate from the result bundle. It contains stable mission/site/stream IDs; launch and station coordinates; all flight and ensemble settings; FC and codec references; per-link noise/loss profiles; stream definitions; station connection profiles; and output preferences. Result files add immutable resolved copies and run/member IDs.

On **Load configuration**, show the title, source version, referenced rocket/FC assets and unresolved paths. Resolve relative paths against the configuration's location; offer **Locate…** for missing assets. Preserve original references, content hashes and unknown settings until migration explicitly handles them. Never silently substitute default flight settings because a control is absent in the new UI. Importing an ordinary saved OpenRocket simulation is a separate **Import flight settings** action with a preview of the settings it replaces.

Save the complete draft atomically as `.rktmission-config.json`. **Save as portable configuration** creates a `.rktmission-setup` ZIP containing the `.ork`, `.fc`, behavior sources, motors, codec descriptions, required geoid data and slate images; machine-specific executable paths are optional overrides. Keep result bundles separate as `.rktmission`. Store map provider/style identifiers and viewport, not unrestricted copies of third-party tile caches or API secrets. Changing a label does not change a stream/site ID. Duplicating a mission creates a new mission identity while retaining chosen settings.

Opening a configuration must not start an application, open a device or execute a bundled extension. At run time, use the existing FC/extension trust checks and an explicit **Launch selected simulated stations** action. This is ordinary run setup, not a new approval requirement for editing files.

## 3. Map, coordinates and site placement

### Default URRG configuration

New missions open at **URRG with four antenna pointers enabled**. Use the following exact references from Telemetry UI's [site_presets.py](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/site_presets.py), decoded by its [location.py](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/location.py). Keep one Base station plus four Away stations; the user's four nearby ground sites are the four Away/pointer sites in the existing station protocol.

| Site | Mission/wire ID | Exact MGRS preset | Latitude ° | Longitude ° |
| --- | --- | --- | --- | --- |
| Launch pad | `launch-site` (physical pad, not station ID) | `18TUN2061530290` | 42.70416776971639 | −77.19029501648315 |
| Base station reference | `launch` | `18TUN2063730181` | 42.70319201731030 | −77.18999209440938 |
| Away 1 / Pointer 1 | `gs1` | `18TUN2177730106` | 42.70278230997951 | −77.17605911889694 |
| Away 2 / Pointer 2 | `gs2` | `18TUN2291333237` | 42.73122112935536 | −77.16317739743833 |
| Away 3 / Pointer 3 | `gs3` | `18TUN2015031101` | 42.71135721362275 | −77.19622600691332 |
| Away 4 / Pointer 4 | `gs4` | `18TUN2259229678` | 42.69911925972465 | −77.16598116908455 |

Preserve the supplied MGRS strings and the canonical coordinates produced by Telemetry UI. Its decoder uses the **grid reference point**, whereas the prototype's generic JavaScript converter returns a **cell center**. These are different conventions: importing or reapplying an unchanged URRG code must retain the existing point, not introduce a half-cell offset. Store the chosen convention and 1 m grid resolution with each location. Display-format changes alone never move a site.

These presets contain **no elevations**. New mission fields are unset (`null`) and marked required before running; do not retain the former desert example's 1400 m or reinterpret Telemetry UI's zero placeholder as a surveyed height. The saved sample likewise has unset elevations. Manually entered heights and their datums are preserved on reload. Fit the initial map to all six points, retaining separate selectable Launch and Base markers even though those two sites are close.

URRG is a default for new drafts only. Loading a saved configuration remains authoritative, including a different launch area, fewer pointers or receiver-only sites. A future **Restore URRG locations** action replaces horizontal preset positions explicitly and preserves entered heights; it does not reset the rocket, link profiles, calibration or connections.

### Editing locations

The site list contains **Launch**, **Base**, and Away 1–4, with a pointer enabled or disabled for each Away site. An Away site can remain a receiver-only site without increasing the pointer count; a video receiver does not inherently require a steerable antenna. This delivery retains the four existing Away wire IDs and does not invent a fifth station. Additional receiver channels within a site require declared endpoint/capability support. Base and launch are separate locations. A **Use launch position** shortcut copies coordinates explicitly rather than permanently coupling them.

Changing pointer count creates or disables pointer assignments. Preserve disabled entries in the draft for restoration, but omit them from the active run. Do not move existing sites or fabricate positions for newly enabled ones. Mark unplaced sites directly in the list and in run validation. Selecting a site reveals its coordinate and mount settings. **Place on map** arms a single placement; dragging a marker or applying coordinates updates both views. A normal map click does not move an unselected site.

Offer decimal WGS84 latitude/longitude and MGRS. Use a tested coordinate conversion library and record the entered form and precision. MGRS denotes a grid cell: show its precision and the selected representative point. Production manual entry uses Telemetry UI's grid-point convention; retain existing coordinates and provenance when importing a cell-center location. New manual MGRS entries in the prototype still use its labeled cell center and require that explicit migration handling. Reformatting a point must not repeatedly quantize its stored coordinates. Validate zones, bands, equal easting/northing precision, hemisphere and bounds; handle unsupported UPS/polar references with an explicit error and latitude/longitude alternative. The prototype uses [proj4js/mgrs](https://github.com/proj4js/mgrs); production conversions need shared independent golden vectors.

Elevation is a separate required field with an explicit MSL/ellipsoidal reference. Neither MGRS nor a clicked image supplies height. A terrain-height lookup, if added, is an explicit action that records provider, datum and resolution. Convert MSL to ellipsoidal height using a known geoid model before the station geometry calculation. Preserve survey coordinates, receiver-reported GPS position/noise, mount height and antenna offsets separately. A one-metre grid reference is not evidence of one-metre survey accuracy.

For each pointer, expose initial azimuth/elevation, mount height, boresight offsets, motion-rate/acceleration limits, angular travel and chosen tracking policy. Put mechanical details under **Advanced mount settings**. The default virtual rates are labeled illustrative. Calibration/ZERO semantics must distinguish coordinate reset from physical movement, as noted in plan 15.

### Map providers

The prototype loads real imagery in the browser and retains visible attribution. Its initial USGS aerial/topographic layer covers URRG; Esri imagery and OpenStreetMap streets are selectable. The horizontal positions match the team's Telemetry UI presets; the preset is not a newly verified field survey. Satellite imagery dates and survey heights must not be inferred from the map's appearance.

| Provider | Design decision |
| --- | --- |
| **USGS aerial + labels** | Working prototype default for the US example. The published service combines orthoimagery with topographic data; do not call it worldwide current satellite coverage. [USGS service](https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryTopo/MapServer) |
| **Esri imagery** | Working selectable imagery layer with source attribution. Production use must retain provider credits and the chosen service's access terms. [Esri basemap information](https://doc.arcgis.com/en/arcgis-online/reference/faq.htm), [attribution guidance](https://support.esri.com/en-us/knowledge-base/what-is-the-correct-way-to-cite-an-arcgis-online-basema-000012040) |
| **OpenStreetMap** | Global street-map fallback; visible attribution, ordinary caching and viewport-driven requests. Do not bulk-prefetch public OSM tiles into mission bundles. [Tile policy](https://operations.osmfoundation.org/policies/tiles/) |
| **Google satellite/hybrid** | Optional configured provider through its official API, using the user's project/key and required attribution. Do not use scraped undocumented tile endpoints. [API setup](https://developers.google.com/maps/documentation/tile/get-api-key), [FAQ](https://developers.google.com/maps/faq) |
| **Sentinel-2** | Optional imagery provider through a configured Copernicus/Sentinel Hub OGC service; offer observation date and cloud selection where supported. An instance/authentication setup is required for that route. [OGC service](https://documentation.dataspace.copernicus.eu/APIs/SentinelHub/OGC.html), [authentication](https://documentation.dataspace.copernicus.eu/APIs/SentinelHub/Overview/Authentication.html) |

Provider failure leaves coordinates and mission settings intact. Show a small unavailable message on the map and allow another provider. Cached imagery/offline packages require a provider that permits that use; the simulation's numerical execution does not depend on continued internet access.

## 4. Preserve the complete flight configuration

The replacement interface binds to the same simulation/options/extension models. Import and snapshot the selected `Simulation`, flight configuration and FC mappings, rather than rebuilding a nominal default flight. The current Telemetry UI `RocketBridge` clears extensions and is unsuitable for this path.

| Existing data | New location and parity requirement |
| --- | --- |
| Rocket, flight configuration, active stages and motor assignments | **Rocket & flight → Vehicle**. Preserve motor data, ignition conditions/delays and staging events/delays. Editing the mission draft does not silently modify the source `.ork`. |
| Recovery device events, delays and altitude settings | **Vehicle → Recovery**. Show which system owns deployment: normal OpenRocket rules or explicit FC mappings. Preserve both saved configurations when switching authority. |
| Launch latitude, longitude and altitude | **Site & stations → Launch** is the single source of truth. The flight page shows a linked summary rather than a second independent set of coordinates. |
| Guide length, tilt, direction, into-wind choice and initial roll rate | **Launch guide**. Preserve units, bounds and existing direction conventions. Into-wind mode disables manual guide bearing without deleting its stored value. |
| Average wind speed, direction, standard deviation and turbulence intensity | **Wind**. Maintain existing dependencies between speed deviation and turbulence intensity; do not let two inconsistent independent values reach the engine. |
| Multi-level wind profile and altitude reference | **Wind → Levels**. Editable rows, import/export, validation and the existing interpolation/model semantics. |
| ISA/custom atmosphere, temperature and pressure | **Atmosphere**. Custom values remain stored while ISA is selected. |
| Geodetic method, timestep, maximum duration, maximum step angle and random seed | **Advanced flight settings**. Preserve existing defaults and serialized values; the mission geometry's canonical ECEF conversion does not silently change the flight solver's chosen geodetic strategy. |
| Simulation extensions and their configuration | **Extensions**. Preserve enabled state, order/settings and unsupported extension data; use a dedicated editor where necessary. Report parallel-execution constraints. |
| FC hardware/sensors/behavior/state machine/timing | **Telemetry → Flight computer** with access to the FC definition editor. Preserve source identity and every resolved option. |
| FC controllers, sensor selection, actuator ownership/mapping, PWM and recovery mappings | **Flight computer → Physical assignments**. Show missing or incompatible rocket component IDs before running. |
| FC telemetry/output settings | **Telemetry** and **Run & results**. Preserve timing and choose one authoritative place for link loss/delay so the old filter is not applied twice. |
| Full ensemble options | **Run & results → Ensemble**, described below. |

Implement parity through a setting inventory and round-trip fixtures: import a configured ordinary simulation, save/load a mission, reconstruct the flight configuration, and compare all semantic fields. This is stronger than a promise that “normal options still work.” Some prototype fields summarize structured assignments; production editors must retain their complete underlying objects.

Preserving a setting does not make an unsupported combination executable. The current FC listener requires one active stage and pad startup. Document 17 defines precomputed ordinary multibranch flights for standard telemetry/Iris ground-link rehearsal, and single-active-stage coupled custom-FC runs. Show branch/vehicle bindings explicitly and reject unsupported FC combinations before starting, without deleting their saved settings.

## 5. Telemetry source and format selection

Expose two independent concepts: **FC execution** and **telemetry encoding**. The primary user choices can remain simple:

1. **Standard telemetry — no custom FC.** Precompute normal OpenRocket physics and recovery, then run links/stations against those branches. A read-only synthetic sensor/observation source emits the legacy Zephyrus payload at the chosen rate. Broadcast each transmission to every configured receiver tuned to that vehicle, with independent reception outcomes; a single-vehicle mission sends to all enabled receivers. It does not create an FC controller or grant onboard actuator authority.
2. **Custom flight computer.** Execute the selected FC with its own sensors, timing, behavior and mappings. If its definition supplies a supported telemetry-format declaration, use that encoder. If it has none, clearly offer **Use legacy Zephyrus encoding** or **Require a format definition**; never silently infer a format from a vehicle name.

The current `.fc` implementation does not provide the general, negotiated telemetry-format registry required here; the built-in legacy packet layout and interval settings are not that registry. Document 17 specifies `.fc` v3 with an optional bounded declarative format, preserving v1/v2 support, plus format ID/version, frame layout, byte order, units/scales, field validity, checksum, timestamp/sequence semantics, encoder identity and receiving decoder capability. Freeze it into the mission bundle. An advertised format must have a matching Telemetry UI decoder before a UI-connected run starts. The current serial worker constructs the fixed legacy decoder.

The packet inspector shows source, format, rate owner, payload length, receiver framing and a sample decoded message. For custom FCs, the FC schedule owns transmit timing unless an explicitly supported override is enabled. Legacy output remains the known 128-byte payload wrapped into a 144-byte station frame. Custom formats declare their own envelope; do not force arbitrary formats into 128 bytes.

For the standard generator, specify mappings from flight/synthetic-sensor values to every supported legacy field. Keep measured/noisy inputs distinct from truth. Fields with no simulated meaning—such as real board current, physical pyro continuity or a hardware-specific state—need a documented wire placeholder and explicit unavailable/synthetic provenance in the receiving adapter and archive. Do not present invented zeros as actual measurements or derive attitude from gyro integrals. Use golden wire vectors shared by Java and Python.

Separate measurement noise, packet loss/corruption and delivery timing. Show global defaults with receiver overrides. Production starts with the explicitly ideal zero-impairment profile; prototype nonzero values are demonstration inputs. Advanced controls include seeded loss bursts, scheduled outages, delay/jitter, bit errors, GPS horizontal/vertical error, barometric error and freshness policy. Document 17 defines distributions, order, FIFO/reordering and clocks. FC sensor noise belongs to the FC model; receiver location error and synthetic-observer noise belong to their respective sources. Avoid applying the same noise twice. Radio reception and station LAN delay are separately labeled stages.

## 6. Video configuration and actual Balius/Iris behavior

The read-only review with the **Telemetry UI** chat confirmed the current layouts:

| Preset / vehicle source | Existing logical channel ID | Away receiving stations |
| --- | --- | --- |
| Balius Digital 1 | `balius.digital` | GS1–GS4 |
| Balius Analog 1 | `balius.analog` | GS1–GS4 |
| Iris Digital 1 — Sustainer | `sustainer.digital` | GS1–GS4 |
| Iris Analog 1 — Sustainer | `sustainer.analog` | GS1–GS3 |
| Iris Analog 2 — Booster | `booster.analog` | GS4 |

Thus the presets are **Balius: 1 digital + 1 analog** and **Iris: 1 digital + 2 analog**. These count onboard logical streams, not receiver copies. A station stream ID includes its receiver, for example `gs4.booster.analog`. Iris GS4 still receives Sustainer Digital even though its analog/telemetry association is Booster. Base/Video currently exposes local Digital only. The prototype's customizable receiver matrix is a proposed capability; compatibility validation must identify nonstandard routes.

Allow **Custom** with independent nonnegative digital and analog counts. Stream count is not limited by the four-pointer limit. Production limits should be capability/resource based and visible; the prototype uses a bounded 0–32 input per type solely to keep the preview manageable. The current Telemetry UI cannot already represent arbitrary `digital2` or `analog3`: update channel/profile validation, source registry, receiver mapping, wall layout and recording identity together.

Each stream row contains enabled state, stable ID, name, digital/analog type, vehicle/stage, selected receivers, associated antenna/link model, slate preview and transport profile. A shared pointer cannot simultaneously point toward separated Booster and Sustainer targets: show the chosen tracking target and allow a fixed/omnidirectional antenna or another pointer for other streams. Do not silently use the correct truth boresight independently for every stream on one physical mount.

### Static image content and transmission

Create one PNG by default, optionally JPEG, per logical stream and run/member. It prominently says **MISSION SIMULATION** and **DIGITAL 2** or **ANALOG 1**, followed by mission title, run/member ID, `.ork` basename and vehicle/stage. Put stream ID, seed, format version and full provenance in an accompanying JSON record. Keep essential labels legible in the 320×180 preview rendition. The image is static: no fabricated camera scene or flight footage is needed.

A still image is the source asset, not the wire transport. Load it once into the frame buffer and emit identical pixels with fresh timestamps at the selected cadence. The existing UI's `MediaService.set_source()` accepts a source with frame listeners and status/source-mode properties; `scripts/network_pair_check.py` already demonstrates a simulation-tagged `SyntheticCapture`. Reuse that seam with the slate pixels and the normal encoder/network path.

Both real analog and digital video currently arrive at the computer as captured digital frames. The live network path uses H.264 Baseline/YUV420P, MPEG-TS and SRT. Current settings are 320×180 at 10 fps for preview and 960×540 at 30 fps for main. The default source normalization is 960×540 RGB24 without audio. Preserve the supported settings as the first compatibility profile; custom resolutions require capability validation. An unchanged application's file input can instead consume a repeated-image MP4/MKV, but a bare PNG/JPEG is not a continuous stream in that input path.

The UI presently filters standard video publishers and the Base wall to LIVE sources. Add explicit simulation acceptance and provenance to both; do not relabel slates LIVE. Existing allocation has two preview and two main sessions total per publisher, across its channels. Show estimated receiver/session demand during review and test admission failures; stream counts alone do not establish concurrency capacity.

Configure impairments per receiver: frame drops, freeze/outage intervals and added playback delay, with optional illustrative noise applied to analog pictures. Keep RF link loss, simulated frame loss and LAN/SRT packet loss separate: SRT retransmission can hide packet loss. Repeated identical pixels are fresh when timestamps/frames advance; stopping delivery must exercise the actual stale threshold, currently five seconds. Do not inject new “NO SIGNAL” frames as a substitute for missing frames when testing freshness.

Accelerated/headless members record logical frame events and assets without requiring an encoder. Full H.264/SRT testing is paced. Pause freezes mission frame time/freshness while transport leases continue on host time; any repeated paused carrier frames are excluded from mission reception metrics. This requires the explicit paused-clock capability in document 17.

## 7. Connections and station startup

The application selector supports a packaged executable, source-project directory plus interpreter, or attach-to-running mode. Show the resolved location and verify it when the user chooses **Check connection**. Known local choices are `/Users/mdn/Developer/RT_Telemetry_UI`, its `.venv/bin/python`, and `dist/RocketGNCMonitor-v0a.app/Contents/MacOS/RocketGNCMonitor-v0a` beneath that project.

Present a station/role table: Base and enabled Away sites, telemetry role, video role, chosen receiver/pointer software endpoints, service port, data directory and connection state. Telemetry and video are separate processes/roles in the current UI architecture. Assign unique instance/node/boot identities, directories and ports while sharing an explicit mission UUID. For ordinary software integration, use named duplex software serial endpoints; expose PTY/virtual COM only as advanced platform-specific options. A selected app path is not proof of compatibility.

Existing CLI arguments cover site, role, vehicle, data directory, mission ID, HTTP bind/port, peers and discovery. They do not currently load a mission configuration, assign serial/video inputs or start polling. Add a versioned launch manifest/control handshake rather than inventing unsupported flags. Use structured process arguments and check `/api/v1/info` for identity/readiness. Check required codecs, simulation acceptance and stream limits before enabling **Start mission**.

Keep **Launch application**, **Connect simulated endpoints**, **Start polling** and **Start mission** as distinct states, with a convenient guided sequence. Reopening a preset never performs them automatically. Base owns simulated uplink authority; Away telemetry/pointer control and read-only remote observation remain separate. The undefined physical Base enable, Iris target switch and remote-pointer firmware commands remain outside this software handshake.

## 8. Ensembles and results

Expose single-flight versus ensemble selection, run count, worker count and seed. Preserve every current uncertainty input: additive thrust noise and its sample interval; temperature σ; pressure σ; independent east/north wind σ; wind-bearing σ; total mass σ; overall drag σ; roll forcing σ; and roll damping σ. Keep their current units, distributions and invalid-multiplier behavior. Add independent station/link/video seeds without perturbing existing flight sampling.

For an ensemble, the live UI attaches to one explicitly selected member, or replays a selected completed member. Do not mix every member into one station's byte stream, create hundreds of GUI instances automatically or reuse station/port identities concurrently. Headless members still record packets, poses and video delivery events.

Store each static slate once, with its hash and metadata, plus codec/profile, frame cadence and impairment timeline. Deterministic replay can regenerate repeated frames without storing identical images thousands of times. Save encoded recordings only when requested or needed for codec/network qualification. Preserve per-receiver differences and gaps. Add `capture-timing.jsonl` explicitly: the current `.rktflight` exporter does not include that sidecar, and remote video-wall reception currently lacks the corresponding recording path.

The ZIP Mission Bundle contains the frozen configuration, FC/standard-generator source, transmitted packets, per-station raw reception, pointer commands/pose, network observations, slates and video events, sampled ensemble parameters and run status. Keep optional single-session `.rktflight` compatibility exports described in plan 15. Map tiles are presentation resources, not required simulation inputs.

## 9. Verification strategy

Verify each boundary independently and then exercise the complete mission through both applications. The work packages below define implementation order and ownership; these acceptance fixtures apply throughout.

Acceptance includes full flight-settings round trips; preset save/load and missing asset handling; coordinate axis and MGRS precision fixtures; malformed custom packet definitions; Balius/Iris receiver routing including GS4; custom Digital 2 capability handling; legible decoded slates at both renditions; stale/recovery; failed main-stream promotion retaining the previous image; publisher capacity exhaustion; and archive replay from complete and interrupted runs.

The prototype was checked in a browser for real imagery loading, coordinate/MGRS edits, invalid coordinate handling, pointer counts, FC mode selection, video counts/presets, PNG export, ensemble visibility, configuration round trips and responsive layouts. The URRG preset is additionally checked against the Telemetry UI decoder, including exact retained codes, unchanged points on reapply and missing-elevation validation. This verifies the design interaction, not a simulator or video integration that has not been implemented.

## 10. Implementation architecture and ownership

Use the delivered browser workspace as the presentation direction. For the first application integration, bundle its production HTML/CSS/JavaScript with OpenRocket and serve it through a **Java-owned local mission service**, bound to loopback on an allocated port. **New Mission Simulation** in OpenRocket opens that workspace in the browser. An embedded desktop browser can use the same interface later without changing mission semantics. This avoids introducing a second flight engine or requiring Node.js on the user's computer.

The local service is the authority for validation, file selection/resolution, canonical coordinates, flight snapshots and job lifecycle. Client-side previews can show immediate feedback, but cannot bypass server validation. Serve bundled, versioned UI libraries locally; external access is for the selected imagery provider. Use a per-launch session token and origin checks for state-changing requests. Keep this local control service separate from the Telemetry UI's station LAN service and its existing network protocol.

| Owner | Responsibility and proposed modules |
| --- | --- |
| OpenRocket core | `MissionDefinition` and versioned reader/writer; full simulation snapshot/import adapter; validation; `MissionRunner`; standard telemetry source; FC codec adapters; radio/pointer models; result store. Proposed classes live in a dedicated mission package. |
| OpenRocket desktop | Menu entry, local mission service, native asset/folder selection, lifecycle ownership and opening the workspace. No flight execution on the Swing event-dispatch or HTTP request thread. |
| Web workspace | Six configuration views, local draft editing, map/provider display, typed error presentation and run progress. It sends structured configuration changes, not executable shell commands. |
| Telemetry UI | Simulation transport factory, shared headless station controller, decoder registry, launch-manifest reader, simulation source/clock adapter, generalized video channels, slate capture source and replay/import support. |
| Shared contracts and fixtures | Versioned schemas, legacy packet vectors, URRG/coordinate expectations, source-mode/role definitions, channel mappings and bundle examples. Both repositories validate the same fixtures. |

Freeze an immutable resolved configuration before scheduling a run. UI edits during execution create a new draft revision; they do not change the active run. Interactive actions and intentionally supported live configuration changes use timestamped command events, not mutation of the original snapshot. Continue to use isolated rocket/FC copies for members and existing extension parallelism restrictions.

The shared station controller is a Qt-independent Python module used by both the UI and a headless worker. Package that worker/runtime with OpenRocket and communicate through `rt-mission.v1` loopback IPC; do not assume Java can call it in-process or require an external Telemetry UI for a headless run. Each station has exactly one command owner. Document 17 defines ownership transfer, barriers, byte chunks and queue failure policy.

### Configuration contract

Adopt production format `openrocket.mission` version 1, distinct from `openrocket.mission-configuration-preview`, using `.rktmission-config.json`; portable inputs use `.rktmission-setup` and results use `.rktmission`. The schema must contain these groups:

| Group | Required semantics |
| --- | --- |
| `identity` | Stable mission UUID, title, schema version and draft revision; run/member identity is generated separately. |
| `assets` | Rocket, motors, FC definitions, behavior and codec assets with original reference, portable relative reference and hash where resolved. Machine paths are not asset identity. |
| `flight` | Complete selected simulation/options/flight-configuration and extension snapshots, not only the fields currently visible in the web form. Use adapters to the existing serialization/model contracts. |
| `vehicleBindings` | Logical vehicle/transmitter identities, selected component/stage UUIDs, branch lineage and active intervals; never infer a vehicle from a legacy payload or branch label. |
| `sites` | Launch plus station IDs, canonical WGS84 coordinates, explicit nullable height/datum, original coordinate input, grid convention/precision and location provenance. |
| `pointers` | Zero to four stable pointer IDs, site association, antenna/mount profile, initial physical pose and calibration state, tracking policy and target vehicle. |
| `telemetry` | FC execution choice, encoder declaration/fallback policy, cadence owner, synthetic observation validity, receiver links and independent seed identities. |
| `video` | Logical stream definitions, vehicle/source identity, image asset/template, receiver associations, antenna/link association, rendition and impairment profiles. |
| `stationBindings` | Site/role instances, application locator, software endpoint selections and requested capabilities. Store preferences separately from ephemeral connection tokens, boot IDs and allocated ports. |
| `execution` | Mode, full ensemble settings, stable random-stream rules, selected interactive member and pacing policy. |
| `outputs` | Result destination, retention choices, encoded-video capture policy and compatibility exports. |

Unknown major versions fail clearly. Unknown optional fields and unsupported extensions must be preserved on read/save or rejected with a specific explanation, never discarded. Save drafts with unresolved assets/heights, but block dependent run actions. Validate all numerical ranges, units, finite values, unique IDs, pointer limits, route references and required capabilities before producing the run snapshot. The prototype's numeric count cap is not a production capacity guarantee.

### UI-to-runner and station contracts

Implement the `/mission-api/v1/` local-service operations in document 17 for **capabilities**, **load/import/save draft**, **validate/resolve**, **start**, **pause/resume/cancel**, **job status/events** and **open results**. Use schema-versioned structured requests, explicit revision/hash matching and bounded asynchronous event delivery. A start request creates one idempotent job; retrying after a lost response must not start a duplicate mission. Validation returns field paths and severity so the workspace can navigate to the exact control.

The station launch manifest carries mission/run/member identity, station and role, source mode `simulation`, transport references, telemetry format ID, expected vehicle/stream IDs, clock/pacing mode, data directory and requested network bindings. The station responds with its actual node/boot identity and capabilities: supported codecs/stream layouts, clock mode, software transport and video resource limits. Resolve optional features or report incompatibility before connecting. Never fabricate existing CLI arguments to achieve this contract.

Keep packet bytes and typed control messages separate. Serial data retains exact bytes and chunk timing; mission commands carry an event ID, target participant and admission time. External callbacks enqueue bounded work. The mission scheduler admits FC-bound bytes only at valid virtual deadlines, as specified in plan 15. Reconnects establish a new connection identity and cannot replay uncertain non-idempotent uplink commands automatically.

The run state machine is **Draft → Validating → Ready → Starting → Running ↔ Paused → Finalizing → Complete**, with explicit **Cancelled** and **Failed** outcomes; ensembles can finish **Complete with failures** while preserving successful members. Stage and station status remain separately visible. Cancellation closes only resources owned by the job, drains committed recorder prefixes and writes partial completion metadata. Result finalization failure preserves the staging directory and previous valid output. A saved file or replay begins disconnected regardless of its prior run state.

## 11. Work packages and definition of done

| Package | Deliverable | Required acceptance evidence |
| --- | --- | --- |
| **A. Shared contracts and URRG defaults** | Production schemas, site preset fixture, geodesy/provenance conventions and complete flight setting inventory. | All six horizontal positions match Telemetry UI; unchanged MGRS reapply does not move them; height remains unset; imports do not overwrite saved locations. |
| **B. Configuration service and workspace** | New OpenRocket entry point, bundled workspace, file resolution, save/load, all six editors and typed validation. | No existing flight/FC/ensemble setting is lost in round-trip fixtures; no asset execution or connections on load; native/packaged launch works offline apart from imagery. |
| **C. Headless mission and standard packets** | Full flight/branch adapter, virtual scheduler, standard Zephyrus observation generator, per-receiver links, bundled Python station worker and pointer plant. | No-custom-FC flight broadcasts golden-compatible packets to Base and four Away receivers; independent impairments and stale-pointing behavior are causal and repeatable; ordinary multibranch playback has explicit vehicle bindings. |
| **D. Custom FC and application connection** | FC encoder/decoder capabilities, station manifest, software serial, explicit simulation UI mode and known-command uplink. | Missing/unsupported formats are handled by the selected policy; byte paths preserve capture and parsing; physical-role restrictions and legacy standalone behavior remain intact. |
| **E. Static video transport** | General stream registry, Balius/Iris presets, receiver matrix, PNG source, simulation publisher/wall and reception/recording metadata. | Both vehicle layouts and a custom second digital stream work; GS4 mappings are correct; labels survive preview/main encoding; staleness, capacity and reconnect tests pass. |
| **F. Ensembles and portable results** | Independent member processes/state, one selected interactive member, bounded result storage, lazy mission-bundle readers and compatibility exports. | Same inputs/worker-independent seeds reproduce canonical results; complete and partial bundles reopen in both applications; encoded video is optional and slate duplication is avoided. |
| **G. Packaged integration** | macOS/Windows and available Linux distribution integration, bundled Python worker/runtime, documented compatibility/model limits. | Run the standard and custom-FC fixtures through packaged applications with software endpoints, no developer environment, complete bundle verification, and clean process/resource shutdown. |

Deliver A and B before treating the prototype's fields as production configuration. Deliver C as the first complete headless mission, then connect D and E; integrate F incrementally as streams are added rather than leaving persistence to the end. G qualifies the packaged paths. The additional gates in document 17 are part of this definition of done. Every package reports actual implementation and tests separately from this specification.

The feature is complete when a user can load the URRG configuration, enter the missing elevations, choose a rocket and either standard telemetry or a compatible FC, configure up to four pointers and arbitrary supported video counts, connect selected simulated stations, execute a single run or ensemble, and reopen the complete result without the original machine paths. Both applications must agree on identities, coordinates, clocks, packet/video provenance and completion status.

Physical firmware enable/target-switch contracts, analog RF waveforms, surveyed heights, measured antenna patterns and qualified mount dynamics remain explicit later inputs. They do not block the software implementation above, and their absence must not be represented as completed hardware qualification.

## 12. Implementation source references

- OpenRocket: [SimulationOptions](../../clone/openrocket/core/src/main/java/info/openrocket/core/simulation/SimulationOptions.java), [conditions editor](../../clone/openrocket/swing/src/main/java/info/openrocket/swing/gui/simulation/SimulationConditionsPanel.java), [options editor](../../clone/openrocket/swing/src/main/java/info/openrocket/swing/gui/simulation/SimulationOptionsPanel.java), [ensemble settings editor](../../clone/openrocket/swing/src/main/java/info/openrocket/swing/gui/simulation/EnsembleOptionsPanel.java), [FC definition](../../clone/openrocket/core/src/main/java/info/openrocket/core/simulation/flightcomputer/FlightComputerDesign.java), [legacy packet engine](../../clone/openrocket/core/src/main/java/edu/mit/rocket_team/telemetry/RTTelemetryEngine.java).
- Telemetry UI: [station profiles](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/station_profile.py), [media sources](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/media.py), [network media](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/network/media.py), [wall receiver](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/network/media_receiver.py), [video wall](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/video_wall.py), [synthetic source fixture](../../../RT_Telemetry_UI/scripts/network_pair_check.py), [CLI](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/__main__.py), [archive exporter](../../../RT_Telemetry_UI/src/rocket_gnc_monitor/flight.py), [wire protocol](../../../RT_Telemetry_UI/docs/PROTOCOL.md).

The source findings and video assignments were reviewed with the Telemetry UI chat against its existing working tree at commit `7b50315`. The recommendations above are proposed changes, not evidence that custom stream counts, FC codecs or launch manifests already exist.
