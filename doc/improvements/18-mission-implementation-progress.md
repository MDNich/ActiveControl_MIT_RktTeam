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

## Remaining work

**B:** Java-owned configuration service, full setting editors, menu/workspace integration and revision/idempotency behavior. The browser in `mission-ui/` is still the design prototype, not a connected mission runner.

**C–G:** deterministic station worker/scheduler and actual simulation, FC/software-station integration, media transmission, ensembles/bundles and packaged qualification remain as specified. No installer or GitHub release should describe Mission Simulation as delivered on the basis of A1 alone.
