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

## Remaining work

**A2:** complete flight-setting inventory and snapshot round trips, preview migration with field-level unresolved findings, `.fc` v3 codec declaration/migration and independent custom-codec vectors, production coordinate conversion/geoid asset handling, and supported-command capability declarations. The current `.fc` v1/v2 reader remains unchanged in A1; it does not yet claim v3 execution support.

**B:** Java-owned configuration service, full setting editors, menu/workspace integration and revision/idempotency behavior. The browser in `mission-ui/` is still the design prototype, not a connected mission runner.

**C–G:** deterministic station worker/scheduler and actual simulation, FC/software-station integration, media transmission, ensembles/bundles and packaged qualification remain as specified. No installer or GitHub release should describe Mission Simulation as delivered on the basis of A1 alone.
