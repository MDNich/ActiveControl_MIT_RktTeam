# Mission configuration design prototype

Open `index.html` through a local web server for live map tiles. From this directory:

```sh
python3 -m http.server 8879 --bind 127.0.0.1
```

Then open `http://127.0.0.1:8879` in a browser. Leaflet 1.9.4 and mgrs 2.1.0 load from jsDelivr. USGS aerial/topographic imagery is the initial layer; Esri imagery and OpenStreetMap streets are also available. The provider attribution stays on the map. Only the viewed tiles are requested; no tile archive is created.

The prototype demonstrates six configuration screens. You can edit coordinates, drag markers, convert MGRS, select 0–4 pointers, retain receiver-only sites, choose standard/custom FC modes, adjust link settings, select Balius/Iris video presets or custom counts, assign multiple receivers per stream, export a static PNG slate, edit Telemetry UI paths, and configure ensemble fields. **Save configuration** exports a JSON draft; **Load configuration** restores it.

Use [the example Balius configuration](example-balius.rktmission-config.json) to try the preload flow. It restores URRG with four pointers, the separate Base station, one digital stream and one analog stream.

This is a design preview. Rocket/FC file pickers retain filenames but do not parse or execute those files. It starts no simulator, Telemetry UI, serial device, camera or video transmission. Input fields representing structured flight/FC assignments are design placeholders; the [specification](../16-mission-configuration-ui.md) defines their required full-model integration and validation. The configuration schema is preview-specific and has not been installed in either application.

The [final implementation contracts](../17-mission-simulation-contracts.md), reviewed 28 October 2026, take precedence over illustrative preview behavior. Production manual MGRS input uses the grid reference point, initial link/noise settings use an ideal zero-impairment profile, and receiver-only sites use the existing four Away identities. Preview imports must preserve the existing coordinate convention and flag incomplete structured settings; changing the JSON format name is not a production migration. This review updates the specification, not the interactive prototype's implementation.

The default launch is `18TUN2061530290`. Base and Away 1–4 come from Telemetry UI's `site_presets.py`, decoded with its `decode_mgrs()` function. Exact MGRS strings and grid-reference-point coordinates are retained; [urrg-sites.json](urrg-sites.json) records all six decoded reference points and hashes of their source files. Reapplying an unchanged preset does not shift it to a cell center. New manual MGRS entries use the browser converter's labeled cell-center convention; latitude/longitude remains available outside its supported UTM region. Elevations start unset because the presets provide no heights. Imagery does not provide terrain elevation or validate a launch site. Other flight/link/mount values remain demonstration inputs. Receiver-only sites are configured under Connections.

`site-preview.png` and `video-preview.png` capture the design. `iris-analog-2-example.png` is a sample generated stream slate. The screenshots include imagery attribution; slates are generated locally from text and a grid.

Browser verification covered imagery loading, pointer counts, coordinate/MGRS round trips and invalid input, custom FC controls, Iris assignments, custom stream counts, PNG export, ensemble selection, configuration save/load, invalid imports, and all six screens at desktop and narrow widths. No backend integration tests or hardware tests are implied.
