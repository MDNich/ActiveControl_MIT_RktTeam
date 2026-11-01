**OpenRocket MIT 6.5 — prerelease. Mission Simulation and flight-computer simulation are beta features.**

## Mission Simulation — Beta

Configure and run a complete software mission from **File → New Mission Simulation (Beta)…** in the native Java application.

- Load saved configurations or portable setups; new missions select the current rocket by default.
- Place the launch site, base station and up to four ground antenna pointers using a map, latitude/longitude or MGRS. Defaults are near URRG, with launch at `18TUN2061530290`; editable ground heights default to 0 m.
- Retain ordinary OpenRocket rocket, motor, staging, recovery, weather and flight-computer settings. Run ensembles or ground-link experiments from saved flights.
- Model telemetry loss, delay, corruption and sensor noise independently for each receiver. Use the legacy Zephyrus format without a custom FC, or the telemetry definition supplied by a simulated FC.
- Advertise simulated Away telemetry/video stations on the local network. The GNC/Telemetry UI can discover them in Base mode without an application path; join the same Mission UUID and choose a network telemetry source. Manual connection addresses must include the advertised port.
- Generate labelled analog and digital video slates, with mission/run/rocket metadata. These are simulated test images, not rendered flight-camera footage.
- Record flight-computer logs, raw packets, receiver/pointer histories, configuration and video assets together in a recoverable `.rktmission` ZIP bundle. Inspect, export and replay results or move a portable setup to another computer.

Standard telemetry publishes Zephyrus states, including PRE_FLIGHT, FLIGHT, APOGEE, MAIN at actual recovery deployment and END at ground contact. Deployment delays are respected; a rejected deployment does not report a parachute. Custom flight computers retain their own transmitted state. Packet-associated simulation attitude and recovery indications support the receiving UI without replacing FC measurements or pointing inputs.

## Fixes and integration

- Native Mission configuration replaces the earlier browser interface.
- Fix Mission creation and packaged Java/Python startup; keep signed application bundles unchanged during execution.
- Preserve advertised station endpoints when starting a configured mission and support simultaneous video viewers.
- Fix recovery-state publication and replay of missions deploying recovery before apogee.
- Restore the Mac application icon and suppress the upstream welcome dialog at startup.

## Downloads

- **macOS:** separate Apple Silicon (`macOS-arm64`) and Intel (`macOS-x86_64`) DMG/ZIP downloads. Drag `OpenRocket_MIT.app` into Applications.
- **Windows x64:** installer (`Windows-x64-Setup.exe`) or portable ZIP; keep the portable application folder together.
- **Linux x86_64:** Ubuntu 24.04 `.deb` or portable `.tar.gz`; launch `OpenRocket_MIT/bin/OpenRocket_MIT` after extracting the portable archive. A graphical desktop and system graphics/font libraries are required.
- **Standalone JAR / existing updater:** `OpenRocket-MIT-v6.5.jar` requires Java 17. For Mission Simulation, install a full 6.5 platform package: a JAR-only update from 6.4 does not install its private Python and network/video libraries.

Full platform packages include Java 17 and the private Mission Python/network runtime. Per-platform manifests and SHA-256 checksums accompany the downloads. The Mac apps are ad-hoc signed and not Apple-notarized; the Windows installer is unsigned.

## Beta scope and verification

Mission Simulation supports repeatable software integration experiments. RF propagation, antenna mechanics, sensors and video impairments are simplified models; map imagery does not determine ground elevation. Results do not qualify real flight hardware or firmware. Use a current GNC/Telemetry UI build with Mission discovery and Zephyrus state-schema support.

Release packages are built from one source revision on native Apple Silicon, Intel Mac, Windows x64 and Ubuntu x86_64 runners. Each runs the Mission regression suite plus standard, custom-FC and recovery fixtures using the packaged Java/Python runtimes, checking all four station telemetry endpoints, original packet state, recovery flags, archive integrity and recorded-packet replay. Both Mac packages additionally run the actual launcher and native Mission configuration workflow. The Windows installer UI and Linux desktop session are not interactively exercised by this release workflow.

[Mission user guide](https://github.com/MDNich/ActiveControl_MIT_RktTeam/blob/v6.5/doc/improvements/19-mission-simulation-user-guide.md) · [Changes since 6.4](https://github.com/MDNich/ActiveControl_MIT_RktTeam/compare/v6.4...v6.5)
