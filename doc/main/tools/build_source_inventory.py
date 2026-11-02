#!/usr/bin/env python3
"""Rebuild the guide's package coverage table and per-file navigation inventory."""
from pathlib import Path
from collections import Counter, defaultdict
import csv
import re

DOC = Path(__file__).resolve().parents[1]
ROOT = DOC.parents[1] / 'clone/openrocket'
# Longest matching prefix wins; all main Java files must have an explicit owner.
GROUPS = {
'core': [
('module-info.java','Module declaration','runtime','Exports and dependency requirements for the core module.'),
('de/congrace/exp4j','Expression parser','results','Bundled arithmetic parser, tokens, functions, variables and expression evaluation.'),
('edu/mit/rocket_team','Translated flight computer','fc','AV/FC state, control and airbrake algorithms, instruments, per-run internals, telemetry, data and hardware abstractions.'),
('info/openrocket/core/aerodynamics','Aerodynamics','aero','Calculator contracts, conditions and coefficients; Barrowman assembly and specialized body, fin, tube, rail-button and airbrake calculators.'),
('info/openrocket/core/appearance','Appearance','gui','Colors, decals and default visual appearances, independent of physical surface finish.'),
('info/openrocket/core/arch','Platform information','runtime','Operating-system/platform identification for application behavior and resource selection.'),
('info/openrocket/core/communication','Application communication','build','Connection abstraction, welcome/release retrieval, bug reports, upstream and MIT update metadata.'),
('info/openrocket/core/componentanalysis','Static analysis','workflows','Analysis parameters, sweep domains, per-component coefficient data and static parameter sweeps.'),
('info/openrocket/core/database','Databases','persistence','Async resource loading, catalog databases, motor lookup and grouped thrust-curve records.'),
('info/openrocket/core/document','Documents','runtime','Design/simulation ownership, storage options, undo, document events and attachment references.'),
('info/openrocket/core/file/openrocket','Native ORK and vanilla export','persistence','Document XML, component savers, import handlers, result/extension persistence and MIT-to-vanilla conversion.'),
('info/openrocket/core/file/rocksim','RockSim conversion','persistence','Format constants and mappings, XML import handlers, export DTOs and saver.'),
('info/openrocket/core/file/rasaero','RASAero conversion','persistence','Design, launch, recovery and simulation conversion through import handlers and export DTOs.'),
('info/openrocket/core/file/wavefrontobj','OBJ mesh export','persistence','Mesh tuples/faces/groups/materials, triangulation, component exporters and cylinder/tube/disk/polygon builders.'),
('info/openrocket/core/file/svg','SVG utility','persistence','Vector document/path construction used by fin-and-tab SVG export.'),
('info/openrocket/core/file/simplesax','XML parser delegation','persistence','Element handlers, reader pooling and warning-aware SAX delegation.'),
('info/openrocket/core/file/motor','Motor-file loading','persistence','RASP, RockSim and ZIP/general dispatch into thrust-curve records.'),
('info/openrocket/core/file/iterator','File iteration','persistence','Directory/JAR resource iteration used by loaders.'),
('info/openrocket/core/file','General file services','persistence','Format dispatch, load/save context, attachment handling, CSV export and shared file contracts.'),
('info/openrocket/core/formatting','Descriptions','extensions','Rocket/configuration descriptions and motor-configuration name substitutions.'),
('info/openrocket/core/gui/util','Shared file filter','extensions','File-filter adapter used by desktop and file-selection code.'),
('info/openrocket/core/l10n','Localization','runtime','Translator interfaces, contextual/debug/resource translators and locale helpers.'),
('info/openrocket/core/logging','Structured diagnostics','extensions','Warnings, errors, messages, priorities, abort information and message sets.'),
('info/openrocket/core/masscalc','Mass and inertia','models','Component/motor aggregation, rigid-body properties and per-component mass-analysis entries.'),
('info/openrocket/core/material','Materials','models','Bulk, sheet and line-density materials, groups and storage.'),
('info/openrocket/core/models/atmosphere','Atmosphere','models','Atmospheric conditions, extended ISA and interpolation/model interfaces.'),
('info/openrocket/core/models/gravity','Gravity','models','Gravity interface and WGS-based latitude/altitude model.'),
('info/openrocket/core/models/wind','Wind','models','Wind interfaces, average/pink-noise and multilevel wind models.'),
('info/openrocket/core/motor','Propulsion','models','Motor records, manufacturers/digests, installed configuration, ignition rules and runtime cluster state.'),
('info/openrocket/core/optimization/general','Numerical search','workflows','Function/point contracts, caches/controllers, golden-section and multidirectional search.'),
('info/openrocket/core/optimization/rocketoptimization','Rocket optimization','workflows','Physical objectives, goals, domains, modifiers and function adapter for trial designs.'),
('info/openrocket/core/optimization/services','Optimization discovery','workflows','Services exposing available objectives and modifiable design/configuration variables.'),
('info/openrocket/core/plugin','Plugin discovery','extensions','Annotations, class discovery and dependency-injection bindings.'),
('info/openrocket/core/preferences','Preferences','runtime','Core-facing application/document preference models and defaults.'),
('info/openrocket/core/preset','Component catalogs','persistence','Typed preset properties, material matching, CSV ingestion and XML DTOs/loaders.'),
('info/openrocket/core/rocketcomponent','Vehicle model','components','All native parts and assemblies, placement strategies, instances, configuration parameters and change propagation.'),
('info/openrocket/core/rocketvisitors','Tree traversal','extensions','Breadth/depth visitors and component/motor-mount listing.'),
('info/openrocket/core/scripting','Script engines','extensions','Engine discovery and GraalJS integration for scripting support.'),
('info/openrocket/core/simulation/customexpression','Derived channels','results','Expression definitions, ranges/functions and a result-writing listener.'),
('info/openrocket/core/simulation/ensemble','Ensembles','workflows','Settings, random draws, isolated execution, perturbations, aggregate statistics and member archives.'),
('info/openrocket/core/simulation/exception','Run exceptions','engine','Calculation, listener/termination and other simulation exception contracts.'),
('info/openrocket/core/simulation/extension','Simulation features','extensions','Persistent extension/provider contracts, implementations and inherited examples.'),
('info/openrocket/core/simulation/flightcomputer','FC host integration','fc','Runtime listener, sensors, scheduler/control bridges, Java programs, design library, recovery and logs.'),
('info/openrocket/core/simulation/listeners','Simulation callbacks','extensions','Computation/event/step interfaces, dispatch helpers, controllers, examples and system stopping/coast listeners.'),
('info/openrocket/core/simulation/motorbatch','Motor comparisons','workflows','Candidate selection, fit/clearance, isolated cases, metrics, report history, CSV and persistence.'),
('info/openrocket/core/simulation','Flight execution and data','engine','Options/conditions/status, event engines, RK/Euler/mode steppers, events and typed flight histories.'),
('info/openrocket/core/startup','Core application services','runtime','Shared application service access and exception handling.'),
('info/openrocket/core/thrustcurve','Online motor data','persistence','API request/response objects, XML parsing, curve decoding/downloads and serialization helpers.'),
('info/openrocket/core/unit','Units','extensions','Conversion, unit groups, values, comparison, fractional formatting and axis ticks.'),
('info/openrocket/core/util','Shared utilities','extensions','Coordinates/transforms, numerics, interpolation, noise, statistics, identity, listeners, mutability and resource helpers.'),
('info/openrocket/core/utils','Developer utilities','extensions','Localization generation/reporting, motor comparison and optimizer exercise programs.'),
],
'swing': [
('module-info.java','Module declaration','runtime','Exports and dependencies for the desktop module.'),
('info/openrocket/swing/communication','Desktop communication','build','Maps release assets to operating platforms and selects installer download links.'),
('info/openrocket/swing/file','Desktop file adapters','persistence','Simulation-table CSV, motor-loading helper, Photo Studio settings adapters and OBJ option chooser.'),
('info/openrocket/swing/gui/adaptors','Property bindings','gui','Boolean/numeric/enum and related Swing models connected to core properties and units.'),
('info/openrocket/swing/gui/components','Shared controls','gui','Reusable buttons, fields, selectors, tables and compass controls.'),
('info/openrocket/swing/gui/configdialog','Component editors','gui','Specialized part dialogs plus appearance, placement, instances, material and override controls.'),
('info/openrocket/swing/gui/customexpression','Expression editor','results','Expression names, symbols, units, functions, editing and validation UI.'),
('info/openrocket/swing/gui/dialogs/componentanalysis','Analysis UI','workflows','Component coefficients, selections, sweep configuration, plotting and export.'),
('info/openrocket/swing/gui/dialogs/flightconfiguration','Configuration dialogs','gui','Motor-mount, ignition, separation, deployment and naming controls.'),
('info/openrocket/swing/gui/dialogs/motor','Motor selection','gui','Motor information/filtering and thrust-curve selection, comparison, plotting and download UI.'),
('info/openrocket/swing/gui/dialogs/optimization','Optimization UI','workflows','Variable/goal selection, background search, progress and plotted history.'),
('info/openrocket/swing/gui/dialogs/preferences','Preferences UI','runtime','Application defaults, units, materials, launch, simulation, display and graphics choices.'),
('info/openrocket/swing/gui/dialogs/preset','Preset dialogs','persistence','Catalog selection, editing and associated model/dialog helpers.'),
('info/openrocket/swing/gui/dialogs','Application dialogs','gui','Shared desktop dialogs including scaling, print setup, warnings and application interactions.'),
('info/openrocket/swing/gui/figure3d/photo','Photo Studio','gui','Posed rocket scene, camera/light settings, exhaust rendering and sky/background implementations.'),
('info/openrocket/swing/gui/figure3d','3D design renderer','gui','JOGL view/render orchestration and component geometry/material rendering.'),
('info/openrocket/swing/gui/figureelements','Figure overlays','gui','Additional figure annotations and derived-property markers.'),
('info/openrocket/swing/gui/flightcomputer','FC design editor','fc','Board/program/controller design, assignments, settings and diagnostics.'),
('info/openrocket/swing/gui/help/tours','Guided help','gui','Tour data loading, panels and guided-help presentation.'),
('info/openrocket/swing/gui/main','Document windows','gui','Main panels/actions, component tree, selection/drag operations and flight-configuration tables.'),
('info/openrocket/swing/gui/plot','Results and playback','results','Plot definitions/axes/events, simulation charts, trajectory adaptation and 3D playback.'),
('info/openrocket/swing/gui/preset','Preset widgets','persistence','Catalog tables, sorting/filtering and preset result interactions.'),
('info/openrocket/swing/gui/print','Printing','results','Reports/templates, paper/layout controls, selection tree and component visitor strategies.'),
('info/openrocket/swing/gui/rocketfigure','2D component shapes','gui','Side/back-view shape providers for supported components and fin-profile drawing.'),
('info/openrocket/swing/gui/scalefigure','Scaled design view','gui','Zoomable figures, coordinate mapping, rulers and rocket-view integration.'),
('info/openrocket/swing/gui/simulation','Simulation UI','workflows','Run/configuration dialogs, workers, launch/wind settings and motor-batch workflows.'),
('info/openrocket/swing/gui/theme','Themes','gui','Application theme definitions and selection behavior.'),
('info/openrocket/swing/gui/util','Desktop utilities','gui','Icons, platform/window/model helpers, validation, update installation and related shared operations.'),
('info/openrocket/swing/gui/watcher','External-file watching','gui','Watch services, events and reactions for desktop files/assets.'),
('info/openrocket/swing/gui/widgets','Additional widgets','gui','Reusable text/table/selection and presentation widgets.'),
('info/openrocket/swing/gui','GUI service boundary','gui','Top-level desktop service/GUI support classes.'),
('info/openrocket/swing/logging','Desktop logs','extensions','Logging backend integration, buffers, filters and diagnostic display support.'),
('info/openrocket/swing/simulation/extension','Extension configuration UI','extensions','Configurator contracts, examples and editors for core simulation extensions.'),
('info/openrocket/swing/startup','Desktop bootstrap','runtime','Entry/startup classes, jar-in-jar loading and injected service providers.'),
('info/openrocket/swing/utils','Desktop/developer utilities','extensions','Standalone application setup, preset tools/converters, scripting and small formatting/input helpers.'),
]}

rows=[]
group_files=defaultdict(list)
for module in ('core','swing'):
    base=ROOT/module/'src/main/java'
    for path in sorted(base.rglob('*.java')):
        relative=path.relative_to(base).as_posix()
        matches=[g for g in GROUPS[module] if relative==g[0] or relative.startswith(g[0]+'/')]
        if not matches:
            raise ValueError(f'No documented owner: {module}/{relative}')
        group=max(matches,key=lambda g:len(g[0]))
        source=path.read_text()
        match=re.search(r'^\s*package\s+([\w.]+)\s*;',source,re.M)
        package=match.group(1) if match else '(module descriptor)'
        rows.append([module,package,path.name,group[1],group[2],path.relative_to(ROOT).as_posix()])
        group_files[(module,group[0])].append(path)
with (DOC/'source-inventory.tsv').open('w',newline='') as f:
    w=csv.writer(f,delimiter='\t',lineterminator='\n')
    w.writerow(['module','package','source_file','subsystem','chapter_label','path_from_openrocket'])
    w.writerows(rows)

parts=[r'\chapter{Whole-codebase package atlas}\label{app:packages}',
r'''This appendix accounts for every Java source under the application's two \code{src/main/java} trees at the documented snapshot. It groups related packages by function and links them to the explanatory chapters. Counts are source files, not public classes, methods, lines of code, or a measure of subsystem importance. Nested types remain within their source file. The two module descriptors are counted separately from named packages.

The inventory contains \textbf{1,282 main Java files}: 826 in core and 456 in Swing, spanning 139 named packages plus the two module descriptors. The bundled expression parser and translated MIT firmware are included. \code{source-inventory.tsv}, beside the main document, lists every file, its package, subsystem, chapter label and source-relative path. It can be filtered to locate a package not printed in full below.

\code{tools/build_source_inventory.py} regenerates both this table and the per-file inventory from the local source tree. It requires every file to match an explicit documented subsystem and rejects uncovered paths. This is a coverage/navigation check, not a claim that every method has been individually explained or numerically validated. The prose describes subsystem behavior and the important interactions, with deeper treatment of geometry, forces and flight/control timing.
''']
for module in ('core','swing'):
    parts.append(r'\section{'+('Core' if module=='core' else 'Swing')+' module}')
    parts.append(r'Package paths below are relative to \code{'+module+r'/src/main/java/}. Except where written in full, the prefix is \code{info/openrocket/'+module+r'/}. Each row includes its descendant packages unless a more specific row is listed.')
    parts.append(r'\begin{longtable}{@{}L{4.6cm}L{0.8cm}L{10.2cm}@{}}\toprule Package family & Files & Function and guide location\\\midrule\endfirsthead')
    parts.append(r'\toprule Package family & Files & Function and guide location\\\midrule\endhead')
    for prefix,title,label,desc in GROUPS[module]:
        files=group_files[(module,prefix)]
        assert files, f'Empty coverage group: {module}/{prefix}'
        short=prefix.removeprefix('info/openrocket/'+module+'/')
        parts.append(r'\code{'+short+r'} & '+str(len(files))+r' & \textbf{'+title+'}. '+desc+r' Chapter~\ref{ch:'+label+r'}.\\')
    parts.append(r'\bottomrule\end{longtable}')
parts.append(r'''\section{Resources, tests, and non-runtime source}
The main resource trees contain 107 core files and 301 Swing files in this snapshot. Core resources include build metadata, translations and data catalogs, serialized thrust curves, catalog CSV inputs, and example ORK designs. Swing resources include service registrations, icons, fonts, themes, textures, sky/exhaust assets, a graphics shader, help content and the Windows update helper. These files can affect runtime behavior or presentation despite not appearing in the Java count.

The test trees contain 169 core Java files and 34 Swing Java files, with 30 and 3 test-resource files respectively. They contain test cases and shared fixtures/helpers, not 203 independently counted test methods. The tests cover selected contracts across models, files, simulations and UI support; the mere existence of a test tree does not establish coverage of every formula or platform. Historical test results are identified separately in the FC and build chapters.

The source-root build files, Gradle wrapper/configuration, module builds and documentation tasks select dependencies, compilation, testing and packaging. \code{tools/documentation} contains source-inventory/documentation tooling. \code{fileformat.txt}, \code{docs/}, \code{doc/}, example/test-writing designs, developer instructions and release material explain interfaces and workflows outside executable classes. The inherited \code{install4j/}, \code{snap/} and workflow files reflect upstream distribution arrangements; the parent repository's \code{packaging/} recipes implement the MIT packages described in Chapter~\ref{ch:build}.

The parent repository also contains firmware-related work and development documentation outside the OpenRocket application checkout. Those are context and integration inputs; this guide's exhaustive Java inventory is specifically the OpenRocket core/Swing codebase plus the firmware translation embedded within core. Generated binaries, IDE metadata and crash logs are not additional application subsystems.
''')
(DOC/'sections/16-package-atlas.tex').write_text('\n\n'.join(parts).rstrip()+'\n')
print(f'Inventoried {len(rows)} Java files; {len(set((r[0],r[1]) for r in rows if r[1]!="(module descriptor)"))} named packages; {len(group_files)} subsystem groups.')
