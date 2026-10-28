package info.openrocket.core.simulation.mission;

import info.openrocket.core.document.Simulation;
import info.openrocket.core.motor.Motor;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import jakarta.json.*;
import java.io.*;
import java.nio.file.*;
import java.security.*;
import java.util.*;
import java.util.function.Function;

/** Java-owned draft revisions and content-addressed flight inputs. No process/device opens on load. */
public final class MissionStore {
    public record Resolved(JsonObject mission, Path flightDirectory, String hash, Path recordedSource) {
        public Resolved(JsonObject mission,Path flightDirectory,String hash){this(mission,flightDirectory,hash,null);}
    }
    public static final class Conflict extends IllegalArgumentException {
        public Conflict(String message) { super(message); }
    }
    private final Path root;
    private final Map<String, MissionDefinition> drafts = new LinkedHashMap<>();
    private final Function<String, Motor> motorLookup;

    public MissionStore(Path root, Function<String, Motor> motorLookup) throws IOException {
        this.root = root.toAbsolutePath(); this.motorLookup = motorLookup;
        Files.createDirectories(this.root.resolve("sources")); Files.createDirectories(this.root.resolve("snapshots")); Files.createDirectories(this.root.resolve("drafts"));
    }
    public Path root() { return root; }
    public synchronized JsonObject create(String title) throws IOException { return load(MissionDefinition.createUrrg(title)); }
    public synchronized JsonObject load(MissionDefinition definition) throws IOException {
        String id = id(definition.json());
        // Explicit loading preserves the mission identity. Advance the local revision to invalidate stale edits.
        if (drafts.containsKey(id)) {
            int revision = Math.max(definition.json().getJsonObject("identity").getInt("revision"),
                    Math.addExact(get(id).getJsonObject("identity").getInt("revision"), 1));
            definition = new MissionDefinition(Json.createObjectBuilder(definition.json()).add("identity",
                    Json.createObjectBuilder(definition.json().getJsonObject("identity")).add("revision", revision)).build());
        }
        definition.write(root.resolve("drafts").resolve(id + ".json")); drafts.put(id, definition);
        return definition.json();
    }
    public synchronized JsonObject get(String id) {
        var value = drafts.get(id); if (value == null) throw new IllegalArgumentException("Unknown draft"); return value.json();
    }
    public synchronized JsonObject update(String id, int expectedRevision, JsonObject value) throws IOException {
        JsonObject previous = get(id);
        if (previous.getJsonObject("identity").getInt("revision") != expectedRevision) throw new Conflict("The draft changed; reload it before saving");
        if (!id(value).equals(id)) throw new IllegalArgumentException("Draft identity cannot change");
        var identity = Json.createObjectBuilder(value.getJsonObject("identity")).add("revision", Math.addExact(expectedRevision, 1));
        var next = new MissionDefinition(Json.createObjectBuilder(value).add("identity", identity).build());
        next.write(root.resolve("drafts").resolve(id + ".json")); drafts.put(id, next); return next.json();
    }
    public synchronized void save(String id, Path file) throws IOException { new MissionDefinition(get(id)).write(file); }

    public synchronized void exportSetup(String id, Path destination) throws IOException {
        var draft = get(id); var files = new TreeMap<String, Path>();
        Path mission = Files.createTempFile(root, "setup-", ".json");
        try {
            MissionJson.writeAtomic(mission, draft); files.put("mission.json", mission);
            String hash = draft.getJsonObject("flight").getString("snapshotHash", "");
            if (!hash.isEmpty()) {
                Path folder = snapshot(hash); MissionFlightSnapshot.restore(folder);
                try (var inputs = Files.list(folder)) {
                    for (var file : inputs.toList()) files.put("assets/" + hash + "/" + file.getFileName(), file);
                }
            }
            if(draft.containsKey("recordedFlight")) {
                String source=draft.getJsonObject("recordedFlight").getString("sourceHash");files.put("sources/"+source+".rktmission",recordedSource(draft));
            }
            MissionArchive.write(destination, MissionArchive.SETUP, "complete", files);
        } finally { Files.deleteIfExists(mission); }
    }

    public synchronized JsonObject importSetup(Path file) throws IOException {
        Path imported = root.resolve("import-" + UUID.randomUUID());
        try (var archive = new MissionArchive(file)) {
            if (!archive.manifest().getString("format").equals(MissionArchive.SETUP)) throw new IOException("Choose a setup archive; result archives open in replay");
            var mission = new MissionDefinition(MissionJson.parse(archive.read("mission.json", MissionJson.MAX_BYTES)));
            String hash = mission.json().getJsonObject("flight").getString("snapshotHash", "");
            String source=mission.json().containsKey("recordedFlight")?mission.json().getJsonObject("recordedFlight").getString("sourceHash"):"";
            for (String name : archive.names()) if (!name.equals("mission.json") && !name.startsWith("assets/" + hash + "/") && !name.equals("sources/"+source+".rktmission"))
                throw new IOException("Unsupported setup asset group");
            archive.extract(imported);
            if (!hash.isEmpty()) {
                if (!hash.matches("[0-9a-f]{64}")) throw new IOException("Invalid input snapshot identity");
                Path inputs = imported.resolve("assets").resolve(hash);
                if (!MissionFlightSnapshot.hash(inputs.resolve("manifest.json")).equals(hash)) throw new IOException("Setup snapshot identity mismatch");
                MissionFlightSnapshot.restore(inputs);
                Path existing = root.resolve("snapshots").resolve(hash);
                if (Files.exists(existing)) MissionFlightSnapshot.restore(snapshot(hash));
                else Files.move(inputs, existing, StandardCopyOption.ATOMIC_MOVE);
            }
            if(!source.isEmpty()) {
                if(!source.matches("[0-9a-f]{64}"))throw new IOException("Invalid recorded source hash");
                Path selected=imported.resolve("sources").resolve(source+".rktmission");
                if(!MissionFlightSnapshot.hash(selected).equals(source))throw new IOException("Recorded source hash mismatch");
                Files.move(selected,root.resolve("sources").resolve(source+".rktmission"),StandardCopyOption.REPLACE_EXISTING);
            }
            return load(mission);
        } finally {
            if (Files.exists(imported)) try (var paths = Files.walk(imported)) {
                for (Path path : paths.sorted(Comparator.reverseOrder()).toList()) Files.delete(path);
            }
        }
    }

    public synchronized JsonObject bindFlight(String id, int revision, Simulation source, String filename) throws IOException {
        var draft = get(id); checkRevision(draft, revision);
        String hash = freeze(source);
        var flight = Json.createObjectBuilder(draft.getJsonObject("flight")).add("snapshotHash", hash)
                .add("filename", filename).add("simulationName", source.getName()).add("editor", MissionFlightEditor.capture(source));
        var assets = Json.createArrayBuilder();
        for (var value : draft.getJsonArray("assets")) if (!value.asJsonObject().getString("id").equals("flight-inputs")) assets.add(value);
        assets.add(Json.createObjectBuilder().add("id", "flight-inputs").add("kind", "rocket").add("reference", "snapshot:" + hash).add("sha256", hash));
        return update(id, revision, Json.createObjectBuilder(draft).remove("recordedFlight").add("flight", flight).add("assets", assets).build());
    }

    /** Isolated native simulation for edit actions. A failed edit is discarded, never partly published. */
    public synchronized Simulation editableFlight(String id) throws IOException {
        var flight = get(id).getJsonObject("flight");
        var simulation = MissionFlightSnapshot.restore(snapshot(flight.getString("snapshotHash", "")));
        MissionFlightEditor.apply(simulation, flight.getJsonObject("editor"), motorLookup); return simulation;
    }

    public synchronized JsonArray validate(String id) {
        var draft = get(id); var issues = Json.createArrayBuilder();
        if (draft.containsKey("requiredCapabilities")) {
            for (var capability : draft.getJsonArray("requiredCapabilities"))
                issue(issues, "/requiredCapabilities", "unsupported-capability", "Required capability is not advertised: " + capability);
        }
        if (draft.containsKey("migration") && !draft.getJsonObject("migration").getBoolean("reviewed", false))
            issue(issues, "/migration", "preview-review-required", "Review the imported preview findings and confirm the production settings");
        try {
            var execution = draft.getJsonObject("execution");
            if (execution.containsKey("actions")) {
                if (execution.getJsonArray("actions").size() > 10000) throw new IllegalArgumentException("At most 10,000 scenario actions");
                var times = new HashMap<String,Integer>();var actionIds=new HashSet<String>();
                for (var value : execution.getJsonArray("actions")) {
                    var action = value.asJsonObject(); MissionActions.validate(action);
                    if(!actionIds.add(action.getString("id")))throw new IllegalArgumentException("Each scenario action needs a unique ID");
                    int count = times.merge(action.getString("timeUs"), 1, Integer::sum);
                    if (count > 200) throw new IllegalArgumentException("At most 200 actions at one mission instant");
                }
            }
        } catch (RuntimeException e) { issue(issues,"/execution/actions","invalid-action",safeMessage(e)); }
        int index = 0;
        for (var entry : draft.getJsonArray("sites")) {
            var site = entry.asJsonObject();
            if (site.getBoolean("enabled") && site.isNull("heightM"))
                issue(issues, "/sites/" + index + "/heightM", "height-required", "Enter a height and confirm MSL or WGS84 ellipsoid datum");
            index++;
        }
        try {
            var simulation = editableFlight(id);
            MissionJobs.admit(draft);
            if(draft.containsKey("recordedFlight")) {
                recordedSource(draft);var recorded=draft.getJsonObject("recordedFlight");
                if(!Set.of("recorded-tx","standard-generator").contains(recorded.getString("telemetrySource")) || !recorded.getString("member").matches("member-[0-9]{1,5}"))
                    throw new IllegalArgumentException("Invalid recorded flight selection");
                if(recorded.getString("telemetrySource").equals("standard-generator") && !draft.getJsonObject("telemetry").getString("mode").equals("standard"))
                    throw new IllegalArgumentException("This recorded trajectory uses the standard generator; derive another experiment to use original FC packets");
                if(!hash(draft.getJsonObject("flight").getJsonObject("editor")).equals(recorded.getString("editorHash"))
                        || !hash(MissionRecordedSource.launch(draft)).equals(recorded.getString("launchHash"))
                        || !hash(Json.createObjectBuilder().add("bindings",draft.getJsonArray("vehicleBindings")).build()).equals(recorded.getString("bindingsHash")))
                    throw new IllegalArgumentException("A recorded flight keeps its rocket, flight settings, launch and vehicle bindings fixed; edit the ground stations and links, or start a new flight configuration");
                if(recorded.getString("telemetrySource").equals("recorded-tx") && !hash(MissionRecordedSource.packetSettings(draft)).equals(recorded.getString("packetSettingsHash")))
                    throw new IllegalArgumentException("Recorded transmissions keep their original format and observations; derive another experiment with the standard generator to change them");
            }
            if (simulation.getActiveConfiguration().getActiveStages().isEmpty())
                issue(issues, "/flight/editor/stages", "no-active-stage", "Enable a rocket stage");
            if (simulation.getActiveConfiguration().getActiveMotors().isEmpty())
                issue(issues, "/flight/editor/motors", "no-active-motor", "Select a motor on an active stage");
            var active = simulation.getActiveConfiguration().getActiveStages();
            for (var entry : draft.getJsonArray("vehicleBindings")) {
                var binding = entry.asJsonObject(); String component = binding.getString("componentId", "");
                if (component.isEmpty() && active.size() != 1)
                    issue(issues, "/vehicleBindings", "stage-binding-required", "Bind each vehicle to an active rocket stage by component identity");
                else if (!component.isEmpty() && active.stream().noneMatch(stage -> stage.getID().toString().equals(component)))
                    issue(issues, "/vehicleBindings", "stage-binding-invalid", "The selected stage is missing or inactive");
            }
            var receiverIds = new HashSet<String>();
            for (var entry : draft.getJsonObject("telemetry").getJsonArray("links")) {
                var link = entry.asJsonObject(); MissionLink.validate(link);
                if (!receiverIds.add(link.getString("receiverId"))) issue(issues, "/telemetry/links", "duplicate-link", "Each receiver has one link profile");
            }
            if (!draft.containsKey("recordedFlight") && draft.getJsonObject("telemetry").getString("mode").equals("custom-fc")) {
                var computers = simulation.getSimulationExtensions().stream().filter(e -> e instanceof ZephyrusFlightComputer fc && fc.isEnabled()).toList();
                if (computers.size() != 1) issue(issues, "/flight/editor/extensions", "fc-count", "Enable exactly one flight computer for a coupled run");
                if (simulation.getActiveConfiguration().getActiveStages().size() != 1)
                    issue(issues, "/flight/editor/stages", "fc-multistage", "Custom FC missions require one active stage and pad startup");
                if (computers.size() == 1) {
                    var fc = (ZephyrusFlightComputer) computers.get(0);
                    if (fc.getMissionDesign() == null) issue(issues, "/flight/editor/extensions", "fc-input", "Choose a flight-computer file to freeze with the mission");
                    else {
                        fc.getMissionDesign().design().requireRunnable();
                        if (fc.getMissionDesign().design().telemetryCodec() == null && draft.getJsonObject("telemetry").getString("fallback").equals("require-format"))
                            issue(issues, "/telemetry/fallback", "codec-required", "This flight computer has no telemetry format declaration; choose legacy fallback or another FC");
                    }
                }
            }
        } catch (IOException | RuntimeException e) { issue(issues, "/flight", "flight-input", safeMessage(e)); }
        return issues.build();
    }

    public synchronized Resolved resolve(String id, int revision) throws IOException {
        var draft = get(id); checkRevision(draft, revision);
        JsonArray issues = validate(id); if (!issues.isEmpty()) throw new IllegalArgumentException(issues.toString());
        var sites = Json.createArrayBuilder(); JsonObject launch = null;
        for (var entry : draft.getJsonArray("sites")) {
            var site = entry.asJsonObject();
            if (site.isNull("heightM")) { sites.add(site); continue; }
            double n = MissionGeoid.bundled().separation(site.getJsonNumber("latitudeDeg").doubleValue(), site.getJsonNumber("longitudeDeg").doubleValue());
            double given = site.getJsonNumber("heightM").doubleValue();
            double msl = site.getString("heightDatum").equals("MSL") ? given : given - n;
            var resolved = Json.createObjectBuilder(site).add("mslHeightM", msl).add("ellipsoidalHeightM", msl + n).add("geoidSeparationM", n).build();
            if (site.getString("id").equals("launch-site")) launch = resolved; sites.add(resolved);
        }
        var simulation = editableFlight(id);
        simulation.getOptions().setLaunchLatitude(launch.getJsonNumber("latitudeDeg").doubleValue());
        simulation.getOptions().setLaunchLongitude(launch.getJsonNumber("longitudeDeg").doubleValue());
        simulation.getOptions().setLaunchAltitude(launch.getJsonNumber("mslHeightM").doubleValue());
        var bindings = Json.createArrayBuilder();
        for (var entry : draft.getJsonArray("vehicleBindings")) {
            var binding = entry.asJsonObject();
            String component = binding.getString("componentId", "");
            if (component.isEmpty()) component = simulation.getActiveConfiguration().getActiveStages().get(0).getID().toString();
            bindings.add(Json.createObjectBuilder(binding).add("componentId", component)
                    .add("flightConfigurationId", simulation.getId().toString()).add("rocketAssetId", "flight-inputs"));
        }
        String flightHash = freeze(simulation);
        var resolvedAssets = Json.createArrayBuilder();
        for (var entry : draft.getJsonArray("assets")) {
            var asset = entry.asJsonObject();
            resolvedAssets.add(asset.getString("id").equals("flight-inputs")
                    ? Json.createObjectBuilder(asset).add("reference", "snapshot:" + flightHash).add("sha256", flightHash).build() : asset);
        }
        var resolved = Json.createObjectBuilder(draft).add("assets", resolvedAssets).add("sites", sites).add("vehicleBindings", bindings).add("geoid", MissionGeoid.bundled().provenance())
                .add("flight", Json.createObjectBuilder(draft.getJsonObject("flight")).add("snapshotHash", flightHash)
                        .add("editor", MissionFlightEditor.capture(simulation))).build();
        return new Resolved(resolved, snapshot(flightHash), hash(resolved),draft.containsKey("recordedFlight")?recordedSource(draft):null);
    }

    public synchronized JsonObject deriveResult(Path file,String member,String telemetrySource) throws IOException {
        if(!member.matches("member-[0-9]{1,5}") || !Set.of("recorded-tx","standard-generator").contains(telemetrySource))throw new IllegalArgumentException("Select a member and an explicit telemetry source");
        Path temporary=Files.createTempDirectory(root,"derive-");
        try(var archive=new MissionArchive(file)) {
            var original=MissionJson.parse(archive.read("mission.json",MissionJson.MAX_BYTES));
            String flightHash=original.getJsonObject("flight").getString("snapshotHash");
            if(!flightHash.matches("[0-9a-f]{64}"))throw new IOException("Invalid recorded flight snapshot");
            String prefix="runs/"+member+"/";var files=new TreeMap<String,Path>();
            for(String name:archive.names())if(name.equals("mission.json") || name.startsWith("assets/"+flightHash+"/")
                    || name.startsWith(prefix+"truth/") || name.startsWith(prefix+"transmitters/") || name.startsWith(prefix+"events/") || name.equals(prefix+"run.json")) {
                Path output=temporary.resolve(name);Files.createDirectories(output.getParent());
                try(var in=archive.open(name)){Files.copy(in,output);}files.put(name,output);
            }
            Path inputs=temporary.resolve("assets").resolve(flightHash);MissionFlightSnapshot.restore(inputs);
            if(!MissionFlightSnapshot.hash(inputs.resolve("manifest.json")).equals(flightHash))throw new IOException("Recorded flight inventory changed");
            Path source=temporary.resolve("source.rktmission");MissionArchive.write(source,MissionArchive.RESULT,archive.manifest().getString("status"),files);
            String sourceHash=MissionFlightSnapshot.hash(source);
            var definition=Json.createObjectBuilder().add("version",1).add("sourceHash",sourceHash).add("member",member).add("telemetrySource",telemetrySource).build();
            try(var check=new MissionRecordedSource(source,definition,()->{})){ /* Validate before publishing a new draft. */ }
            Path snapshot=root.resolve("snapshots").resolve(flightHash);if(!Files.exists(snapshot))Files.move(inputs,snapshot);
            Files.move(source,root.resolve("sources").resolve(sourceHash+".rktmission"),StandardCopyOption.REPLACE_EXISTING);
            var editor=original.getJsonObject("flight").getJsonObject("editor");
            editor=Json.createObjectBuilder(editor).add("options",Json.createObjectBuilder(editor.getJsonObject("options"))
                    .add("ensemble",Json.createObjectBuilder(editor.getJsonObject("options").getJsonObject("ensemble")).add("enabled","false"))).build();
            var stations=Json.createArrayBuilder();for(var value:original.getJsonArray("stationBindings"))if(value.asJsonObject().getString("role").equals("telem"))stations.add(Json.createObjectBuilder(value.asJsonObject()).add("owner","headless-policy"));
            var telemetry=Json.createObjectBuilder(original.getJsonObject("telemetry"));if(telemetrySource.equals("standard-generator"))telemetry.add("mode","standard");
            var mission=Json.createObjectBuilder(original).remove("recordedFlight")
                    .add("identity",Json.createObjectBuilder(original.getJsonObject("identity")).add("missionId",UUID.randomUUID().toString()).add("revision",0)
                            .add("title",original.getJsonObject("identity").getString("title").substring(0,Math.min(170,original.getJsonObject("identity").getString("title").length()))+" · ground-link experiment"))
                    .add("flight",Json.createObjectBuilder(original.getJsonObject("flight")).add("editor",editor))
                    .add("execution",Json.createObjectBuilder(original.getJsonObject("execution")).add("mode","ground-link").add("clockMode","lockstep").add("applicationMember",0).add("actions",JsonValue.EMPTY_JSON_ARRAY))
                    .add("stationBindings",stations).add("telemetry",telemetry)
                    .add("outputs",Json.createObjectBuilder(original.getJsonObject("outputs")).add("filename","ground-link.rktmission").add("recordEncodedVideo",false)).build();
            definition=Json.createObjectBuilder(definition).add("editorHash",hash(editor)).add("launchHash",hash(MissionRecordedSource.launch(mission)))
                    .add("bindingsHash",hash(Json.createObjectBuilder().add("bindings",mission.getJsonArray("vehicleBindings")).build()))
                    .add("packetSettingsHash",hash(MissionRecordedSource.packetSettings(mission))).build();
            return load(new MissionDefinition(Json.createObjectBuilder(mission).add("recordedFlight",definition).build()));
        }finally{try(var paths=Files.walk(temporary)){for(var path:paths.sorted(Comparator.reverseOrder()).toList())Files.delete(path);}}
    }
    private Path recordedSource(JsonObject mission) throws IOException {
        String hash=mission.getJsonObject("recordedFlight").getString("sourceHash");
        if(!hash.matches("[0-9a-f]{64}"))throw new IOException("Invalid recorded source identity");
        Path path=root.resolve("sources").resolve(hash+".rktmission");
        if(Files.isSymbolicLink(path) || !Files.isRegularFile(path) || !MissionFlightSnapshot.hash(path).equals(hash))throw new IOException("Recorded source is unavailable or changed; import its portable setup");
        return path;
    }

    public Path snapshot(String hash) throws IOException {
        if (!hash.matches("[0-9a-f]{64}")) throw new IOException("Select or resolve a rocket snapshot first");
        Path folder = root.resolve("snapshots").resolve(hash);
        if (Files.isSymbolicLink(folder) || !Files.isDirectory(folder)) throw new IOException("Frozen rocket input is unavailable; resolve it or import a portable setup");
        if (!MissionFlightSnapshot.hash(folder.resolve("manifest.json")).equals(hash)) throw new IOException("Frozen input inventory changed");
        return folder;
    }
    private String freeze(Simulation simulation) throws IOException {
        Path temporary = root.resolve("snapshots").resolve("preparing-" + UUID.randomUUID());
        MissionFlightSnapshot.capture(simulation, temporary);
        String hash = MissionFlightSnapshot.hash(temporary.resolve("manifest.json"));
        Path destination = root.resolve("snapshots").resolve(hash);
        if (Files.exists(destination)) {
            MissionFlightSnapshot.restore(snapshot(hash));
            try (var files = Files.list(temporary)) { for (Path file : files.toList()) Files.delete(file); }
            Files.delete(temporary);
        } else Files.move(temporary, destination, StandardCopyOption.ATOMIC_MOVE);
        return hash;
    }
    public static String hash(JsonObject json) {
        try { return HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(MissionJson.bytes(canonical(json).asJsonObject()))); }
        catch (IOException | NoSuchAlgorithmException e) { throw new IllegalArgumentException(e); }
    }
    private static JsonValue canonical(JsonValue value) {
        if (value instanceof JsonObject object) { var b = Json.createObjectBuilder(); new TreeSet<>(object.keySet()).forEach(k -> b.add(k, canonical(object.get(k)))); return b.build(); }
        if (value instanceof JsonArray array) { var b = Json.createArrayBuilder(); array.forEach(v -> b.add(canonical(v))); return b.build(); }
        return value;
    }
    private static void checkRevision(JsonObject draft, int expected) {
        if (draft.getJsonObject("identity").getInt("revision") != expected) throw new Conflict("Draft revision changed; reload before continuing");
    }
    public static String id(JsonObject draft) { return draft.getJsonObject("identity").getString("missionId"); }
    public static void issue(JsonArrayBuilder out, String path, String code, String message) {
        out.add(Json.createObjectBuilder().add("path", path).add("code", code).add("severity", "error").add("message", message));
    }
    private static String safeMessage(Exception e) { return e.getMessage() == null ? "Invalid flight input (" + e.getClass().getSimpleName() + ")" : e.getMessage(); }
}
