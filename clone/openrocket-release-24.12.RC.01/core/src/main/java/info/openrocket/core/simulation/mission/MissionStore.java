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
    public record Resolved(JsonObject mission, Path flightDirectory, String hash) { }
    public static final class Conflict extends IllegalArgumentException {
        public Conflict(String message) { super(message); }
    }
    private final Path root;
    private final Map<String, MissionDefinition> drafts = new LinkedHashMap<>();
    private final Function<String, Motor> motorLookup;

    public MissionStore(Path root, Function<String, Motor> motorLookup) throws IOException {
        this.root = root.toAbsolutePath(); this.motorLookup = motorLookup;
        Files.createDirectories(this.root.resolve("snapshots")); Files.createDirectories(this.root.resolve("drafts"));
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
            MissionArchive.write(destination, MissionArchive.SETUP, "complete", files);
        } finally { Files.deleteIfExists(mission); }
    }

    public synchronized JsonObject importSetup(Path file) throws IOException {
        Path imported = root.resolve("import-" + UUID.randomUUID());
        try (var archive = new MissionArchive(file)) {
            if (!archive.manifest().getString("format").equals(MissionArchive.SETUP)) throw new IOException("Choose a setup archive; result archives open in replay");
            var mission = new MissionDefinition(MissionJson.parse(archive.read("mission.json", MissionJson.MAX_BYTES)));
            String hash = mission.json().getJsonObject("flight").getString("snapshotHash", "");
            for (String name : archive.names()) if (!name.equals("mission.json") && !name.startsWith("assets/" + hash + "/"))
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
        return update(id, revision, Json.createObjectBuilder(draft).add("flight", flight).add("assets", assets).build());
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
        int index = 0;
        for (var entry : draft.getJsonArray("sites")) {
            var site = entry.asJsonObject();
            if (site.getBoolean("enabled") && site.isNull("heightM"))
                issue(issues, "/sites/" + index + "/heightM", "height-required", "Enter a height and confirm MSL or WGS84 ellipsoid datum");
            index++;
        }
        try {
            var simulation = editableFlight(id);
            if (simulation.getActiveConfiguration().getActiveStages().isEmpty())
                issue(issues, "/flight/editor/stages", "no-active-stage", "Enable a rocket stage");
            if (simulation.getActiveConfiguration().getActiveMotors().isEmpty())
                issue(issues, "/flight/editor/motors", "no-active-motor", "Select a motor on an active stage");
            if (draft.getJsonObject("telemetry").getString("mode").equals("custom-fc")) {
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
        String flightHash = freeze(simulation);
        var resolved = Json.createObjectBuilder(draft).add("sites", sites).add("geoid", MissionGeoid.bundled().provenance())
                .add("flight", Json.createObjectBuilder(draft.getJsonObject("flight")).add("snapshotHash", flightHash)
                        .add("editor", MissionFlightEditor.capture(simulation))).build();
        return new Resolved(resolved, snapshot(flightHash), hash(resolved));
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
