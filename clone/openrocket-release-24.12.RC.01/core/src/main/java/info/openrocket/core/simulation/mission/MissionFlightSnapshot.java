package info.openrocket.core.simulation.mission;

import info.openrocket.core.document.*;
import info.openrocket.core.file.*;
import info.openrocket.core.file.openrocket.OpenRocketSaver;
import info.openrocket.core.logging.*;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.simulation.flightcomputer.*;
import jakarta.json.*;
import java.io.*;
import java.nio.file.*;
import java.security.*;
import java.util.*;

/** Native design plus complete settings, motor curves and FC inputs. Capture/restore never runs extensions. */
public final class MissionFlightSnapshot {
    private MissionFlightSnapshot() { }

    public static JsonObject capture(Simulation source, Path directory) throws IOException {
        Files.createDirectory(directory); // New snapshot only: never overwrite a prior run's inputs.
        JsonObject options = MissionFlightOptions.capture(source.getOptions());
        JsonObject editor = MissionFlightEditor.capture(source);
        var rocket = source.getRocket().copyWithOriginalID();
        var document = OpenRocketDocumentFactory.createDocumentFromRocket(rocket);
        Simulation simulation = source.duplicateSimulation(document, rocket);
        document.addSimulation(simulation);
        var fcFiles = Json.createObjectBuilder();
        var types = Json.createArrayBuilder();
        for (int i = 0; i < simulation.getSimulationExtensions().size(); i++) {
            var extension = simulation.getSimulationExtensions().get(i);
            types.add(extension.getClass().getName());
            if (extension instanceof ZephyrusFlightComputer fc && fc.hasDesignFile()) {
                Path designPath = fc.getMissionDesign() != null ? fc.getMissionDesign().path()
                        : FlightComputerLibrary.resolve(fc.getDesignReference());
                FlightComputerDesign.read(designPath); // Validate before freezing the original bytes.
                String name = "fc-" + i + ".fc";
                Files.copy(designPath, directory.resolve(name));
                fcFiles.add(Integer.toString(i), name);
            }
        }
        var storage = new StorageOptions(); storage.setSaveSimulationData(false);
        try (var output = Files.newOutputStream(directory.resolve("rocket.ork"))) {
            new OpenRocketSaver().save(output, document, storage, new WarningSet(), new ErrorSet());
        }
        MissionJson.writeAtomic(directory.resolve("motors.json"), MissionMotorSnapshot.capture(rocket));
        JsonObject meta = Json.createObjectBuilder().add("format", "openrocket.mission-flight").add("version", 1)
                .add("name", source.getName()).add("flightConfigurationId", source.getId().key.toString())
                .add("options", options).add("editor", editor).add("extensionTypes", types).add("fcFiles", fcFiles).build();
        MissionJson.writeAtomic(directory.resolve("flight.json"), meta);
        var hashes = Json.createObjectBuilder();
        try (var files = Files.list(directory)) {
            for (Path file : files.sorted().toList()) hashes.add(file.getFileName().toString(), hash(file));
        }
        JsonObject manifest = Json.createObjectBuilder().add("format", "openrocket.mission-flight-inventory")
                .add("version", 1).add("files", hashes).build();
        MissionJson.writeAtomic(directory.resolve("manifest.json"), manifest);
        return meta;
    }

    public static Simulation restore(Path directory) throws IOException {
        JsonObject manifest = read(directory.resolve("manifest.json"));
        if (!"openrocket.mission-flight-inventory".equals(manifest.getString("format", "")) || manifest.getInt("version", -1) != 1)
            throw new IOException("Unsupported flight snapshot inventory");
        JsonObject files = manifest.getJsonObject("files");
        for (String required : List.of("rocket.ork", "flight.json", "motors.json"))
            if (!files.containsKey(required)) throw new IOException("Missing snapshot input " + required);
        for (String name : files.keySet()) {
            if (!name.matches("rocket\\.ork|flight\\.json|motors\\.json|fc-[0-9]+\\.fc")) throw new IOException("Invalid snapshot path");
            Path file = directory.resolve(name);
            if (Files.isSymbolicLink(file) || Files.size(file) > 64 * 1024 * 1024 || !hash(file).equals(files.getString(name)))
                throw new IOException("Snapshot hash mismatch: " + name);
        }
        var meta = read(directory.resolve("flight.json"));
        if (!"openrocket.mission-flight".equals(meta.getString("format", "")) || meta.getInt("version", -1) != 1)
            throw new IOException("Unsupported flight snapshot");
        var loader = new GeneralRocketLoader(directory.resolve("rocket.ork").toFile(), MissionMotorSnapshot.finder(read(directory.resolve("motors.json"))));
        OpenRocketDocument document;
        try { document = loader.load(); }
        catch (RocketLoadException e) { throw new IOException("Cannot load frozen rocket", e); }
        if (document.getSimulationCount() != 1) throw new IOException("Snapshot must contain one selected simulation");
        Simulation simulation = document.getSimulation(0);
        var expectedTypes = meta.getJsonArray("extensionTypes");
        if (simulation.getSimulationExtensions().size() != expectedTypes.size()) throw new IOException("Snapshot extensions unavailable");
        for (int i = 0; i < expectedTypes.size(); i++) {
            var extension = simulation.getSimulationExtensions().get(i);
            if (!extension.getClass().getName().equals(expectedTypes.getString(i))) throw new IOException("Snapshot extension type mismatch");
            String key = Integer.toString(i);
            if (meta.getJsonObject("fcFiles").containsKey(key)) {
                String name = meta.getJsonObject("fcFiles").getString(key);
                if (!files.containsKey(name) || !(extension instanceof ZephyrusFlightComputer)) throw new IOException("Invalid frozen FC reference");
                Path path = directory.resolve(name).toAbsolutePath();
                ((ZephyrusFlightComputer) extension).setMissionDesign(new FlightComputerLibrary.Resolved(
                        FlightComputerDesign.read(path), path, files.getString(name)));
            }
        }
        if (!simulation.getId().key.toString().equals(meta.getString("flightConfigurationId"))) throw new IOException("Selected flight configuration was lost");
        var restoredOptions = MissionFlightOptions.restore(meta.getJsonObject("options"));
        simulation.copySimulationOptionsFrom(restoredOptions);
        simulation.getOptions().setRandomSeed(restoredOptions.getRandomSeed());
        // Native .ork serialization normalizes ignition overrides and omits top-stage separation settings.
        // Keep the complete editable inventory in the companion, including currently inactive values.
        if (meta.containsKey("editor")) MissionFlightEditor.apply(simulation, meta.getJsonObject("editor"), digest -> null);
        return simulation;
    }

    private static JsonObject read(Path file) throws IOException {
        try (var input = Files.newInputStream(file)) { return MissionJson.read(input); }
    }

    public static String hash(Path path) throws IOException {
        try {
            var digest = MessageDigest.getInstance("SHA-256");
            try (var input = Files.newInputStream(path)) {
                byte[] bytes = new byte[64 * 1024]; int count;
                while ((count = input.read(bytes)) >= 0) digest.update(bytes, 0, count);
            }
            return HexFormat.of().formatHex(digest.digest());
        } catch (NoSuchAlgorithmException e) { throw new IllegalStateException(e); }
    }
}
