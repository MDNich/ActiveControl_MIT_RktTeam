package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.io.*;
import java.math.BigInteger;
import java.nio.file.*;
import java.util.*;

/** Immutable mission draft. Unknown optional data survives save/load; a draft is not a runnable job. */
public final class MissionDefinition {
    private static final JsonObject SCHEMA = MissionJson.resource("mission.schema.json");
    private final JsonObject json;

    public MissionDefinition(JsonObject json) {
        try { MissionJson.bytes(json); } catch (IOException e) { throw new IllegalArgumentException(e); }
        List<MissionSchema.Issue> issues = validate(json);
        if (!issues.isEmpty()) throw new IllegalArgumentException(issues.toString());
        this.json = json;
    }

    public JsonObject json() { return json; }

    public static MissionDefinition createUrrg(String title) {
        JsonObject defaults = MissionJson.resource("urrg-draft.json");
        JsonObject identity = Json.createObjectBuilder(defaults.getJsonObject("identity"))
                .add("missionId", UUID.randomUUID().toString()).add("title", title).build();
        return new MissionDefinition(Json.createObjectBuilder(defaults).add("identity", identity).build());
    }

    public static MissionDefinition read(Path path) throws IOException {
        try (var input = Files.newInputStream(path)) {
            return new MissionDefinition(MissionJson.read(input));
        }
    }

    public void write(Path path) throws IOException { MissionJson.writeAtomic(path, json); }

    public static List<MissionSchema.Issue> validate(JsonObject json) {
        var issues = new ArrayList<>(MissionSchema.validate(SCHEMA, json));
        if (!issues.isEmpty()) return List.copyOf(issues);
        Map<String, JsonObject> sites = unique(json, "sites", issues);
        Map<String, JsonObject> vehicles = unique(json, "vehicleBindings", issues);
        Map<String, JsonObject> assets = unique(json, "assets", issues);
        unique(json, "pointers", issues);
        unique(json, "stationBindings", issues);
        for (String id : List.of("launch-site", "launch"))
            if (!sites.containsKey(id) || !sites.get(id).getBoolean("enabled"))
                issues.add(new MissionSchema.Issue("/sites", "required-site", "Enabled " + id + " is required"));
        var mounts = new HashSet<String>();
        for (int i = 0; i < json.getJsonArray("pointers").size(); i++) {
            JsonObject pointer = json.getJsonArray("pointers").getJsonObject(i);
            String site = pointer.getString("siteId");
            reference(sites, site, "/pointers/" + i + "/siteId", issues);
            reference(vehicles, pointer.getString("targetVehicleId"), "/pointers/" + i + "/targetVehicleId", issues);
            if (pointer.getBoolean("enabled") && (!mounts.add(site) || (sites.containsKey(site) && !sites.get(site).getBoolean("enabled"))))
                issues.add(new MissionSchema.Issue("/pointers/" + i, "pointer-site", "One enabled pointer per enabled Away site"));
        }
        for (int i = 0; i < json.getJsonArray("vehicleBindings").size(); i++) {
            JsonObject vehicle = json.getJsonArray("vehicleBindings").getJsonObject(i);
            if (vehicle.containsKey("rocketAssetId"))
                reference(assets, vehicle.getString("rocketAssetId"), "/vehicleBindings/" + i + "/rocketAssetId", issues);
        }
        var channels = new HashSet<String>();
        JsonArray streams = json.getJsonObject("video").getJsonArray("streams");
        for (int i = 0; i < streams.size(); i++) {
            JsonObject stream = streams.getJsonObject(i);
            if (!channels.add(stream.getString("id"))) issues.add(new MissionSchema.Issue("/video/streams/" + i + "/id", "duplicate-id", "Duplicate stream ID"));
            reference(vehicles, stream.getString("vehicleId"), "/video/streams/" + i + "/vehicleId", issues);
            var receivers = new HashSet<String>();
            for (var receiver : stream.getJsonArray("receivers")) {
                String id = ((JsonString) receiver).getString();
                reference(sites, id, "/video/streams/" + i + "/receivers", issues);
                if (!receivers.add(id)) issues.add(new MissionSchema.Issue("/video/streams/" + i + "/receivers", "duplicate-route", "Duplicate receiver"));
            }
        }
        var owners = new HashSet<String>();
        for (int i = 0; i < json.getJsonArray("stationBindings").size(); i++) {
            JsonObject station = json.getJsonArray("stationBindings").getJsonObject(i);
            String site = station.getString("siteId");
            reference(sites, site, "/stationBindings/" + i + "/siteId", issues);
            if (!station.getString("owner").equals("observer") && station.getString("role").equals("telem") && !owners.add(site))
                issues.add(new MissionSchema.Issue("/stationBindings/" + i, "multiple-owners", "One telemetry command owner per station"));
        }
        JsonObject execution = json.getJsonObject("execution");
        if (new BigInteger(execution.getString("seed")).bitLength() > 64)
            issues.add(new MissionSchema.Issue("/execution/seed", "range", "Seed must fit unsigned 64 bits"));
        if (Long.parseLong(execution.getString("deliveryTailUs")) > 60_000_000L)
            issues.add(new MissionSchema.Issue("/execution/deliveryTailUs", "range", "Delivery tail cannot exceed 60 seconds"));
        return List.copyOf(issues);
    }

    private static Map<String, JsonObject> unique(JsonObject json, String group, List<MissionSchema.Issue> issues) {
        var result = new LinkedHashMap<String, JsonObject>();
        JsonArray values = json.getJsonArray(group);
        for (int i = 0; i < values.size(); i++) {
            JsonObject value = values.getJsonObject(i);
            if (result.putIfAbsent(value.getString("id"), value) != null)
                issues.add(new MissionSchema.Issue("/" + group + "/" + i + "/id", "duplicate-id", "Duplicate ID"));
        }
        return result;
    }

    private static void reference(Map<String, JsonObject> group, String id, String path, List<MissionSchema.Issue> issues) {
        if (!group.containsKey(id)) issues.add(new MissionSchema.Issue(path, "missing-reference", "Unknown ID " + id));
    }
}
