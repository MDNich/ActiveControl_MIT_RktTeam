package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.io.*;
import java.nio.file.*;
import java.util.*;

/** Routes receiver bytes to exactly one command owner and optional read-only station applications. */
public final class MissionParticipants implements AutoCloseable {
    private final Path directory;
    private final JsonObject mission, preferences;
    private final String runId, memberId;
    private final MissionRecorder recorder;
    private final MissionWorker headless;
    private final Map<String, MissionWorker> applications = new LinkedHashMap<>();
    private final Map<String, JsonObject> bindings = new LinkedHashMap<>();
    private final Map<String, JsonObject> identities = new LinkedHashMap<>();
    public MissionParticipants(Path directory, JsonObject mission, String runId, String memberId,
                               JsonObject preferences, MissionRecorder recorder) throws IOException {
        this.directory = directory; this.mission = mission; this.runId = runId; this.memberId = memberId;
        this.preferences = preferences; this.recorder = recorder;
        headless = new MissionWorker(directory.resolve("headless"), MissionStore.id(mission), runId, memberId);
    }
    public void configure(JsonArray stations) throws IOException {
        var configs = new TreeMap<String, JsonObject>(); stations.forEach(v -> configs.put(v.asJsonObject().getString("id"), v.asJsonObject()));
        var headlessConfigs = Json.createArrayBuilder();
        for (var value : stations) {
            var station = value.asJsonObject();
            headlessConfigs.add(Json.createObjectBuilder(station).add("owner", station.getString("owner").equals("interactive-ui") ? "observer" : "headless-policy"));
        }
        headless.configure(headlessConfigs.build());
        for (var value : mission.getJsonArray("stationBindings")) {
            var binding = value.asJsonObject(); String site = binding.getString("siteId"), id = binding.getString("id");
            if (!configs.containsKey(site) || binding.getString("owner").equals("headless-policy")) continue;
            var config = Json.createObjectBuilder(configs.get(site)).add("owner", binding.getString("owner"))
                    .add("pointerEnabled", binding.getString("role").equals("telem") && configs.get(site).getBoolean("pointerEnabled")).build();
            var network = Json.createObjectBuilder().add("network_bind_host", "127.0.0.1").add("network_port", 0)
                    .add("network_discovery", false).add("network_accept_source_modes", Json.createArrayBuilder().add("simulation"))
                    .add("mission_id", MissionStore.id(mission)).add("peers", JsonValue.EMPTY_JSON_ARRAY).add("mission_simulation", true).build();
            Path data = directory.resolve(id).resolve("data");
            var manifest = Json.createObjectBuilder().add("station", config).add("role", binding.getString("role"))
                    .add("coupled", mission.getJsonObject("telemetry").getString("mode").equals("custom-fc"))
                    .add("vehicleMode", mission.getJsonArray("vehicleBindings").size() > 1 ? "iris" : "balius")
                    .add("dataDirectory", data.toString()).add("network", network).build();
            MissionWorker app = new MissionWorker(directory.resolve(id), MissionStore.id(mission), runId, memberId, manifest, preferences);
            applications.put(id, app); bindings.put(id, binding);
            identities.put(id, app.configure(Json.createArrayBuilder().add(config).build()));
        }
        var peers = Json.createArrayBuilder();
        identities.values().forEach(identity -> { if (identity.containsKey("endpoint")) peers.add(identity.getString("endpoint")); });
        JsonObject body = Json.createObjectBuilder().add("kind", "network-peers").add("endpoints", peers).build();
        for (var app : applications.values()) app.request("action", -1_000_000, body);
    }
    public JsonObject advance(long timeUs, JsonArray stations) throws IOException {
        // Remove scripted pointer actions from the headless observer at an interactive-owned site.
        var passive = Json.createArrayBuilder();
        for (var value : stations) {
            var station = value.asJsonObject(); boolean interactive = bindings.values().stream().anyMatch(b -> b.getString("siteId").equals(station.getString("id")) && b.getString("owner").equals("interactive-ui"));
            passive.add(interactive ? Json.createObjectBuilder(station).add("actions", JsonValue.EMPTY_JSON_ARRAY).build() : station);
        }
        var primary = new TreeMap<String, JsonObject>();
        for (var value : headless.advance(timeUs, passive.build()).getJsonArray("stations")) primary.put(value.asJsonObject().getString("id"), value.asJsonObject());
        var actions = Json.createArrayBuilder();
        for (var entry : applications.entrySet()) {
            var binding = bindings.get(entry.getKey()); String site = binding.getString("siteId");
            var batch = stations.stream().map(JsonValue::asJsonObject).filter(b -> b.getString("id").equals(site)).findFirst().orElseThrow();
            if (binding.getString("owner").equals("observer")) batch = Json.createObjectBuilder(batch).add("actions", JsonValue.EMPTY_JSON_ARRAY).build();
            var result = entry.getValue().advance(timeUs, Json.createArrayBuilder().add(batch).build());
            if (binding.getString("owner").equals("interactive-ui")) {
                primary.put(site, result.getJsonArray("stations").getJsonObject(0));
                result.getJsonArray("operatorActions").forEach(actions::add);
            }
        }
        var responses = Json.createArrayBuilder(); primary.values().forEach(responses::add);
        return Json.createObjectBuilder().add("stations", responses).add("operatorActions", actions).build();
    }
    public JsonObject provenance() {
        var apps = Json.createObjectBuilder(); identities.forEach(apps::add);
        return Json.createObjectBuilder(headless.provenance()).add("applications", apps).build();
    }
    @Override public void close() throws IOException {
        headless.close(); for (var app : applications.values()) app.close();
        // Processes have closed their native recorder/WAL before session assets are inventoried.
        for (String id : applications.keySet()) {
            Path data = directory.resolve(id).resolve("data"); if (!Files.isDirectory(data)) continue;
            var binding = bindings.get(id);
            String prefix = "runs/" + runId + "/stations/" + binding.getString("siteId") + "/sessions/" + binding.getString("role") + "/" + id + "/";
            recorder.json(prefix + "participant.json", identities.get(id));
            try (var files = Files.walk(data)) {
                for (Path file : files.filter(Files::isRegularFile).toList()) {
                    String relative = data.relativize(file).toString().replace('\\', '/');
                    if (relative.startsWith("sessions/") || relative.equals("mission-clock.jsonl")) recorder.asset(prefix + relative, file);
                }
            }
        }
    }
}
