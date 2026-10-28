package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.io.*;
import java.nio.file.*;
import java.util.*;

/** Routes receiver bytes to exactly one command owner and optional read-only station applications. */
public final class MissionParticipants implements AutoCloseable {
    private final Path directory;
    private final JsonObject mission, preferences;
    private final JsonArray videoStreams;
    private final String runId, memberId;
    private final MissionRecorder recorder;
    private final MissionWorker headless;
    private final Map<String, MissionWorker> applications = new LinkedHashMap<>();
    private final Map<String, JsonObject> bindings = new LinkedHashMap<>();
    private final Map<String, JsonObject> identities = new LinkedHashMap<>();
    private final Map<String, String> owners = new TreeMap<>();
    private final Map<String, Long> ownershipEpochs = new TreeMap<>();
    private boolean paused;
    private boolean cancelled;
    private String ownershipFailure = "";
    private final Map<String, Long> discardActionsThrough = new HashMap<>();
    public MissionParticipants(Path directory, JsonObject mission, String runId, String memberId,
                               JsonObject preferences, MissionRecorder recorder, JsonArray videoStreams) throws IOException {
        this.directory = directory; this.mission = mission; this.runId = runId; this.memberId = memberId;
        this.preferences = preferences; this.recorder = recorder; this.videoStreams = videoStreams;
        headless = new MissionWorker(directory.resolve("headless"), MissionStore.id(mission), runId, memberId);
    }
    public void configure(JsonArray stations) throws IOException {
        var configs = new TreeMap<String, JsonObject>(); stations.forEach(v -> configs.put(v.asJsonObject().getString("id"), v.asJsonObject()));
        var headlessConfigs = Json.createArrayBuilder();
        for (var value : stations) {
            var station = value.asJsonObject();
            owners.put(station.getString("id"), station.getString("owner").equals("interactive-ui") ? "interactive-ui" : "headless-policy");
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
                    .add("videoStreams", videoStreams).add("dataDirectory", data.toString()).add("network", network).build();
            manifest = Json.createObjectBuilder(manifest).add("recordEncodedVideo", mission.getJsonObject("outputs").getBoolean("recordEncodedVideo", false)).build();
            var appPreferences=preferences;
            if(preferences.getJsonObject("attachments")!=null && preferences.getJsonObject("attachments").containsKey(id))
                appPreferences=Json.createObjectBuilder(preferences).add("attachmentTicket",preferences.getJsonObject("attachments").getString(id)).build();
            MissionWorker app = new MissionWorker(directory.resolve(id), MissionStore.id(mission), runId, memberId, manifest, appPreferences);
            applications.put(id, app); bindings.put(id, binding);
            identities.put(id, app.configure(Json.createArrayBuilder().add(config).build()));
        }
        var peers = Json.createArrayBuilder();
        identities.values().forEach(identity -> { if (identity.containsKey("endpoint")) peers.add(identity.getString("endpoint")); });
        JsonObject body = Json.createObjectBuilder().add("kind", "network-peers").add("endpoints", peers).build();
        for (var app : applications.values()) app.request("action", -1_000_000, body);
    }
    public synchronized JsonObject advance(long timeUs, JsonArray stations) throws IOException {
        while(paused)try{wait();}catch(InterruptedException e){Thread.currentThread().interrupt();throw new IOException("Station wait interrupted",e);}
        if(cancelled)throw new java.util.concurrent.CancellationException("Mission cancelled");
        var clean=Json.createArrayBuilder();
        for(var value:stations){var station=value.asJsonObject();String site=station.getString("id");
            var actions=Json.createArrayBuilder();
            for(var action:station.getJsonArray("actions"))if(timeUs>discardActionsThrough.getOrDefault(site,Long.MIN_VALUE)
                    && !(ownershipEpochs.containsKey(site) && action.asJsonObject().getString("id","").equals("scenario-track-at-launch")))actions.add(action);
            clean.add(Json.createObjectBuilder(station).add("actions",actions));
        }
        stations=clean.build();
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
    public synchronized void pause(boolean paused, long timeUs) throws IOException {
        if(!paused && !ownershipFailure.isEmpty())throw new IOException("Ownership handoff failed; cancel and restart the member: "+ownershipFailure);
        headless.request(paused ? "pause" : "resume", timeUs, JsonValue.EMPTY_JSON_OBJECT);
        for (var app : applications.values()) app.request(paused ? "pause" : "resume", timeUs, JsonValue.EMPTY_JSON_OBJECT);
        this.paused = paused;
        notifyAll();
    }
    public synchronized JsonObject ownership() {
        var result = Json.createArrayBuilder();
        owners.forEach((site, owner) -> result.add(Json.createObjectBuilder().add("siteId", site).add("owner", owner)
                .add("epoch", ownershipEpochs.getOrDefault(site, 0L)).add("application", bindings.values().stream().anyMatch(b -> b.getString("siteId").equals(site) && b.getString("role").equals("telem")))));
        return Json.createObjectBuilder().add("stations", result).build();
    }
    public synchronized void cancel() { cancelled=true;paused=false;notifyAll(); }
    public synchronized JsonObject transfer(String site, String owner, long timeUs) throws IOException {
        if(!paused)throw new IllegalArgumentException("Pause the mission before transferring station control");
        if(!owners.containsKey(site) || !Set.of("interactive-ui", "headless-policy").contains(owner))throw new IllegalArgumentException("Unknown station or command owner");
        String id = bindings.entrySet().stream().filter(e -> e.getValue().getString("siteId").equals(site) && e.getValue().getString("role").equals("telem")).map(Map.Entry::getKey).findFirst()
                .orElseThrow(() -> new IllegalArgumentException("Start this station with an observer or interactive application to enable ownership transfer"));
        long epoch=ownershipEpochs.getOrDefault(site,0L)+1;
        var base=Json.createObjectBuilder().add("kind","ownership").add("siteId",site).add("epoch",Long.toString(epoch)).build();
        // Revoke both owners before granting either: a failed second handshake leaves no dual owner.
        try {
            headless.request("action",timeUs,Json.createObjectBuilder(base).add("owner","observer").build());
            applications.get(id).request("action",timeUs,Json.createObjectBuilder(base).add("owner","observer").build());
        epoch++;
        var grant=Json.createObjectBuilder(base).add("epoch",Long.toString(epoch)).add("owner",owner).build();
        (owner.equals("interactive-ui")?applications.get(id):headless).request("action",timeUs,grant);
        discardActionsThrough.put(site,timeUs);
        owners.put(site,owner);ownershipEpochs.put(site,epoch);
        bindings.put(id,Json.createObjectBuilder(bindings.get(id)).add("owner",owner.equals("interactive-ui")?owner:"observer").build());
        var outcome=Json.createObjectBuilder(grant).add("timeUs",Long.toString(timeUs)).add("trackingRestartRequired",true).build();
        recorder.jsonLine("runs/"+runId+"/ownership/events",outcome);
        return outcome;
        } catch(IOException e){ownershipFailure=e.getMessage();throw e;}
    }
    public JsonObject provenance() {
        var apps = Json.createObjectBuilder(); identities.forEach(apps::add);
        return Json.createObjectBuilder(headless.provenance()).add("applications", apps).build();
    }
    @Override public void close() throws IOException {
        IOException failure=null;
        try{headless.finish();}catch(IOException e){failure=e;}
        for(var app:applications.values())try{app.finish();}catch(IOException e){if(failure==null)failure=e;else failure.addSuppressed(e);}
        // Processes have closed their native recorder/WAL before session assets are inventoried.
        for (String id : applications.keySet()) {
            Path data = directory.resolve(id).resolve("data"); if (!Files.isDirectory(data)) continue;
            var binding = bindings.get(id);
            String prefix = "runs/" + runId + "/stations/" + binding.getString("siteId") + "/sessions/" + binding.getString("role") + "/" + id + "/";
            recorder.json(prefix + "participant.json", identities.getOrDefault(id,JsonValue.EMPTY_JSON_OBJECT));
            try (var files = Files.walk(data)) {
                for (Path file : files.filter(Files::isRegularFile).toList()) {
                    String relative = data.relativize(file).toString().replace('\\', '/');
                    if (relative.startsWith("sessions/") || relative.equals("mission-clock.jsonl") || relative.equals("capture-timing.jsonl") || relative.equals("wall-timing.jsonl")) recorder.asset(prefix + relative, file);
                }
            }
        }
        if(failure!=null)throw failure;
    }
}
