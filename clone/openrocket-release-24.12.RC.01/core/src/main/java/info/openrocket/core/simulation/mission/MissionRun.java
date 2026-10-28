package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.io.*;
import java.nio.file.*;
import java.util.*;

/** One member's canonical clock: plant -> TX -> link deliveries -> station policy -> commands -> commit. */
public final class MissionRun {
    public interface Control {
        void check();
        void progress(long timeUs, long endUs, String phase);
    }
    private record Delivery(long timeUs, String site, long ordinal, byte[] bytes, JsonObject metadata) { }
    private final MissionStore.Resolved resolved;
    private final MissionRecorder recorder;
    private final String runId;
    private final long member;
    private final Control control;
    private final Map<String, JsonObject> sites = new TreeMap<>(), pointers = new TreeMap<>(), bindings = new TreeMap<>(), stations = new TreeMap<>();
    private final Map<String, MissionPointer> mounts = new TreeMap<>();
    private final Map<String, MissionLink> links = new TreeMap<>();
    private final Map<String, long[]> counts = new TreeMap<>();
    private final PriorityQueue<Delivery> deliveries = new PriorityQueue<>(Comparator.comparingLong(Delivery::timeUs).thenComparing(Delivery::site).thenComparingLong(Delivery::ordinal));
    private long timeUs = -1_000_000;
    public MissionRun(MissionStore.Resolved resolved, MissionRecorder recorder, String runId, long member, Control control) {
        this.resolved = resolved; this.recorder = recorder; this.runId = runId; this.member = member; this.control = control;
    }
    public JsonObject execute(Path privateDirectory) throws Exception {
        JsonObject mission = resolved.mission(), execution = mission.getJsonObject("execution");
        var simulation = MissionFlightSnapshot.restore(resolved.flightDirectory());
        if (!mission.getJsonObject("telemetry").getString("mode").equals("standard")) throw new IllegalArgumentException("Coupled FC runner is not installed in this increment");
        if (simulation.getOptions().getEnsembleSettings().enabled()) throw new IllegalArgumentException("Mission ensemble runner is not installed in this increment");
        prepare();
        control.progress(0, 0, "Calculating complete flight branches");
        var truth = MissionTruth.compute(simulation, () -> { control.check(); try { recorder.checkpointIfDue(); } catch (IOException e) { throw new UncheckedIOException(e); } });
        truth.record(recorder, runId);
        for (var binding : bindings.values()) if (truth.at(binding, 0) == null) throw new IllegalArgumentException("Vehicle " + binding.getString("id") + " has no launch trajectory");
        long end = truth.end(), tail = Long.parseLong(execution.getString("deliveryTailUs")), stop = Math.addExact(end, tail);
        String seed = execution.getString("seed");
        var sources = new TreeMap<String, MissionTelemetrySource>();
        bindings.forEach((id, binding) -> sources.put(id, new MissionTelemetrySource(mission.getJsonObject("telemetry"), binding, sites.get("launch-site"), seed, member)));
        var stationConfigs = Json.createArrayBuilder();
        for (var entry : stations.entrySet()) {
            String id = entry.getKey(); var site = sites.get(id); var pointer = pointers.get(id);
            String policy = pointer == null ? "geodetic.v1" : pointer.getString("trackingPolicy");
            double lat = number(site, "latitudeDeg", 0), lon = number(site, "longitudeDeg", 0);
            if (policy.equals("legacy-ui.v1")) { lat = Math.rint(lat * 1e5) / 1e5; lon = Math.rint(lon * 1e5) / 1e5; }
            stationConfigs.add(Json.createObjectBuilder().add("id", id).add("origin", Json.createArrayBuilder().add(lat).add(lon)
                            .add(number(site, "ellipsoidalHeightM", 0) + (pointer == null ? 0 : number(pointer, "mountHeightM", 0))))
                    .add("policy", policy).add("pointerEnabled", pointer != null).add("owner", entry.getValue().getString("owner")));
        }
        try (var worker = new MissionWorker(privateDirectory, MissionStore.id(mission), runId, "member-" + member)) {
            worker.configure(stationConfigs.build());
            recorder.json("runs/" + runId + "/run.json", Json.createObjectBuilder().add("runId", runId).add("member", member)
                    .add("source", "standard-precomputed").add("seed", seed).add("worker", worker.provenance()).add("timeUnit", "signed-int64-microseconds")
                    .add("warmupUs", "1000000").add("flightEndUs", Long.toString(end)).add("deliveryEndUs", Long.toString(stop))
                    .add("sampling", MissionRandom.VERSION).add("queueMessages", 256).add("queueBytes", 4 * 1024 * 1024).build());
            long policyTick = -1_000_000;
            boolean connected = false, started = false;
            while (true) {
                long next = policyTick <= stop ? policyTick : Long.MAX_VALUE;
                for (var source : sources.values()) if (source.next() <= end) next = Math.min(next, source.next());
                if (!deliveries.isEmpty() && deliveries.peek().timeUs <= stop) next = Math.min(next, deliveries.peek().timeUs);
                if (!started) next = Math.min(next, 0);
                if (next == Long.MAX_VALUE) break;
                timeUs = next; control.check(); control.progress(timeUs, stop, "Replaying telemetry and tracking stations");
                for (var mount : mounts.values()) mount.advance(timeUs);
                for (var entry : sources.entrySet()) if (entry.getValue().next() == timeUs && timeUs <= end) {
                    var packet = entry.getValue().advance(timeUs, truth);
                    if (packet != null) transmit(entry.getKey(), packet, truth);
                }
                var actions = new HashMap<String, JsonArray>();
                for (String id : stations.keySet()) {
                    var array = Json.createArrayBuilder();
                    if (!connected) array.add(Json.createObjectBuilder().add("kind", "connect").add("id", "scenario-connect"));
                    if (!started && timeUs >= 0 && pointers.containsKey(id) && stations.get(id).getString("owner").equals("headless-policy"))
                        array.add(Json.createObjectBuilder().add("kind", "start-tracking").add("id", "scenario-track-at-launch"));
                    actions.put(id, array.build());
                }
                connected = true; if (timeUs >= 0) started = true;
                var batch = new TreeMap<String, List<JsonObject>>(); stations.keySet().forEach(id -> batch.put(id, new ArrayList<>()));
                int count = 0;
                while (!deliveries.isEmpty() && deliveries.peek().timeUs == timeUs) {
                    Delivery delivery = deliveries.remove();
                    var record = Json.createObjectBuilder(delivery.metadata).add("data", Base64.getEncoder().encodeToString(delivery.bytes)).build();
                    recorder.raw("runs/" + runId + "/stations/" + delivery.site + "/receiver", timeUs, delivery.bytes, delivery.metadata);
                    batch.get(delivery.site).add(record);
                    if (++count == 200) { consume(worker, batch, Map.of(), false, truth); batch.values().forEach(List::clear); count = 0; }
                }
                consume(worker, batch, actions, true, truth);
                if (policyTick == timeUs) {
                    for (var entry : mounts.entrySet()) recorder.jsonLine("runs/" + runId + "/stations/" + entry.getKey() + "/pointer/pose",
                            Json.createObjectBuilder(entry.getValue().state()).add("trackingErrorDeg", trackingError(entry.getKey(), truth)).build());
                    policyTick += 200_000;
                }
                recorder.checkpointIfDue();
            }
            // Pending deliveries beyond the explicit tail stay dropped/undelivered, never drained at infinity.
            for (var delivery : deliveries) event("delivery-after-tail", stop, Json.createObjectBuilder(delivery.metadata).add("siteId", delivery.site).build());
        }
        var metrics = Json.createObjectBuilder().add("profile", "mission-reception.v1").add("intervalStartUs", "-1000000").add("intervalEndUs", Long.toString(stop));
        var receivers = Json.createObjectBuilder();
        for (var entry : counts.entrySet()) { long[] c = entry.getValue(); receivers.add(entry.getKey(), Json.createObjectBuilder().add("eligibleTransmissions", c[0])
                .add("rfDelivered", c[1]).add("acceptedFrames", c[2]).add("rejectedFrames", c[3]).add("pointerCommands", c[4])
                .add("acceptedFraction", c[0] == 0 ? JsonValue.NULL : Json.createValue((double) c[2] / c[0]))); }
        var result = metrics.add("receivers", receivers).build(); recorder.json("runs/" + runId + "/metrics.json", result);
        recorder.json("runs/" + runId + "/validation.json", Json.createObjectBuilder().add("status", "complete").add("truthLeakToPolicy", false)
                .add("unavailableHardwareChannels", true).add("pointerModel", "illustrative-rate-limited.v1").build());
        recorder.checkpoint("incomplete"); return result;
    }
    private void prepare() {
        var mission = resolved.mission();
        for (var entry : mission.getJsonArray("sites")) if (entry.asJsonObject().getBoolean("enabled")) sites.put(entry.asJsonObject().getString("id"), entry.asJsonObject());
        for (var entry : mission.getJsonArray("vehicleBindings")) bindings.put(entry.asJsonObject().getString("id"), entry.asJsonObject());
        for (var entry : mission.getJsonArray("pointers")) if (entry.asJsonObject().getBoolean("enabled")) {
            var p = entry.asJsonObject(); pointers.put(p.getString("siteId"), p); mounts.put(p.getString("siteId"), new MissionPointer(p));
        }
        for (var entry : mission.getJsonArray("stationBindings")) {
            var station = entry.asJsonObject(); if (!station.getString("role").equals("telem") || !sites.containsKey(station.getString("siteId"))) continue;
            stations.put(station.getString("siteId"), station); counts.put(station.getString("siteId"), new long[5]);
        }
        for (String id : stations.keySet()) {
            JsonObject profile = mission.getJsonObject("telemetry").getJsonArray("links").stream().map(JsonValue::asJsonObject)
                    .filter(p -> p.getString("receiverId").equals(id)).findFirst().orElse(Json.createObjectBuilder().add("receiverId", id).build());
            links.put(id, new MissionLink(profile, mission.getJsonObject("execution").getString("seed"), member));
        }
    }
    private void transmit(String vehicle, MissionTelemetrySource.Packet packet, MissionTruth truth) throws IOException {
        recorder.raw("runs/" + runId + "/transmitters/" + vehicle + "/tx", timeUs, packet.payload(), packet.metadata());
        for (var entry : stations.entrySet()) {
            String site = entry.getKey(), target = entry.getValue().getString("targetVehicleId", bindings.keySet().iterator().next());
            boolean tuned = target.equals(vehicle); if (tuned) counts.get(site)[0]++;
            var outcome = links.get(site).transmit(timeUs, packet.ordinal(), packet.payload(), 144, tuned, beamQualified(site, truth));
            event("link-" + outcome.outcome(), timeUs, Json.createObjectBuilder(packet.metadata()).add("siteId", site).add("deliveryUs", Long.toString(outcome.deliveryUs())).build());
            if (outcome.payload() != null) {
                counts.get(site)[1]++;
                byte[] frame = LegacyTelemetryEnvelope.wrap(outcome.payload(), MissionTelemetrySource.trailer(sites.get(site)));
                deliveries.add(new Delivery(outcome.deliveryUs(), site, packet.ordinal(), frame, packet.metadata()));
            }
        }
    }
    private void consume(MissionWorker worker, Map<String, List<JsonObject>> batch, Map<String, JsonArray> actions, boolean evaluate, MissionTruth truth) throws IOException {
        var array = Json.createArrayBuilder();
        for (String id : stations.keySet()) { var packets = Json.createArrayBuilder(); batch.get(id).forEach(packets::add);
            array.add(Json.createObjectBuilder().add("id", id).add("deliveries", packets).add("actions", actions.getOrDefault(id, JsonValue.EMPTY_JSON_ARRAY)).add("evaluate", evaluate)); }
        var result = worker.advance(timeUs, array.build());
        for (var entry : result.getJsonArray("stations")) {
            var station = entry.asJsonObject(); String id = station.getString("id");
            counts.get(id)[2] += station.getJsonArray("samples").size(); counts.get(id)[3] += station.getInt("rejected");
            for (var sample : station.getJsonArray("samples")) recorder.jsonLine("runs/" + runId + "/stations/" + id + "/receiver/decoded",
                    Json.createObjectBuilder(sample.asJsonObject()).add("timeUs", Long.toString(timeUs)).build());
            for (var action : station.getJsonArray("actions")) event("station-action", timeUs, Json.createObjectBuilder(action.asJsonObject()).add("siteId", id).build());
            for (var value : station.getJsonArray("writes")) {
                var write = value.asJsonObject(); byte[] raw = Base64.getDecoder().decode(write.getString("data"));
                var mount = mounts.get(id); if (mount == null) throw new IOException("Station commanded an unowned pointer");
                recorder.raw("runs/" + runId + "/stations/" + id + "/pointer/tx", timeUs, raw, Json.createObjectBuilder().add("rawRole", 3).add("source", "simulated-pointer-tx").build());
                String outcome = mount.command(raw, Long.parseLong(write.getString("createdUs")), Long.parseLong(write.getString("expiresUs")), timeUs);
                counts.get(id)[4]++; event("pointer-command", timeUs, Json.createObjectBuilder().add("siteId", id).add("outcome", outcome).build());
            }
        }
    }
    private double trackingError(String site, MissionTruth truth) {
        var mount = mounts.get(site); if (mount == null) return -1;
        String target = stations.get(site).getString("targetVehicleId", bindings.keySet().iterator().next());
        var state = truth.at(bindings.get(target), timeUs); if (state == null) return -1;
        var location = sites.get(site); var los = MissionGeodesy.pointing(state.ecef(), number(location, "latitudeDeg", 0), number(location, "longitudeDeg", 0),
                number(location, "ellipsoidalHeightM", 0) + number(pointers.get(site), "mountHeightM", 0));
        if (los == null) return -1;
        double a = Math.toRadians(los[1]), b = Math.toRadians(mount.elevation());
        return Math.toDegrees(Math.acos(MissionPointer.clamp(Math.sin(a) * Math.sin(b) + Math.cos(a) * Math.cos(b) * Math.cos(Math.toRadians(los[0] - mount.azimuth())), -1, 1)));
    }
    private boolean beamQualified(String site, MissionTruth truth) {
        var profile = resolved.mission().getJsonObject("telemetry").getJsonArray("links").stream().map(JsonValue::asJsonObject).filter(p -> p.getString("receiverId").equals(site)).findFirst().orElse(JsonValue.EMPTY_JSON_OBJECT);
        if (!profile.getBoolean("directional", false)) return true;
        double error = trackingError(site, truth); return error >= 0 && error <= number(profile, "beamWidthDeg", 60) / 2;
    }
    private void event(String kind, long time, JsonObject metadata) throws IOException { recorder.event(runId, Json.createObjectBuilder(metadata).add("kind", kind).add("timeUs", Long.toString(time)).build()); }
    private static double number(JsonObject object, String key, double fallback) { return MissionPointer.number(object, key, fallback); }
}
