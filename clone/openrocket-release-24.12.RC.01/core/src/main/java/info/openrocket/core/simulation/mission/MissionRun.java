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
        default void beginMember(int member) { }
        default void participants(MissionParticipants value) { }
        default JsonArray drainActions(int maximum) { return JsonValue.EMPTY_JSON_ARRAY; }
        default JsonObject reconnectParticipant(JsonObject failure) throws IOException { throw new IOException("Mandatory participant lost: "+failure.getString("message")); }
        default void participantRestored() { }
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
    private final Map<String, MissionReceptionMetrics> reception = new TreeMap<>();
    private final Map<String, long[]> counts = new TreeMap<>();
    private final PriorityQueue<Delivery> deliveries = new PriorityQueue<>(Comparator.comparingLong(Delivery::timeUs).thenComparing(Delivery::site).thenComparing(d -> d.metadata.getString("transmitterId")).thenComparingLong(Delivery::ordinal));
    private long timeUs = -1_000_000, deliveryBytes;
    private static final int MAX_PENDING = 100_000;
    private static final long MAX_PENDING_BYTES = 64L * 1024 * 1024;
    private final PriorityQueue<JsonObject> tape = new PriorityQueue<>(Comparator.comparingLong((JsonObject a) -> Long.parseLong(a.getString("timeUs"))).thenComparing(a -> a.getString("id", "")));
    private record Uplink(long timeUs, long ordinal, byte[] bytes, String id) { }
    private final PriorityQueue<Uplink> uplinks = new PriorityQueue<>(Comparator.comparingLong(Uplink::timeUs).thenComparingLong(Uplink::ordinal));
    private MissionLink uplink;
    private long uplinkOrdinal;

    public MissionRun(MissionStore.Resolved resolved, MissionRecorder recorder, String runId, long member, Control control) {
        this.resolved = resolved; this.recorder = recorder; this.runId = runId; this.member = member; this.control = control;
    }
    private info.openrocket.core.simulation.ensemble.EnsembleRunParameters ensembleInputs;
    public MissionRun ensembleMember(info.openrocket.core.simulation.ensemble.EnsembleRunParameters inputs) { ensembleInputs = inputs; return this; }
    private MissionParticipants worker;
    private MissionVideo video;
    private JsonObject preferences = JsonValue.EMPTY_JSON_OBJECT;
    private boolean headlessParticipants;
    public MissionRun headlessParticipants(boolean value) { headlessParticipants=value;return this; }
    public MissionRun applicationPreferences(JsonObject value) { preferences = value; return this; }
    private MissionTruth truth;
    private TelemetryCodec codec;
    private edu.mit.rocket_team.FC.RTFC computer;
    private long policyTick = -1_000_000, stop, transmitted;
    private boolean connected, started;

    private MissionRecordedSource recorded;
    public JsonObject execute(Path privateDirectory) throws Exception {
        if(resolved.recordedSource()==null)return executeInternal(privateDirectory);
        try(var source=new MissionRecordedSource(resolved.recordedSource(),resolved.mission().getJsonObject("recordedFlight"),this::checkpoint)) {
            recorded=source;return executeInternal(privateDirectory);
        }
    }
    private JsonObject executeInternal(Path privateDirectory) throws Exception {
        JsonObject mission = resolved.mission(), execution = mission.getJsonObject("execution");
        var simulation = MissionFlightSnapshot.restore(resolved.flightDirectory());
        var ensembleSettings = simulation.getOptions().getEnsembleSettings();
        var extra = new ArrayList<info.openrocket.core.simulation.listeners.SimulationListener>();
        if (ensembleSettings.enabled()) {
            if (ensembleInputs == null) throw new IllegalArgumentException("An ensemble member requires frozen sampled inputs");
            info.openrocket.core.simulation.ensemble.EnsembleSampler.apply(simulation, ensembleSettings, ensembleInputs);
            extra.add(new info.openrocket.core.simulation.ensemble.EnsembleRunListener(ensembleSettings, ensembleInputs));
            var inputs = Json.createObjectBuilder(); new TreeMap<>(ensembleInputs.attributes()).forEach(inputs::add);
            recorder.json("runs/" + runId + "/ensemble-inputs.json", inputs.build());
        }
        boolean coupled = recorded==null && mission.getJsonObject("telemetry").getString("mode").equals("custom-fc");
        prepare();
        JsonObject participantMission=mission;
        if(headlessParticipants) {
            var roles=Json.createArrayBuilder();
            for(var value:mission.getJsonArray("stationBindings"))if(value.asJsonObject().getString("role").equals("telem"))
                roles.add(Json.createObjectBuilder(value.asJsonObject()).add("owner","headless-policy"));
            participantMission=Json.createObjectBuilder(mission).add("stationBindings",roles).build();
            stations.replaceAll((id,binding)->Json.createObjectBuilder(binding).add("owner","headless-policy").build());
        }
        recorder.json("runs/"+runId+"/participant-mode.json",Json.createObjectBuilder().add("headlessOverride",headlessParticipants)
                .add("stationBindings",participantMission.getJsonArray("stationBindings")).build());
        video = new MissionVideo(mission, recorder, runId, member);
        var fc = info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer.read(simulation);
        if(recorded!=null) {
            truth=recorded.truth;
            if(mission.getJsonObject("recordedFlight").getString("telemetrySource").equals("recorded-tx") && recorded.original.getJsonObject("telemetry").getString("mode").equals("custom-fc")) {
                if(fc.getMissionDesign()==null)throw new IOException("Recorded custom packets require the original codec design");
                codec=fc.getMissionDesign().design().telemetryCodec();
            }
            recorder.json("runs/"+runId+"/recorded-source.json",Json.createObjectBuilder(mission.getJsonObject("recordedFlight")).add("flightComputerExecuted",false).build());
        } else if (coupled) {
            if (simulation.getActiveConfiguration().getActiveStageCount() != 1 || bindings.size() != 1 || fc.getMissionDesign() == null)
                throw new IllegalArgumentException("Coupled missions need one active stage, one bound vehicle and a frozen FC design");
            codec = fc.getMissionDesign().design().telemetryCodec();
        } else {
            control.progress(0, 0, "Calculating complete flight branches");
            truth = MissionTruth.compute(simulation, this::checkpoint, extra.toArray(info.openrocket.core.simulation.listeners.SimulationListener[]::new));
        }
        try (var stationWorker = new MissionParticipants(privateDirectory, participantMission, runId, "member-" + member, preferences, recorder, video.applicationStreams())) {
            worker = stationWorker; control.participants(worker); worker.configure(stationConfiguration());
            if (coupled) {
                truth = new MissionTruth(this::checkpoint);
                stop = Math.round(simulation.getOptions().getMaxSimulationTime() * 1e6);
                fc.setMissionHook(hook(privateDirectory.resolve("fc")));
                extra.add(truth);
                try { simulation.simulate(extra.toArray(info.openrocket.core.simulation.listeners.SimulationListener[]::new)); }
                finally {
                if (computer != null) computer.telemetry.finishAt(false, computer.trace.bootUs());
                if (!truth.branches().isEmpty()) truth.record(recorder, runId);
                if (computer != null && computer.telemetry.getLogPath() != null) {
                    try (var files = Files.list(computer.telemetry.getLogPath().getParent())) {
                        for (Path file : files.toList()) recorder.asset("runs/" + runId + "/fc/" + bindings.keySet().iterator().next() + "/" + file.getFileName(), file);
                    }
                }
                }
            } else {
                truth.record(recorder, runId);
                for (var binding : bindings.values()) if (truth.at(binding, 0) == null) throw new IllegalArgumentException("Vehicle " + binding.getString("id") + " has no launch trajectory");
                long end = truth.end(); video.end(end); stop = Math.addExact(end, Long.parseLong(execution.getString("deliveryTailUs")));
                var sources = new TreeMap<String, MissionTelemetrySource>();
                boolean savedPackets=recorded!=null && mission.getJsonObject("recordedFlight").getString("telemetrySource").equals("recorded-tx");
                if(!savedPackets)bindings.forEach((id, binding) -> sources.put(id, new MissionTelemetrySource(mission.getJsonObject("telemetry"), binding, sites.get("launch-site"), execution.getString("seed"), member)));
                while (true) {
                    long next = nextStationDeadline();
                    if(savedPackets && recorded.next()<=end)next=Math.min(next,recorded.next());
                    for (var source : sources.values()) if (source.next() <= end) next = Math.min(next, source.next());
                    if (next > stop) break;
                    moveClock(next, "Replaying telemetry and tracking stations");
                    for (var entry : sources.entrySet()) if (entry.getValue().next() == timeUs && timeUs <= end) {
                        var packet = entry.getValue().advance(timeUs, truth); if (packet != null) transmit(entry.getKey(), packet, truth);
                    }
                    if(savedPackets)while(recorded.next()==timeUs && timeUs<=end) {
                        var packet=recorded.take();transmit(packet.vehicle(),packet.packet(),truth);
                    }
                    serviceBoundary();
                }
            }
            for (var delivery : deliveries) event("delivery-after-tail", stop, Json.createObjectBuilder(delivery.metadata).add("siteId", delivery.site).build());
            for (var pending : uplinks) event("uplink-after-tail", stop, Json.createObjectBuilder().add("commandId", pending.id).build());
            recorder.json("runs/" + runId + "/run.json", Json.createObjectBuilder().add("runId", runId).add("member", member)
                    .add("source", recorded!=null ? "recorded-flight-"+mission.getJsonObject("recordedFlight").getString("telemetrySource") : coupled ? "custom-fc-coupled" : "standard-precomputed").add("seed", execution.getString("seed"))
                    .add("worker", worker.provenance()).add("timeUnit", "signed-int64-microseconds")
                    .add("warmupUs", "1000000").add("flightEndUs", Long.toString(truth.end())).add("deliveryEndUs", Long.toString(stop))
                    .add("sampling", MissionRandom.VERSION).add("queueMessages", 256).add("queueBytes", 4 * 1024 * 1024).build());
        }
        return metrics();
    }
    private JsonArray stationConfiguration() {
        var stationConfigs = Json.createArrayBuilder();
        for (var entry : stations.entrySet()) {
            String id = entry.getKey(); var site = sites.get(id); var pointer = pointers.get(id);
            String policy = pointer == null ? "geodetic.v1" : pointer.getString("trackingPolicy");
            double lat = number(site, "latitudeDeg", 0), lon = number(site, "longitudeDeg", 0);
            if (policy.equals("legacy-ui.v1")) { lat = Math.rint(lat * 1e5) / 1e5; lon = Math.rint(lon * 1e5) / 1e5; }
            stationConfigs.add(Json.createObjectBuilder().add("id", id).add("origin", Json.createArrayBuilder().add(lat).add(lon)
                            .add(number(site, "ellipsoidalHeightM", 0) + (pointer == null ? 0 : number(pointer, "mountHeightM", 0))))
                    .add("policy", policy).add("pointerEnabled", pointer != null).add("owner", entry.getValue().getString("owner"))
                    .add("codec", codec == null ? JsonValue.NULL : codec.definition()));
        }
        return stationConfigs.build();
    }
    private void checkpoint() { control.check(); try { recorder.checkpointIfDue(); } catch (IOException e) { throw new UncheckedIOException(e); } }
    private long nextStationDeadline() {
        long next = Math.min(policyTick, video.next());
        if (!deliveries.isEmpty()) next = Math.min(next, deliveries.peek().timeUs);
        if (!started) next = Math.min(next, 0);
        if (!tape.isEmpty()) next = Math.min(next, Long.parseLong(tape.peek().getString("timeUs")));
        if (!uplinks.isEmpty()) next = Math.min(next, uplinks.peek().timeUs);
        return next;
    }
    private void moveClock(long time, String phase) {
        if (time < timeUs) throw new IllegalStateException("Mission clock moved backwards");
        control.check(); timeUs = time; control.progress(timeUs, stop, phase);
        for (var mount : mounts.values()) mount.advance(timeUs);
    }
    private void serviceBoundary() throws IOException {
        video.advance(timeUs);
        var actions = new HashMap<String, JsonArray>();
        for (String id : stations.keySet()) {
            var array = Json.createArrayBuilder();
            if (!connected) array.add(Json.createObjectBuilder().add("kind", "connect").add("id", "scenario-connect"));
            if (!started && timeUs >= 0 && pointers.containsKey(id) && !stations.get(id).getString("owner").equals("interactive-ui"))
                array.add(Json.createObjectBuilder().add("kind", "start-tracking").add("id", "scenario-track-at-launch"));
            actions.put(id, array.build());
        }
        connected = true; if (timeUs >= 0) started = true;
        var pendingActions = new ArrayList<JsonObject>();
        while (!tape.isEmpty() && Long.parseLong(tape.peek().getString("timeUs")) <= timeUs) pendingActions.add(tape.remove());
        for (var value : control.drainActions(200 - pendingActions.size())) pendingActions.add(value.asJsonObject());
        for (var action : pendingActions) {
            MissionActions.validate(action); String site = action.getString("siteId");
            event("scenario-action", timeUs, action);
            if(action.containsKey("expiresUs") && timeUs>Long.parseLong(action.getString("expiresUs"))) {
                event("action-rejected",timeUs,Json.createObjectBuilder(action).add("reason","Operator command expired before admission").build());continue;
            }
            if (!stations.containsKey(site)) { event("action-rejected", timeUs, Json.createObjectBuilder(action).add("reason", "Station is disabled").build()); continue; }
            if (action.getString("kind").equals("uplink")) {
                if (computer == null) { event("action-rejected", timeUs, Json.createObjectBuilder(action).add("reason", "A precomputed flight cannot receive FC commands").build()); continue; }
                byte[] raw = MissionActions.packet(action); long ordinal = uplinkOrdinal++;
                recorder.raw("runs/" + runId + "/stations/launch/uplink/tx", timeUs, raw, action);
                var outcome = uplink.transmit(timeUs, ordinal, raw, raw.length, true, true);
                event("uplink-" + outcome.outcome(), timeUs, Json.createObjectBuilder(action).add("wireAcknowledgment", false).build());
                if (outcome.payload() != null) {
                    if(uplinks.size()>=10_000)throw new IOException("Uplink backlog exceeded 10,000 commands; reduce delay or command rate");
                    uplinks.add(new Uplink(outcome.deliveryUs(), ordinal, outcome.payload(), action.getString("id", "uplink-" + ordinal)));
                }
            } else if (action.getString("kind").equals("set-target")) {
                String target = action.getString("targetVehicleId");
                if (!bindings.containsKey(target)) throw new IllegalArgumentException("Unknown target vehicle");
                stations.put(site, Json.createObjectBuilder(stations.get(site)).add("targetVehicleId", target).build());
                deliveries.removeIf(delivery -> { if(!delivery.site.equals(site))return false;deliveryBytes-=delivery.bytes.length;return true; });
                actions.put(site, Json.createArrayBuilder(actions.get(site)).add(Json.createObjectBuilder().add("kind", "disconnect"))
                        .add(Json.createObjectBuilder().add("kind", "connect")).build());
                event("target-changed", timeUs, Json.createObjectBuilder(action).add("acknowledgment", "simulator-control-event").build());
            } else actions.put(site, Json.createArrayBuilder(actions.get(site)).add(action).build());
        }
        while (!uplinks.isEmpty() && uplinks.peek().timeUs <= timeUs) {
            var delivery = uplinks.remove(); computer.telemetry.enqueue(delivery.bytes, delivery.id);
            recorder.raw("runs/" + runId + "/fc/" + bindings.keySet().iterator().next() + "/uplink/rx", timeUs, delivery.bytes,
                    Json.createObjectBuilder().add("commandId", delivery.id).add("outcome", "queued-for-next-FC-loop").build());
        }
        var batch = new TreeMap<String, List<JsonObject>>(); stations.keySet().forEach(id -> batch.put(id, new ArrayList<>()));
        int count = 0;
        while (!deliveries.isEmpty() && deliveries.peek().timeUs == timeUs) {
            Delivery delivery = deliveries.remove(); deliveryBytes-=delivery.bytes.length;
            var record = Json.createObjectBuilder(delivery.metadata).add("data", Base64.getEncoder().encodeToString(delivery.bytes)).build();
            recorder.raw("runs/" + runId + "/stations/" + delivery.site + "/receiver", timeUs, delivery.bytes, delivery.metadata);
            batch.get(delivery.site).add(record);
            if (++count == 200) { consume(worker, batch, Map.of(), false, truth); batch.values().forEach(List::clear); count = 0; }
        }
        int actionCount = actions.values().stream().mapToInt(JsonArray::size).sum();
        if (count + actionCount > 256) { consume(worker, batch, Map.of(), false, truth); batch.values().forEach(List::clear); }
        consume(worker, batch, actions, true, truth);
        if (policyTick == timeUs) {
            for (var entry : mounts.entrySet()) recorder.jsonLine("runs/" + runId + "/stations/" + entry.getKey() + "/pointer/pose",
                    Json.createObjectBuilder(entry.getValue().state()).add("trackingErrorDeg", trackingError(entry.getKey(), truth)).build());
            policyTick += 200_000;
        }
        recorder.checkpointIfDue();
    }
    private JsonObject metrics() throws IOException {
        var metrics = Json.createObjectBuilder().add("profile", "mission-reception.v1").add("intervalStartUs", "-1000000").add("intervalEndUs", Long.toString(stop));
        var receivers = Json.createObjectBuilder();
        for (var entry : counts.entrySet()) { long[] c = entry.getValue(); receivers.add(entry.getKey(), Json.createObjectBuilder().add("eligibleTransmissions", c[0])
                .add("rfDelivered", c[1]).add("acceptedFrames", c[2]).add("rejectedFrames", c[3]).add("pointerCommands", c[4])
                .add("analysis", reception.get(entry.getKey()).json(stop)).add("acceptedFraction", c[0] == 0 ? JsonValue.NULL : Json.createValue((double) c[2] / c[0]))); }
        var result = metrics.add("receivers", receivers).build(); recorder.json("runs/" + runId + "/metrics.json", result);
        recorder.json("runs/" + runId + "/validation.json", Json.createObjectBuilder().add("status", "complete").add("truthLeakToPolicy", false)
                .add("unavailableHardwareChannels", true).add("pointerModel", "illustrative-rate-limited.v1").build());
        recorder.checkpoint("incomplete"); return result;
    }
    private MissionFlightHook hook(Path output) {
        return new MissionFlightHook() {
            public Path outputDirectory() { return output; }
            public void start(edu.mit.rocket_team.FC.RTFC fc, info.openrocket.core.simulation.SimulationStatus status) {
                computer = fc; truth.startSimulation(status); computer.telemetry.setMissionTransport(MissionRun.this::fcTransmit);
            }
            public long nextMissionDeadlineUs() { return nextStationDeadline(); }
            public void before(long bootUs) {
                long missionUs = bootUs - 1_000_000;
                try {
                    while (nextStationDeadline() < missionUs) { moveClock(nextStationDeadline(), "Running flight computer and stations"); serviceBoundary(); }
                    moveClock(missionUs, "Running flight computer and stations");
                } catch (IOException e) { throw new UncheckedIOException(e); }
            }
            public void after(long bootUs) {
                try { if (nextStationDeadline() <= timeUs) serviceBoundary(); }
                catch (IOException e) { throw new UncheckedIOException(e); }
            }
            public void accepted(info.openrocket.core.simulation.SimulationStatus status) { truth.postStep(status); }
            public void diagnostic(String line) {
                if (computer == null) return;
                if (line.contains("action=command.") || line.contains("action=gps.zero") || line.contains("action=output.discard")) {
                    int at = line.indexOf(" action=");
                    try { event("fc-diagnostic", computer.trace.bootUs() - 1_000_000,
                            Json.createObjectBuilder().add("diagnostic", at < 0 ? line : line.substring(at + 1)).add("wireAcknowledgment", false).add("commandId", computer.telemetry.getCurrentCommandId()).build()); }
                    catch (IOException e) { throw new UncheckedIOException(e); }
                }
            }
            public void end(info.openrocket.core.simulation.SimulationStatus status, boolean complete) {
                truth.postStep(status);
                stop = Math.round(status.getSimulationTime() * 1e6) + Long.parseLong(resolved.mission().getJsonObject("execution").getString("deliveryTailUs"));
                video.end(Math.round(status.getSimulationTime() * 1e6));
                if (!complete) return;
                try { while (nextStationDeadline() <= stop) { moveClock(nextStationDeadline(), "Completing final deliveries"); serviceBoundary(); } }
                catch (IOException e) { throw new UncheckedIOException(e); }
            }
        };
    }
    private void fcTransmit(byte[] payload) {
        String vehicle = bindings.keySet().iterator().next();
        long boot = computer.trace.bootUs();
        var missing = Json.createArrayBuilder(); new TreeSet<>(computer.getMissionMissingFields()).forEach(missing::add);
        var metadata = Json.createObjectBuilder().add("packetId", vehicle + "-" + transmitted).add("transmitterId", vehicle)
                .add("txTimeUs", Long.toString(boot - 1_000_000)).add("bootTimeUs", Long.toString(boot))
                .add("acquisitionTimeUs", Long.toString(computer.getLastGpsAcquisitionUs() - 1_000_000))
                .add("barometerTimeUs", Long.toString(computer.getLastSensorAcquisitionUs() - 1_000_000)).add("missingFields", missing)
                .add("heightReference", Json.createObjectBuilder().add("convention", "relative-msl").add("baselineMslM", computer.gps.getReferenceMslM())
                        .add("epoch", computer.gps.getReferenceEpoch()).add("geoidSeparationM", MissionGeoid.bundled().separation(computer.gps.getLatitude(), computer.gps.getLongitude()))).build();
        try { transmit(vehicle, new MissionTelemetrySource.Packet(boot - 1_000_000, transmitted++, payload, metadata), truth); }
        catch (IOException e) { throw new UncheckedIOException(e); }
    }
    private void prepare() {
        var mission = resolved.mission();
        if (mission.getJsonObject("execution").containsKey("actions")) for (var action : mission.getJsonObject("execution").getJsonArray("actions")) {
            MissionActions.validate(action.asJsonObject()); tape.add(action.asJsonObject());
        }
        var uplinkProfile = mission.getJsonObject("telemetry").getJsonObject("uplink");
        uplink = new MissionLink(Json.createObjectBuilder(uplinkProfile == null ? JsonValue.EMPTY_JSON_OBJECT : uplinkProfile).add("receiverId", "fc").build(), mission.getJsonObject("execution").getString("seed"), member);
        for (var entry : mission.getJsonArray("sites")) if (entry.asJsonObject().getBoolean("enabled")) sites.put(entry.asJsonObject().getString("id"), entry.asJsonObject());
        for (var entry : mission.getJsonArray("vehicleBindings")) bindings.put(entry.asJsonObject().getString("id"), entry.asJsonObject());
        for (var entry : mission.getJsonArray("pointers")) if (entry.asJsonObject().getBoolean("enabled")) {
            var p = entry.asJsonObject(); pointers.put(p.getString("siteId"), p); mounts.put(p.getString("siteId"), new MissionPointer(p));
        }
        for (var entry : mission.getJsonArray("stationBindings")) {
            var station = entry.asJsonObject(); if (!station.getString("role").equals("telem") || !sites.containsKey(station.getString("siteId"))) continue;
            stations.put(station.getString("siteId"), station); counts.put(station.getString("siteId"), new long[5]); reception.put(station.getString("siteId"), new MissionReceptionMetrics());
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
            var outcome = links.get(site).transmit(vehicle, timeUs, packet.ordinal(), packet.payload(), codec == null ? 144 : codec.frameBytes(), tuned, beamQualified(site, truth));
            event("link-" + outcome.outcome(), timeUs, Json.createObjectBuilder(packet.metadata()).add("siteId", site).add("deliveryUs", Long.toString(outcome.deliveryUs())).build());
            if (outcome.payload() != null) {
                counts.get(site)[1]++;
                byte[] trailer = MissionTelemetrySource.trailer(sites.get(site));
                byte[] frame = codec == null ? LegacyTelemetryEnvelope.wrap(outcome.payload(), trailer)
                        : codec.frame(outcome.payload(), codec.definition().getString("receiverTrailer").equals("none") ? new byte[0] : trailer);
                if(deliveries.size()>=MAX_PENDING || deliveryBytes+frame.length>MAX_PENDING_BYTES)
                    throw new IOException("Receiver backlog exceeded 100,000 frames or 64 MiB; reduce transmission rate or link delay, or increase UART baud");
                deliveries.add(new Delivery(outcome.deliveryUs(), site, packet.ordinal(), frame, packet.metadata()));deliveryBytes+=frame.length;
            }
        }
    }
    private void consume(MissionParticipants worker, Map<String, List<JsonObject>> batch, Map<String, JsonArray> actions, boolean evaluate, MissionTruth truth) throws IOException {
        var array = Json.createArrayBuilder();
        for (String id : stations.keySet()) { var packets = Json.createArrayBuilder(); batch.get(id).forEach(packets::add);
            array.add(Json.createObjectBuilder().add("id", id).add("deliveries", packets).add("actions", actions.getOrDefault(id, JsonValue.EMPTY_JSON_ARRAY)).add("evaluate", evaluate)
                    .add("videoState", video.state(id))
                    .add("pointerPose", mounts.containsKey(id) ? mounts.get(id).state() : JsonValue.EMPTY_JSON_OBJECT)); }
        var result = worker.advance(timeUs, array.build());
        for (var action : result.getJsonArray("operatorActions")) tape.add(Json.createObjectBuilder(action.asJsonObject()).add("timeUs", Long.toString(timeUs + 1)).build());
        for (var entry : result.getJsonArray("stations")) {
            var station = entry.asJsonObject(); String id = station.getString("id");
            var mountState = mounts.get(id);
            reception.get(id).advance(timeUs, station.getBoolean("tracking"), trackingError(id, truth), mountState != null && mountState.state().getBoolean("saturated"));
            for (var sample : station.getJsonArray("samples")) reception.get(id).sample(sample.asJsonObject(), timeUs, beamQualified(id, truth));
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
        for(var value:result.getJsonArray("participantFailures")) {
            var failure=value.asJsonObject();if(!failure.getBoolean("mandatory"))continue;
            worker.pause(true,timeUs);
            while(true) {
                JsonObject selected=control.reconnectParticipant(failure);
                try{worker.reconnect(failure.getString("bindingId"),timeUs,selected);control.participantRestored();break;}
                catch(IOException e){failure=Json.createObjectBuilder(failure).add("message",e.getMessage()==null?"Reconnect failed":e.getMessage()).build();}
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
