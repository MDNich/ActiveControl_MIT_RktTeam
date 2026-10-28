package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;

/** Java-owned jobs survive browser reconnects. Restarted processes recover prefixes, never resume an FC. */
public final class MissionJobs implements MissionApi.Jobs {
    private final Path root;
    private final Map<String, Job> jobs = new ConcurrentHashMap<>();
    private final ExecutorService executor = Executors.newFixedThreadPool(2, r -> { var t = new Thread(r, "mission-run"); t.setDaemon(true); return t; });
    public MissionJobs(Path root) throws IOException { this.root = root; Files.createDirectories(root.resolve("jobs")); }
    @Override public JsonObject capabilities() {
        try { MissionWorker.interpreter(); return Json.createObjectBuilder().add("available", true).add("standard", true).add("coupled", true)
                .add("video", false).add("ensemble", false).add("workerProtocol", "rt-mission.v1").build(); }
        catch (IOException e) { return Json.createObjectBuilder().add("available", false).add("reason", e.getMessage()).build(); }
    }
    @Override public JsonObject start(MissionStore.Resolved resolved) throws IOException {
        admit(resolved.mission());
        var job = new Job(resolved); jobs.put(job.id, job); job.persist(); executor.submit(job::run); return job.snapshot();
    }
    private static void admit(JsonObject mission) {
        if (!mission.getJsonObject("video").getJsonArray("streams").isEmpty()) throw new IllegalArgumentException("Video transmission is being integrated; choose zero streams for this headless increment");
        var ensemble = mission.getJsonObject("flight").getJsonObject("editor").getJsonObject("options").getJsonObject("ensemble");
        if (ensemble.getString("enabled", "false").equals("true")) throw new IllegalArgumentException("Mission ensembles are being integrated; choose a single flight in this increment");
        boolean interactive = mission.getJsonArray("stationBindings").stream().anyMatch(v -> v.asJsonObject().getString("owner").equals("interactive-ui"));
        if (interactive && !mission.getJsonObject("execution").getString("clockMode", "lockstep").equals("paced"))
            throw new IllegalArgumentException("Interactive station control requires paced execution");
        for (var entry : mission.getJsonArray("pointers")) if (entry.asJsonObject().getString("trackingPolicy").equals("truth-benchmark.v1"))
            throw new IllegalArgumentException("Truth benchmark is an analysis-only mode; choose a receiver tracking policy");
    }
    @Override public JsonObject request(String id, String action, JsonObject body) throws IOException {
        if (!id.matches("[0-9a-f-]{36}")) throw new IllegalArgumentException("Invalid job ID");
        Job job = jobs.get(id);
        if (job == null) {
            Path file = root.resolve("jobs").resolve(id).resolve("status.json");
            if (!Files.isRegularFile(file)) throw new IllegalArgumentException("Unknown mission job");
            try (var input = Files.newInputStream(file)) {
                var saved = MissionJson.read(input);
                if (!Set.of("complete", "failed", "cancelled", "complete-with-failures").contains(saved.getString("status")))
                    saved = Json.createObjectBuilder(saved).add("status", "incomplete").add("phase", "Interrupted run; committed staging is available for recovery").build();
                if (!Set.of("status", "results").contains(action)) throw new IllegalArgumentException("A restarted runner can reopen results but cannot resume this run");
                return saved;
            }
        }
        synchronized (job) {
            switch (action) {
                case "status": case "results": return job.snapshot();
                case "pause": if (job.status.equals("running")) { job.paused = true; job.status = "paused"; } break;
                case "resume": if (job.paused) { job.paused = false; job.status = "running"; job.notifyAll(); } break;
                case "cancel": if (!job.terminal()) { job.cancelled = true; job.paused = false; job.notifyAll(); } break;
                case "action": {
                    if (job.terminal() || job.status.equals("finalizing")) throw new IllegalArgumentException("Run no longer accepts actions");
                    MissionActions.validate(body);
                    var queuedAction = Json.createObjectBuilder(body).add("id", "operator-" + UUID.randomUUID()).build();
                    if (!job.actions.offer(queuedAction)) throw new IllegalArgumentException("Mission command queue is full; pause and let it drain before adding commands");
                    return Json.createObjectBuilder().add("accepted", true).add("id", queuedAction.getString("id")).build();
                }
                case "events": return Json.createObjectBuilder().add("events", Json.createArrayBuilder(job.notifications)).add("sequence", job.notificationSequence).build();
                default: throw new IllegalArgumentException("Unknown mission job operation");
            }
            job.persist(); return job.snapshot();
        }
    }
    private final class Job implements MissionRun.Control {
        final String id = UUID.randomUUID().toString();
        final MissionStore.Resolved resolved;
        final Path directory, staging, archive;
        final ArrayBlockingQueue<JsonObject> actions = new ArrayBlockingQueue<>(256);
        final ArrayDeque<JsonObject> notifications = new ArrayDeque<>();
        long notificationSequence, timeUs = -1_000_000, endUs, lastPersist, wallStart, pausedNanos;
        String status = "starting", phase = "Preparing mission", failure = "";
        boolean paused, cancelled;
        JsonObject metrics = JsonValue.EMPTY_JSON_OBJECT;
        Job(MissionStore.Resolved resolved) throws IOException {
            this.resolved = resolved; directory = root.resolve("jobs").resolve(id); Files.createDirectory(directory);
            staging = directory.resolve("staging"); archive = directory.resolve("mission.rktmission");
        }
        synchronized boolean terminal() { return Set.of("complete", "complete-with-failures", "failed", "cancelled").contains(status); }
        synchronized JsonObject snapshot() { return Json.createObjectBuilder().add("id", id).add("missionId", MissionStore.id(resolved.mission()))
                .add("status", status).add("phase", phase).add("timeUs", Long.toString(timeUs)).add("endUs", Long.toString(endUs))
                .add("failure", failure).add("metrics", metrics).add("staging", staging.toString())
                .add("result", Files.isRegularFile(archive) ? archive.toString() : "").add("notificationSequence", notificationSequence).build(); }
        synchronized void persist() throws IOException { MissionJson.writeAtomic(directory.resolve("status.json"), snapshot()); lastPersist = System.nanoTime(); }
        @Override public JsonArray drainActions(int maximum) {
            var result = Json.createArrayBuilder();
            // Leave room for warmup/start actions in the same bounded IPC batch.
            for (int i = 0; i < Math.min(200, maximum); i++) { var action = actions.poll(); if (action == null) break; result.add(action); }
            return result.build();
        }
        @Override public synchronized void check() {
            long begin = System.nanoTime(); boolean waited = false;
            while (paused && !cancelled) try { waited = true; wait(100); } catch (InterruptedException e) { Thread.currentThread().interrupt(); throw new CancellationException("Mission interrupted"); }
            if (waited) pausedNanos += System.nanoTime() - begin;
            if (cancelled || Thread.currentThread().isInterrupted()) throw new CancellationException("Mission cancelled");
        }
        @Override public void progress(long time, long end, String description) {
            synchronized (this) {
                timeUs = time; endUs = end;
                if (!phase.equals(description)) { phase = description; notifications.add(Json.createObjectBuilder().add("sequence", notificationSequence++).add("message", description).build()); if (notifications.size() > 256) notifications.removeFirst(); }
            }
            if (end > 0 && resolved.mission().getJsonObject("execution").getString("clockMode", "lockstep").equals("paced")) {
                if (wallStart == 0) wallStart = System.nanoTime() - (time + 1_000_000) * 1000;
                while (true) { check(); long remaining = wallStart + pausedNanos + (time + 1_000_000) * 1000 - System.nanoTime(); if (remaining <= 0) break;
                    try { Thread.sleep(Math.min(50, Math.max(1, remaining / 1_000_000))); } catch (InterruptedException e) { Thread.currentThread().interrupt(); throw new CancellationException(); } }
            }
            if (System.nanoTime() - lastPersist > 1_000_000_000L) try { persist(); } catch (IOException e) { throw new UncheckedIOException(e); }
        }
        void run() {
            MissionRecorder recorder = null;
            try {
                synchronized (this) { status = "running"; persist(); }
                recorder = new MissionRecorder(staging, resolved.mission());
                String hash = resolved.mission().getJsonObject("flight").getString("snapshotHash");
                try (var files = Files.list(resolved.flightDirectory())) { for (var file : files.toList()) recorder.asset("assets/" + hash + "/" + file.getFileName(), file); }
                JsonObject preferences = JsonValue.EMPTY_JSON_OBJECT;
                if (Files.isRegularFile(root.resolve("machine-preferences.json"))) try (var in = Files.newInputStream(root.resolve("machine-preferences.json"))) { preferences = MissionJson.read(in); }
                metrics = new MissionRun(resolved, recorder, "member-0", 0, this).applicationPreferences(preferences).execute(directory.resolve("private-worker"));
                synchronized (this) { status = "finalizing"; phase = "Verifying and saving the result bundle"; persist(); }
                recorder.finish(archive, "complete");
                synchronized (this) { status = "complete"; phase = "Mission complete"; }
            } catch (Exception e) {
                synchronized (this) { status = e instanceof CancellationException ? "cancelled" : "failed"; failure = e.getMessage() == null ? e.getClass().getSimpleName() : e.getMessage(); phase = "Committed results preserved"; }
                if (recorder != null) try { recorder.json("failure.json", Json.createObjectBuilder().add("status", status).add("reason", failure).build()); recorder.finish(archive, status); }
                catch (IOException finalization) { synchronized (this) { failure += "; bundle finalization failed: " + finalization.getMessage(); } }
            } finally {
                if (recorder != null) try { recorder.close(); } catch (IOException e) { synchronized (this) { status = "failed"; failure += "; checkpoint failed: " + e.getMessage(); } }
                try { persist(); } catch (IOException ignored) { /* The last durable status and staging remain discoverable. */ }
            }
        }
    }
    @Override public void close() { for (var job : jobs.values()) synchronized (job) { job.cancelled = true; job.paused = false; job.notifyAll(); } executor.shutdown(); }
}
