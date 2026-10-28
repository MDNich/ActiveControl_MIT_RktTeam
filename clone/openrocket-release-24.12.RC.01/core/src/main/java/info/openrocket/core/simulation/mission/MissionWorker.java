package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.io.*;
import java.net.*;
import java.nio.file.*;
import java.security.*;
import java.util.*;

/** One authenticated, bounded lockstep Python station process per ensemble member. */
public final class MissionWorker implements AutoCloseable {
    private final String missionId, runId, memberId;
    private final String sessionId = UUID.randomUUID().toString(), connectionId = UUID.randomUUID().toString();
    private final Path manifest;
    private Process process;
    private Socket socket;
    private InputStream input;
    private OutputStream output;
    private long sequence, replySequence = -1;
    private JsonObject provenance;

    public MissionWorker(Path privateDirectory, String missionId, String runId, String memberId) throws IOException {
        this(privateDirectory, missionId, runId, memberId, null, null);
    }
    public MissionWorker(Path privateDirectory, String missionId, String runId, String memberId,
                         JsonObject application, JsonObject preferences) throws IOException {
        this.missionId = missionId; this.runId = runId; this.memberId = memberId;
        Files.createDirectories(privateDirectory);
        try { Files.setPosixFilePermissions(privateDirectory, java.nio.file.attribute.PosixFilePermissions.fromString("rwx------")); }
        catch (UnsupportedOperationException ignored) { }
        manifest = privateDirectory.resolve("launch-private.json");
        byte[] bundle;
        try (var in = MissionWorker.class.getResourceAsStream("/mission/worker/station-worker.zip")) {
            if (in == null) throw new IOException("Bundled station worker is missing");
            bundle = in.readNBytes(4 * 1024 * 1024);
        }
        JsonObject metadata;
        try (var in = MissionWorker.class.getResourceAsStream("/mission/worker/worker.json")) { metadata = MissionJson.read(in); }
        if (!digest(bundle).equals(metadata.getString("sha256"))) throw new IOException("Bundled station worker hash mismatch");
        Path zip = privateDirectory.resolve("station-worker.zip"); Files.write(zip, bundle);
        Path python = application == null ? interpreter() : null;
        byte[] random = new byte[32]; new SecureRandom().nextBytes(random);
        String credential = HexFormat.of().formatHex(random);
        try (var server = new ServerSocket(0, 1, InetAddress.getByName("127.0.0.1"))) {
            server.setSoTimeout(application == null ? 10_000 : 30_000);
            var launch = envelope("hello", -1_000_000, JsonValue.EMPTY_JSON_OBJECT);
            MissionJson.writeAtomic(manifest, Json.createObjectBuilder(launch)
                    .add("host", "127.0.0.1").add("port", server.getLocalPort()).add("credential", credential)
                    .addAll(Json.createObjectBuilder(application == null ? JsonValue.EMPTY_JSON_OBJECT : application)).build());
            try { Files.setPosixFilePermissions(manifest, java.nio.file.attribute.PosixFilePermissions.fromString("rw-------")); }
            catch (UnsupportedOperationException ignored) { }
            var builder = application == null ? new ProcessBuilder(python.toString(), "-I", "-c",
                    "import sys,runpy;sys.path.insert(0,sys.argv.pop(1));runpy.run_module('rocket_gnc_monitor.mission_worker',run_name='__main__')",
                    zip.toString(), manifest.toString()) : applicationProcess(preferences, manifest);
            builder.redirectOutput(privateDirectory.resolve("worker.stdout.log").toFile());
            builder.redirectError(privateDirectory.resolve("worker.stderr.log").toFile());
            process = builder.start(); socket = server.accept(); socket.setSoTimeout(10_000); socket.setTcpNoDelay(true);
            input = new BufferedInputStream(socket.getInputStream()); output = new BufferedOutputStream(socket.getOutputStream());
            var hello = request("hello", -1_000_000, Json.createObjectBuilder().add("credential", credential).build());
            if (!hello.getJsonArray("capabilities").contains(Json.createValue("mission-clock.v1")))
                throw new IOException("Worker lacks the mission clock capability");
            provenance = Json.createObjectBuilder(metadata).add("runtime", hello).add("developmentInterpreter",
                    System.getProperty("openrocket.mission.python") != null).build();
            Files.deleteIfExists(manifest); // Process has read the credential; it never enters a result archive.
        } catch (IOException | RuntimeException e) { close(); throw e; }
    }

    private static ProcessBuilder applicationProcess(JsonObject preferences, Path manifest) throws IOException {
        if (preferences.containsKey("sourceProject") && preferences.containsKey("pythonExecutable")) {
            Path source = Path.of(preferences.getString("sourceProject")).resolve("src");
            Path python = Path.of(preferences.getString("pythonExecutable"));
            if (!Files.isRegularFile(source.resolve("rocket_gnc_monitor/mission_application.py")) || !Files.isExecutable(python))
                throw new IOException("Selected Telemetry UI source/interpreter does not support Mission Simulation");
            return new ProcessBuilder(python.toString(), "-I", "-c",
                    "import sys,runpy;sys.path.insert(0,sys.argv.pop(1));runpy.run_module('rocket_gnc_monitor',run_name='__main__')",
                    source.toString(), "--mission-manifest", manifest.toString());
        }
        if (preferences.containsKey("packagedApp")) {
            Path app = Path.of(preferences.getString("packagedApp"));
            if (Files.isDirectory(app) && app.toString().endsWith(".app")) {
                try (var files = Files.list(app.resolve("Contents/MacOS"))) {
                    var executables = files.filter(Files::isExecutable).filter(Files::isRegularFile).toList();
                    if (executables.size() != 1) throw new IOException("Select the application executable inside this bundle");
                    app = executables.get(0);
                }
            }
            if (Files.isExecutable(app)) return new ProcessBuilder(app.toString(), "--mission-manifest", manifest.toString());
        }
        throw new IOException("Select a Mission-capable Telemetry UI application or source project and Python in Connections");
    }
    public static Path interpreter() throws IOException {
        String override = System.getProperty("openrocket.mission.python");
        if (override != null) {
            Path path = Path.of(override).toAbsolutePath();
            if (!Files.isRegularFile(path) || !Files.isExecutable(path)) throw new IOException("Configured development Python is unavailable");
            return path;
        }
        // Installers supply this location alongside the Java runtime, never through a portable setup.
        String home = System.getProperty("openrocket.mission.runtime");
        if (home != null) {
            Path path = Path.of(home).resolve(System.getProperty("os.name").startsWith("Windows") ? "python.exe" : "bin/python3");
            if (Files.isExecutable(path)) return path;
        }
        throw new IOException("Bundled Mission Python runtime is unavailable; repair the installation");
    }

    public JsonObject provenance() { return provenance; }
    public JsonObject configure(JsonArray stations) throws IOException {
        return request("configure", -1_000_000, Json.createObjectBuilder().add("stations", stations).build());
    }
    public JsonObject advance(long timeUs, JsonArray stations) throws IOException {
        long sent = sequence;
        JsonObject response = request("advance", timeUs, Json.createObjectBuilder().add("stations", stations).build());
        if (!response.getString("acknowledgedSequence").equals(Long.toString(sent))) throw new IOException("Worker acknowledged the wrong boundary");
        return response;
    }
    public synchronized JsonObject request(String type, long timeUs, JsonObject body) throws IOException {
        MissionWire.write(output, envelope(type, timeUs, body)); output.flush(); sequence++;
        JsonObject response = MissionWire.read(input);
        if (response == null) throw new EOFException("Station worker disconnected");
        for (String key : List.of("sessionId", "missionId", "runId", "memberId", "participantId", "connectionId"))
            if (!response.getString(key).equals(envelope(type, timeUs, body).getString(key))) throw new IOException("Worker identity mismatch");
        long received = Long.parseLong(response.getString("sequence"));
        if (received <= replySequence || !response.getString("missionTimeUs").equals(Long.toString(timeUs)))
            throw new IOException("Worker sequence or mission clock mismatch");
        replySequence = received;
        if (response.getString("type").equals("error")) throw new IOException("Station worker: " + response.getJsonObject("body").getString("message"));
        String expected = switch (type) { case "hello" -> "capabilities"; case "configure" -> "ready";
            case "advance" -> "advanced"; case "close" -> "closed"; default -> "result"; };
        if (!response.getString("type").equals(expected)) throw new IOException("Unexpected worker response");
        return response.getJsonObject("body");
    }
    private JsonObject envelope(String type, long timeUs, JsonObject body) {
        return Json.createObjectBuilder().add("protocol", "rt-mission.v1").add("type", type).add("messageId", "request-" + sequence)
                .add("sessionId", sessionId).add("missionId", missionId).add("runId", runId).add("memberId", memberId)
                .add("participantId", "stations").add("connectionId", connectionId).add("sequence", Long.toString(sequence))
                .add("missionTimeUs", Long.toString(timeUs)).add("body", body).build();
    }
    public static String digest(byte[] bytes) {
        try { return HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(bytes)); }
        catch (NoSuchAlgorithmException e) { throw new IllegalStateException(e); }
    }
    @Override public void close() {
        try { if (socket != null && !socket.isClosed() && input != null) {
            request("close", 0, JsonValue.EMPTY_JSON_OBJECT);
            if (process != null) process.waitFor(5, java.util.concurrent.TimeUnit.SECONDS);
        } } catch (IOException | RuntimeException ignored) { }
        catch (InterruptedException e) { Thread.currentThread().interrupt(); }
        try { if (socket != null) socket.close(); } catch (IOException ignored) { }
        if (process != null) { process.destroy(); try { if (!process.waitFor(2, java.util.concurrent.TimeUnit.SECONDS)) process.destroyForcibly(); }
            catch (InterruptedException e) { Thread.currentThread().interrupt(); process.destroyForcibly(); } }
        try { Files.deleteIfExists(manifest); } catch (IOException ignored) { }
    }
}
