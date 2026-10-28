package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.channels.FileChannel;
import java.nio.file.*;
import java.util.*;

/** Append-only canonical streams with raw-before-index commits and durable prefix checkpoints. */
public final class MissionRecorder implements AutoCloseable {
    public static final long CHUNK_BYTES = 16 * 1024 * 1024;
    private final Path root;
    private final Map<String, String> hashes = new TreeMap<>();
    private final Map<String, Path> files = new TreeMap<>();
    private final Map<String, Stream> streams = new LinkedHashMap<>();
    private final Map<String, JsonObject> chunks = new TreeMap<>();
    private long sequence, lastCheckpoint = System.nanoTime();
    private boolean closed;
    private final class Stream {
        final String group, type;
        int ordinal;
        String name;
        FileOutputStream out;
        java.security.MessageDigest contentDigest;
        long length, firstTime, lastTime, firstSequence, lastSequence;
        Stream(String group, String type) { this.group = group; this.type = type; }
        void open() throws IOException {
            String extension = type.equals("index") ? "jsonl" : type;
            name = group + "/" + (type.equals("index") ? "index-" : "chunk-") + String.format(Locale.ROOT, "%05d", ordinal++) + "." + extension;
            Path file = path(name); Files.createDirectories(file.getParent()); out = new FileOutputStream(file.toFile()); files.put(name, file);
            try { contentDigest = java.security.MessageDigest.getInstance("SHA-256"); } catch (java.security.NoSuchAlgorithmException e) { throw new IllegalStateException(e); }
            length = 0; firstTime = lastTime = firstSequence = lastSequence = 0;
        }
        JsonObject descriptor() { return Json.createObjectBuilder().add("path", name).add("committedBytes", length)
                .add("firstTimeUs", Long.toString(firstTime)).add("lastTimeUs", Long.toString(lastTime))
                .add("firstSequence", Long.toString(firstSequence)).add("lastSequence", Long.toString(lastSequence))
                .add("schema", 1).add("sha256", hash()).build(); }
        String hash() { try { return HexFormat.of().formatHex(((java.security.MessageDigest) contentDigest.clone()).digest()); } catch (CloneNotSupportedException e) { throw new IllegalStateException(e); } }
        long write(long time, byte[] bytes) throws IOException {
            if (bytes.length > CHUNK_BYTES) throw new IOException("Canonical record exceeds chunk limit");
            if (out == null) open();
            if (length + bytes.length > CHUNK_BYTES) { out.getChannel().force(true); out.close(); chunks.put(name, descriptor()); open(); }
            if (length == 0) { firstTime = time; firstSequence = sequence; }
            long offset = length; out.write(bytes); contentDigest.update(bytes); length += bytes.length; lastTime = time; lastSequence = sequence++;
            return offset;
        }
    }
    public MissionRecorder(Path root, JsonObject mission) throws IOException {
        Files.createDirectory(root); this.root = root.toAbsolutePath(); json("mission.json", mission); checkpoint("incomplete");
    }
    public Path root() { return root; }
    private Path path(String name) throws IOException {
        if (!name.matches("[A-Za-z0-9][A-Za-z0-9._-]*(?:/[A-Za-z0-9][A-Za-z0-9._-]*)*")) throw new IOException("Invalid recorder path");
        return root.resolve(name);
    }
    public synchronized void json(String name, JsonObject value) throws IOException {
        Path file = path(name); Files.createDirectories(file.getParent()); MissionJson.writeAtomic(file, value);
        try (var channel = FileChannel.open(file, StandardOpenOption.WRITE)) { channel.force(true); } files.put(name, file); hashes.put(name, MissionFlightSnapshot.hash(file));
    }
    public synchronized void asset(String name, Path source) throws IOException {
        Path target = path(name); Files.createDirectories(target.getParent()); Files.copy(source, target);
        try (var channel = FileChannel.open(target, StandardOpenOption.WRITE)) { channel.force(true); } files.put(name, target); hashes.put(name, MissionFlightSnapshot.hash(target));
    }
    public synchronized void line(String group, String type, long timeUs, String text) throws IOException {
        if (closed) throw new IOException("Recorder is closed");
        streams.computeIfAbsent(group + ":" + type, ignored -> new Stream(group, type)).write(timeUs, text.getBytes(StandardCharsets.UTF_8));
    }
    public synchronized void event(String run, JsonObject event) throws IOException { jsonLine("runs/" + run + "/events", event); }
    public synchronized void jsonLine(String group, JsonObject event) throws IOException {
        line(group, "jsonl", Long.parseLong(event.getString("timeUs")), event.toString() + "\n");
    }
    public synchronized JsonObject raw(String group, long timeUs, byte[] bytes, JsonObject metadata) throws IOException {
        if (closed) throw new IOException("Recorder is closed");
        var stream = streams.computeIfAbsent(group + ":bin", ignored -> new Stream(group, "bin"));
        long offset = stream.write(timeUs, bytes);
        var index = Json.createObjectBuilder(metadata).add("timeUs", Long.toString(timeUs)).add("file", stream.name)
                .add("offset", offset).add("length", bytes.length).add("sha256", MissionWorker.digest(bytes)).build();
        line(group, "index", timeUs, index.toString() + "\n"); return index;
    }
    public synchronized void checkpointIfDue() throws IOException { if (System.nanoTime() - lastCheckpoint >= 1_000_000_000L) checkpoint("incomplete"); }
    public synchronized void checkpoint(String status) throws IOException {
        // Binary data reaches durable storage before any committed index may reference it.
        for (var s : streams.values()) if (s.out != null && s.type.equals("bin")) s.out.getChannel().force(true);
        for (var s : streams.values()) if (s.out != null) { s.out.getChannel().force(true); chunks.put(s.name, s.descriptor()); }
        var inventory = Json.createArrayBuilder();
        for (var entry : files.entrySet()) {
            JsonObject chunk = chunks.get(entry.getKey());
            inventory.add(chunk == null ? Json.createObjectBuilder().add("path", entry.getKey()).add("committedBytes", Files.size(entry.getValue())).add("sha256", hashes.get(entry.getKey())).build() : chunk);
        }
        var checkpoint = Json.createObjectBuilder().add("format", "openrocket.mission-checkpoint").add("version", 1)
                .add("status", status).add("chunkBytes", CHUNK_BYTES).add("sequence", Long.toString(sequence)).add("files", inventory).build();
        Path file = root.resolve("checkpoint.json"); MissionJson.writeAtomic(file, checkpoint, MissionArchive.MAX_MANIFEST);
        try (var channel = FileChannel.open(file, StandardOpenOption.WRITE)) { channel.force(true); }
        lastCheckpoint = System.nanoTime();
    }
    public synchronized void finish(Path destination, String status) throws IOException {
        checkpoint(status); closeStreams();
        files.put("checkpoint.json", root.resolve("checkpoint.json"));
        MissionArchive.write(destination, MissionArchive.RESULT, status, files);
        closed = true;
    }
    private void closeStreams() throws IOException {
        IOException failure = null;
        for (var stream : streams.values()) if (stream.out != null) {
            try { stream.out.close(); } catch (IOException e) { failure = e; } finally { stream.out = null; }
        }
        if (failure != null) throw failure;
    }
    @Override public synchronized void close() throws IOException {
        if (!closed) { checkpoint("incomplete"); closeStreams(); closed = true; }
    }
}
