package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import org.apache.commons.compress.archivers.zip.ZipArchiveEntry;
import org.apache.commons.compress.archivers.zip.ZipFile;
import java.io.*;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.security.*;
import java.util.*;
import java.util.zip.*;

/** ZIP64 inventory, atomic publication and bounded lazy verified reads for setups and results. */
public final class MissionArchive implements AutoCloseable {
    public static final int MAX_ENTRIES = 100_000, MAX_MANIFEST = 32 * 1024 * 1024;
    public static final long MAX_BYTES = 64L * 1024 * 1024 * 1024;
    public static final String SETUP = "openrocket.mission-setup", RESULT = "openrocket.mission-result";
    private final ZipFile zip;
    private final JsonObject manifest;
    private final Map<String, ZipArchiveEntry> entries = new LinkedHashMap<>();
    private final Map<String, JsonObject> inventory = new LinkedHashMap<>();

    public MissionArchive(Path path) throws IOException {
        MissionZipPreflight.check(path);
        zip = new ZipFile(path.toFile());
        try {
            long total = 0; var folded = new HashSet<String>();
            var enumeration = zip.getEntries();
            while (enumeration.hasMoreElements()) {
                var entry = enumeration.nextElement(); String name = entry.getName(); safeName(name);
                if (entries.size() == MAX_ENTRIES || entry.isDirectory() || entry.isUnixSymlink() || ((entry.getUnixMode() & 0170000) != 0 && (entry.getUnixMode() & 0170000) != 0100000) || !zip.canReadEntryData(entry)
                        || entry.getSize() < 0 || entry.getSize() > MAX_BYTES || entry.getCompressedSize() < 0
                        || !folded.add(name.toLowerCase(Locale.ROOT)) || entries.putIfAbsent(name, entry) != null)
                    throw new IOException("Invalid, duplicate or unsupported archive entry: " + name);
                total = Math.addExact(total, entry.getSize());
                if (total > MAX_BYTES) throw new IOException("Mission archive exceeds 64 GiB uncompressed");
            }
            for (String name : entries.keySet()) {
                int slash = name.indexOf('/');
                while (slash >= 0) {
                    if (folded.contains(name.substring(0, slash).toLowerCase(Locale.ROOT))) throw new IOException("Archive file/directory conflict");
                    slash = name.indexOf('/', slash + 1);
                }
            }
            var meta = entries.get("manifest.json");
            if (meta == null || meta.getSize() > MAX_MANIFEST) throw new IOException("Missing or oversized mission inventory");
            try (var input = zip.getInputStream(meta)) { manifest = MissionJson.parse(input.readNBytes(MAX_MANIFEST + 1), MAX_MANIFEST); }
            if (!Set.of(SETUP, RESULT).contains(manifest.getString("format", "")) || manifest.getInt("version", -1) != 1)
                throw new IOException("Unsupported mission archive");
            if (!Set.of("complete", "incomplete", "cancelled", "failed", "complete-with-failures").contains(manifest.getString("status", "")))
                throw new IOException("Unknown archive completion status");
            for (var value : manifest.getJsonArray("files")) {
                var file = value.asJsonObject(); String name = file.getString("path"); safeName(name);
                var entry = entries.get(name);
                if (name.equals("manifest.json") || inventory.putIfAbsent(name, file) != null || entry == null
                        || file.getJsonNumber("length").longValueExact() != entry.getSize() || !file.getString("sha256").matches("[0-9a-f]{64}"))
                    throw new IOException("Inconsistent mission inventory: " + name);
            }
            if (inventory.size() != entries.size() - 1 || !inventory.containsKey("mission.json")) throw new IOException("Unlisted or missing mission entries");
        } catch (IOException | RuntimeException e) { zip.close(); throw new IOException("Cannot open mission archive: " + e.getMessage(), e); }
    }
    public JsonObject manifest() { return manifest; }
    public Set<String> names() { return Collections.unmodifiableSet(inventory.keySet()); }

    /** Hash is checked at EOF, including when a consumer closes early. Nothing is executed by a reader. */
    public InputStream open(String name) throws IOException {
        var item = inventory.get(name); if (item == null) throw new IOException("Unknown archive entry: " + name);
        return new FilterInputStream(zip.getInputStream(entries.get(name))) {
            final MessageDigest digest = digest(); long count; boolean checked;
            private void verify() throws IOException {
                if (checked) return;
                checked = true;
                if (count != item.getJsonNumber("length").longValueExact() || !HexFormat.of().formatHex(digest.digest()).equals(item.getString("sha256")))
                    throw new IOException("Archive entry hash/length mismatch: " + name);
            }
            @Override public int read() throws IOException { byte[] b = new byte[1]; return read(b, 0, 1) < 0 ? -1 : b[0] & 255; }
            @Override public int read(byte[] b, int off, int len) throws IOException {
                int n = in.read(b, off, len);
                if (n < 0) verify();
                else { count += n; if (count > item.getJsonNumber("length").longValueExact()) throw new IOException("Archive expands beyond declared length"); digest.update(b, off, n); }
                return n;
            }
            @Override public long skip(long n) throws IOException {
                byte[] b = new byte[65536]; long skipped = 0;
                while (skipped < n) { int count = read(b, 0, (int) Math.min(b.length, n - skipped)); if (count < 0) break; skipped += count; }
                return skipped;
            }
            @Override public void close() throws IOException {
                try { if (!checked) { byte[] b = new byte[65536]; while (read(b) >= 0) { } } }
                finally { in.close(); }
            }
        };
    }
    public byte[] read(String name, int maximum) throws IOException {
        var item = inventory.get(name);
        if (item == null || item.getJsonNumber("length").longValueExact() > maximum) throw new IOException("Entry exceeds read limit: " + name);
        try (var input = open(name)) { return input.readNBytes(maximum + 1); }
    }
    public void verifyAll() throws IOException {
        byte[] bytes = new byte[65536];
        for (String name : names()) try (var input = open(name)) { while (input.read(bytes) >= 0) { } }
    }
    public void extract(Path destination) throws IOException {
        Files.createDirectory(destination); // Never overwrite arbitrary existing files or traverse existing links.
        for (String name : names()) {
            Path file = destination.resolve(name); Files.createDirectories(file.getParent());
            try (var input = open(name); var output = Files.newOutputStream(file, StandardOpenOption.CREATE_NEW)) { input.transferTo(output); }
        }
    }
    @Override public void close() throws IOException { zip.close(); }

    public static void write(Path destination, String format, String status, Map<String, Path> files) throws IOException {
        if (!Set.of(SETUP, RESULT).contains(format) || files.size() >= MAX_ENTRIES || !files.containsKey("mission.json")) throw new IOException("Invalid archive inventory");
        var inventory = Json.createArrayBuilder(); long bytes = 0;
        for (var entry : new TreeMap<>(files).entrySet()) {
            safeName(entry.getKey()); if (entry.getKey().equals("manifest.json")) throw new IOException("Manifest is generated by the writer");
            Path file = entry.getValue();
            if (!Files.isRegularFile(file, LinkOption.NOFOLLOW_LINKS)) throw new IOException("Archive source must be a regular file");
            long length = Files.size(file); bytes = Math.addExact(bytes, length);
            if (bytes > MAX_BYTES) throw new IOException("Mission archive exceeds 64 GiB");
            inventory.add(Json.createObjectBuilder().add("path", entry.getKey()).add("length", length).add("sha256", MissionFlightSnapshot.hash(file)).add("type", type(entry.getKey())));
        }
        var manifest = Json.createObjectBuilder().add("format", format).add("version", 1).add("status", status).add("compression", "deflate-3")
                .add("files", inventory).build().toString().getBytes(StandardCharsets.UTF_8);
        if (manifest.length > MAX_MANIFEST) throw new IOException("Archive inventory is too large");
        Path target = destination.toAbsolutePath();
        if (Files.getFileStore(target.getParent()).getUsableSpace() < bytes + manifest.length + 1024 * 1024)
            throw new IOException("Insufficient free space for the finalized mission archive");
        Path temporary = Files.createTempFile(target.getParent(), ".mission-finalizing-", ".zip");
        boolean complete = false;
        try {
            try (var zip = new ZipOutputStream(Files.newOutputStream(temporary), StandardCharsets.UTF_8)) {
                zip.setLevel(3);
                for (var item : new TreeMap<>(files).entrySet()) {
                    var entry = new ZipEntry(item.getKey()); entry.setTime(0);
                    if (Set.of("png", "jpg", "jpeg", "mp4", "zip", "rktflight").contains(type(item.getKey()))) {
                        var crc = new CRC32(); try (var input = Files.newInputStream(item.getValue())) { byte[] b = new byte[65536]; int n; while ((n = input.read(b)) >= 0) crc.update(b, 0, n); }
                        entry.setMethod(ZipEntry.STORED); entry.setSize(Files.size(item.getValue())); entry.setCrc(crc.getValue());
                    }
                    zip.putNextEntry(entry); Files.copy(item.getValue(), zip); zip.closeEntry();
                }
                var meta = new ZipEntry("manifest.json"); meta.setTime(0); zip.putNextEntry(meta); zip.write(manifest); zip.closeEntry();
            }
            try (var channel = FileChannel.open(temporary, StandardOpenOption.WRITE)) { channel.force(true); }
            try (var reader = new MissionArchive(temporary)) { reader.verifyAll(); }
            Files.move(temporary, target, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING); complete = true;
        } finally { if (!complete) Files.deleteIfExists(temporary); }
    }
    private static MessageDigest digest() { try { return MessageDigest.getInstance("SHA-256"); } catch (NoSuchAlgorithmException e) { throw new IllegalStateException(e); } }
    private static String type(String name) { int dot = name.lastIndexOf('.'); return dot < 0 ? "bytes" : name.substring(dot + 1).toLowerCase(Locale.ROOT); }
    static void safeName(String name) throws IOException {
        if (name.length() > 1024 || !name.matches("[a-zA-Z0-9][a-zA-Z0-9._-]*(?:/[a-zA-Z0-9][a-zA-Z0-9._-]*)*"))
            throw new IOException("Unsafe mission archive path");
        for (String part : name.split("/")) {
            String stem = part.split("\\.", 2)[0].toUpperCase(Locale.ROOT);
            if (part.endsWith(".") || Set.of("CON", "PRN", "AUX", "NUL", "COM1", "COM2", "COM3", "COM4", "COM5", "COM6", "COM7", "COM8", "COM9", "LPT1", "LPT2", "LPT3", "LPT4", "LPT5", "LPT6", "LPT7", "LPT8", "LPT9").contains(stem))
                throw new IOException("Non-portable mission archive path");
        }
    }
}
