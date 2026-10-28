package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.io.*;
import java.nio.ByteBuffer;
import java.nio.charset.*;
import java.nio.channels.FileChannel;
import java.nio.file.*;
import java.util.*;

/** Bounded, duplicate-key rejecting JSON and atomic draft persistence. Never executes assets. */
public final class MissionJson {
    public static final int MAX_BYTES = 1_048_576;
    public static final int MAX_DEPTH = 64;

    private MissionJson() { }

    public static JsonObject read(InputStream input) throws IOException {
        return parse(input.readNBytes(MAX_BYTES + 1));
    }

    public static JsonObject parse(byte[] bytes) throws IOException {
        if (bytes.length > MAX_BYTES) throw new IOException("Mission JSON exceeds 1 MiB");
        String text;
        try {
            text = StandardCharsets.UTF_8.newDecoder().onMalformedInput(CodingErrorAction.REPORT)
                    .onUnmappableCharacter(CodingErrorAction.REPORT).decode(ByteBuffer.wrap(bytes)).toString();
        } catch (CharacterCodingException e) {
            throw new IOException("Mission JSON must be UTF-8", e);
        }
        // Check depth and duplicate keys before building a recursive tree.
        var keys = new ArrayDeque<Set<String>>();
        int depth = 0;
        try (var parser = Json.createParser(new StringReader(text))) {
            while (parser.hasNext()) {
                var event = parser.next();
                switch (event) {
                    case START_OBJECT, START_ARRAY -> {
                        if (++depth > MAX_DEPTH) throw new IOException("Mission JSON exceeds depth 64");
                        keys.push(new HashSet<>());
                    }
                    case END_OBJECT, END_ARRAY -> { depth--; keys.pop(); }
                    case KEY_NAME -> {
                        if (!StandardCharsets.UTF_8.newEncoder().canEncode(parser.getString()))
                            throw new IOException("Invalid Unicode scalar in JSON key");
                        if (!keys.peek().add(parser.getString())) throw new IOException("Duplicate JSON key");
                    }
                    case VALUE_STRING -> {
                        if (!StandardCharsets.UTF_8.newEncoder().canEncode(parser.getString()))
                            throw new IOException("Invalid Unicode scalar in JSON string");
                    }
                    case VALUE_NUMBER -> {
                        if (!Double.isFinite(parser.getBigDecimal().doubleValue()))
                            throw new IOException("Non-finite JSON number");
                    }
                    default -> { }
                }
            }
            try (var reader = Json.createReader(new StringReader(text))) {
                return reader.readObject();
            }
        } catch (RuntimeException e) {
            throw new IOException("Invalid mission JSON", e);
        }
    }

    public static byte[] bytes(JsonObject json) throws IOException {
        byte[] result = (json.toString() + "\n").getBytes(StandardCharsets.UTF_8);
        parse(result); // Same limits apply to programmatically constructed objects.
        return result;
    }

    public static void writeAtomic(Path destination, JsonObject json) throws IOException {
        byte[] bytes = bytes(json);
        Path target = destination.toAbsolutePath();
        Path temp = Files.createTempFile(target.getParent(), ".mission-", ".tmp");
        try {
            try (var channel = FileChannel.open(temp, StandardOpenOption.WRITE)) {
                var buffer = ByteBuffer.wrap(bytes);
                while (buffer.hasRemaining()) channel.write(buffer);
                channel.force(true);
            }
            // Failure leaves the previous destination intact; no non-atomic fallback.
            Files.move(temp, target, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING);
        } finally {
            Files.deleteIfExists(temp);
        }
    }

    public static JsonObject resource(String name) {
        if (!name.matches("[a-z0-9.-]+\\.json")) throw new IllegalArgumentException("Invalid resource name");
        try (var input = MissionJson.class.getResourceAsStream("/mission/v1/" + name)) {
            if (input == null) throw new IOException("Missing mission contract " + name);
            return read(input);
        } catch (IOException e) {
            throw new IllegalStateException("Cannot load mission contract " + name, e);
        }
    }
}
