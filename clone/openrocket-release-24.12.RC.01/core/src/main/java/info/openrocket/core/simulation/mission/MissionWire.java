package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.io.*;
import java.util.Base64;

/** rt-mission.v1 framing only. This class opens no sockets or devices. */
public final class MissionWire {
    private static final JsonObject SCHEMA = MissionJson.resource("ipc.schema.json");
    public static final int MAX_CHUNK_BYTES = 65_536;
    private MissionWire() { }

    public static void validate(JsonObject message) {
        var issues = MissionSchema.validate(SCHEMA, message);
        if (!issues.isEmpty()) throw new IllegalArgumentException(issues.toString());
        try {
            Long.parseLong(message.getString("missionTimeUs"));
            Long.parseLong(message.getString("sequence"));
        } catch (NumberFormatException e) { throw new IllegalArgumentException("IPC integer out of range", e); }
        String type = message.getString("type");
        if (type.equals("rx-bytes") || type.equals("tx-bytes")) {
            JsonObject body = message.getJsonObject("body");
            if (!(body.get("data") instanceof JsonString data)) throw new IllegalArgumentException("Missing byte data");
            String encoded = data.getString();
            if (encoded.length() > ((MAX_CHUNK_BYTES + 2) / 3) * 4) throw new IllegalArgumentException("Byte chunk too large");
            byte[] bytes;
            try { bytes = Base64.getDecoder().decode(encoded); }
            catch (IllegalArgumentException e) { throw new IllegalArgumentException("Invalid base64", e); }
            if (bytes.length > MAX_CHUNK_BYTES || !Base64.getEncoder().encodeToString(bytes).equals(encoded))
                throw new IllegalArgumentException("Byte chunk must use bounded canonical base64");
        }
    }

    /** Returns null only for a clean EOF between frames, never for a truncated frame. */
    public static JsonObject read(InputStream input) throws IOException {
        int first = input.read();
        if (first < 0) return null;
        byte[] rest = input.readNBytes(3);
        if (rest.length != 3) throw new EOFException("Truncated mission frame header");
        long size = ((long) first << 24) | ((rest[0] & 255L) << 16) | ((rest[1] & 255L) << 8) | (rest[2] & 255L);
        if (size < 1 || size > MissionJson.MAX_BYTES) throw new IOException("Mission frame size out of bounds");
        byte[] body = input.readNBytes((int) size);
        if (body.length != size) throw new EOFException("Truncated mission frame");
        JsonObject message = MissionJson.parse(body);
        try { validate(message); } catch (IllegalArgumentException e) { throw new IOException("Invalid mission frame", e); }
        return message;
    }

    public static void write(OutputStream output, JsonObject message) throws IOException {
        validate(message);
        byte[] bytes = MissionJson.bytes(message);
        var data = new DataOutputStream(output);
        data.writeInt(bytes.length);
        data.write(bytes);
    }
}
