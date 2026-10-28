package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.nio.*;
import java.util.*;

/** Declarative scenario actions and the actual 16-byte FC command envelope. */
public final class MissionActions {
    private MissionActions() { }
    public static final Set<String> KINDS = Set.of("connect", "disconnect", "start-tracking", "hold", "zero", "point", "uplink", "set-target");
    public static void validate(JsonObject action) {
        if (!KINDS.contains(action.getString("kind"))) throw new IllegalArgumentException("Unknown mission action");
        if (!Set.of("launch", "gs1", "gs2", "gs3", "gs4").contains(action.getString("siteId"))) throw new IllegalArgumentException("Unknown action station");
        if (action.containsKey("timeUs")) { long t = Long.parseLong(action.getString("timeUs")); if (t < -1_000_000 || t > 86_400_000_000L) throw new IllegalArgumentException("Action time out of bounds"); }
        if (action.getString("kind").equals("uplink")) {
            if (!action.getString("siteId").equals("launch")) throw new IllegalArgumentException("Only Base owns FC uplink");
            packet(action);
        }
        if (action.getString("kind").equals("point")) for (String key : List.of("azimuthDeg", "elevationDeg")) {
            double n = action.getJsonNumber(key).doubleValue(); if (!Double.isFinite(n) || Math.abs(n) > 360) throw new IllegalArgumentException("Invalid pointer angle");
        }
    }
    public static byte[] packet(JsonObject action) {
        // Explicit raw input supports malformed-packet experiments without inventing acknowledgments.
        if (action.containsKey("dataHex")) {
            String raw = action.getString("dataHex"); if (!raw.matches("(?:[0-9a-fA-F]{2}){1,256}")) throw new IllegalArgumentException("Raw command must be 1..256 bytes");
            return HexFormat.of().parseHex(raw);
        }
        int opcode = action.getInt("opcode"), value = action.getInt("value", 0);
        if (!Set.of(1,2,3,4,5,6,8,9,10,11,16,17,18,19,20,21,22).contains(opcode)) throw new IllegalArgumentException("Opcode is unavailable in this FC runtime (07 is not implemented)");
        if (value < 0 || value > 255) throw new IllegalArgumentException("Command argument must fit one byte");
        byte[] packet = new byte[16]; packet[0] = (byte) 0xaa; packet[12] = (byte) value; packet[13] = (byte) opcode;
        if (opcode == 3 || opcode == 5) {
            double angle = MissionPointer.number(action, "angleDeg", 0);
            if (!Double.isFinite(angle) || Math.abs(angle) > 180) throw new IllegalArgumentException("Manual angle must be -180..180 degrees");
            ByteBuffer.wrap(packet).order(ByteOrder.LITTLE_ENDIAN).putFloat(9, (float) angle);
        }
        int sum = 0; for (int i = 1; i < 14; i++) sum += packet[i] & 255;
        packet[14] = (byte) (sum >>> 8); packet[15] = (byte) sum; return packet;
    }
}
