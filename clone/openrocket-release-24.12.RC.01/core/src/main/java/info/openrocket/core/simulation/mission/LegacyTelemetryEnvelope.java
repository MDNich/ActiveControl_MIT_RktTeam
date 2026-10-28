package info.openrocket.core.simulation.mission;

import java.util.Arrays;

/** Exact legacy receiver wrapping. Corrupted payloads can be wrapped to test decoder rejection. */
public final class LegacyTelemetryEnvelope {
    public static final int PAYLOAD_BYTES = 128;
    public static final int TRAILER_BYTES = 14;
    public static final int FRAME_BYTES = 144;
    private LegacyTelemetryEnvelope() { }

    public static byte[] wrap(byte[] payload, byte[] trailer) {
        if (payload.length != PAYLOAD_BYTES || trailer.length != TRAILER_BYTES)
            throw new IllegalArgumentException("Legacy payload/trailer must be 128/14 bytes");
        byte[] frame = new byte[FRAME_BYTES];
        frame[0] = frame[1] = (byte) 0xab;
        System.arraycopy(payload, 0, frame, 2, PAYLOAD_BYTES);
        System.arraycopy(trailer, 0, frame, 2 + PAYLOAD_BYTES, TRAILER_BYTES);
        return frame;
    }

    public static boolean validPayload(byte[] payload) {
        if (payload.length != PAYLOAD_BYTES) return false;
        int sum = 0;
        for (int i = 0; i < 127; i++) sum += payload[i] & 255;
        return (sum & 255) == (payload[127] & 255);
    }

    public static byte[] payload(byte[] frame) {
        if (frame.length != FRAME_BYTES || (frame[0] & 255) != 0xab || (frame[1] & 255) != 0xab)
            throw new IllegalArgumentException("Invalid legacy receiver frame");
        return Arrays.copyOfRange(frame, 2, 130);
    }
}
