package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.util.*;

/** Ordered link pipeline. RF loss precedes corruption, delay and optional serial pacing. */
public final class MissionLink {
    public record Outcome(String outcome, long deliveryUs, byte[] payload, long serializationUs) { }
    private final JsonObject profile;
    private final String seed, receiver;
    private final long member;
    private long lastDeliveryUs = -1_000_000, serialFreeUs = -1_000_000;
    public MissionLink(JsonObject profile, String seed, long member) {
        validate(profile); this.profile = profile; this.seed = seed; this.member = member; receiver = profile.getString("receiverId");
    }
    public static void validate(JsonObject profile) {
        for (String key : List.of("packetLossFraction", "bitErrorProbability")) bounded(profile, key, 0, 1);
        for (String key : List.of("delayMs", "jitterMs")) bounded(profile, key, 0, 60_000);
        bounded(profile, "uartBaud", 0, 10_000_000); bounded(profile, "beamWidthDeg", .01, 360);
        if (profile.containsKey("uartBaud") && number(profile, "uartBaud", 0) != 0 && number(profile, "uartBaud", 0) < 50)
            throw new IllegalArgumentException("UART baud must be zero (software serial) or at least 50");
        for (var value : profile.getJsonArray("outages") == null ? JsonValue.EMPTY_JSON_ARRAY : profile.getJsonArray("outages")) {
            var interval = value.asJsonObject(); long start = Long.parseLong(interval.getString("startUs")), end = Long.parseLong(interval.getString("endUs"));
            if (start < -1_000_000 || end <= start || end > 86_400_000_000L) throw new IllegalArgumentException("Invalid outage interval");
        }
    }
    private static void bounded(JsonObject p, String key, double low, double high) {
        if (!p.containsKey(key)) return;
        double n = number(p, key, 0); if (!Double.isFinite(n) || n < low || n > high) throw new IllegalArgumentException("Invalid link " + key);
    }
    public Outcome transmit(long txUs, long packet, byte[] payload, int frameBytes, boolean tuned, boolean qualified) {
        if (!tuned) return new Outcome("routing-excluded", txUs, null, 0);
        for (var entry : profile.getJsonArray("outages") == null ? JsonValue.EMPTY_JSON_ARRAY : profile.getJsonArray("outages")) {
            var window = entry.asJsonObject(); if (txUs >= Long.parseLong(window.getString("startUs")) && txUs < Long.parseLong(window.getString("endUs")))
                return new Outcome("outage", txUs, null, 0);
        }
        if (profile.getBoolean("directional", false) && !qualified) return new Outcome("outside-beam", txUs, null, 0);
        if (uniform("packet-loss", packet, 0) < number(profile, "packetLossFraction", 0)) return new Outcome("packet-loss", txUs, null, 0);
        byte[] bytes = payload.clone(); double bitError = number(profile, "bitErrorProbability", 0); boolean corrupted = false;
        if (bitError > 0) for (int i = 0; i < bytes.length; i++) for (int bit = 0; bit < 8; bit++) {
            if (uniform("bit-error", packet, i * 8L + bit) < bitError) { bytes[i] ^= (byte) (1 << bit); corrupted = true; }
        }
        double jitter = number(profile, "jitterMs", 0) * MissionRandom.normal(seed, member, receiver, "delay", "link.v1", packet, 0);
        long delay = Math.round(Math.max(0, number(profile, "delayMs", 0) + jitter) * 1000);
        long arrival = Math.addExact(txUs, delay);
        if (!profile.getBoolean("allowReordering", false)) arrival = Math.max(arrival, lastDeliveryUs);
        long serialization = 0; double baud = number(profile, "uartBaud", 0);
        if (baud > 0) {
            serialization = (long) Math.ceil(frameBytes * 10_000_000.0 / baud);
            arrival = Math.addExact(Math.max(serialFreeUs, arrival), serialization); serialFreeUs = arrival;
        }
        lastDeliveryUs = arrival;
        return new Outcome(corrupted ? "corrupted" : "delivered", arrival, bytes, serialization);
    }
    private double uniform(String phenomenon, long event, long draw) { return MissionRandom.uniform(seed, member, receiver, phenomenon, "link.v1", event, draw); }
    private static double number(JsonObject object, String key, double fallback) { return MissionPointer.number(object, key, fallback); }
}
