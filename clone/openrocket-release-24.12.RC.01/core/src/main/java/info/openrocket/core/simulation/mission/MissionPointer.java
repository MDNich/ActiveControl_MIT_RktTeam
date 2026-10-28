package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.nio.*;

/** Illustrative rate-limited ground mount. Reference zero changes encoder offsets, never pose. */
public final class MissionPointer {
    private double azimuth, elevation, targetAzimuth, targetElevation, azimuthZero, elevationZero;
    private final double azimuthRate, elevationRate;
    private long timeUs = -1_000_000;
    private boolean saturated;
    public MissionPointer(JsonObject config) {
        azimuth = targetAzimuth = number(config, "initialAzimuthDeg", 0);
        elevation = targetElevation = number(config, "initialElevationDeg", 0);
        azimuthRate = number(config, "azimuthRateDegS", 90); elevationRate = number(config, "elevationRateDegS", 60);
    }
    public void advance(long time) {
        if (time < timeUs) throw new IllegalArgumentException("Pointer clock moved backwards");
        double dt = (time - timeUs) / 1e6;
        azimuth += clamp(targetAzimuth - azimuth, -azimuthRate * dt, azimuthRate * dt);
        elevation += clamp(targetElevation - elevation, -elevationRate * dt, elevationRate * dt);
        timeUs = time;
    }
    public String command(byte[] packet, long createdUs, long expiresUs, long now) {
        advance(now);
        if (now > expiresUs || now < createdUs) return "expired";
        if (packet.length != 11 || (packet[0] & 255) != 0xaa) return "rejected-frame";
        int sum = 0; for (int i = 1; i < 10; i++) sum += packet[i] & 255;
        if ((sum & 255) != (packet[10] & 255)) return "rejected-checksum";
        var data = ByteBuffer.wrap(packet).order(ByteOrder.LITTLE_ENDIAN);
        double az = data.getFloat(2), el = data.getFloat(6);
        int opcode = packet[1] & 255;
        if (opcode > 5 || !Double.isFinite(az) || !Double.isFinite(el)) return "rejected-value";
        if (opcode == 5) {
            // Firmware reference reset: keep physical motion/pose and move the encoder origin.
            azimuthZero = azimuth; elevationZero = elevation; return "applied-zero";
        }
        double requestedAz = switch (opcode) { case 3 -> azimuth - 5; case 4 -> azimuth + 5; case 0 -> az + azimuthZero; default -> targetAzimuth; };
        double requestedEl = switch (opcode) { case 1 -> elevation + 5; case 2 -> elevation - 5; case 0 -> el + elevationZero; default -> targetElevation; };
        targetAzimuth = azimuth + ((requestedAz - azimuth + 540) % 360 + 360) % 360 - 180;
        targetElevation = clamp(requestedEl, 0, 90); saturated = targetElevation != requestedEl;
        return saturated ? "applied-saturated" : "applied";
    }
    public JsonObject state() {
        return Json.createObjectBuilder().add("timeUs", Long.toString(timeUs)).add("azimuthDeg", azimuth)
                .add("elevationDeg", elevation).add("targetAzimuthDeg", targetAzimuth).add("targetElevationDeg", targetElevation)
                .add("encoderAzimuthDeg", azimuth - azimuthZero).add("encoderElevationDeg", elevation - elevationZero)
                .add("saturated", saturated).add("model", "illustrative-rate-limited.v1").build();
    }
    public double azimuth() { return azimuth; }
    public double elevation() { return elevation; }
    static double clamp(double value, double min, double max) { return Math.max(min, Math.min(max, value)); }
    static double number(JsonObject object, String key, double fallback) { return object.containsKey(key) ? object.getJsonNumber(key).doubleValue() : fallback; }
}
