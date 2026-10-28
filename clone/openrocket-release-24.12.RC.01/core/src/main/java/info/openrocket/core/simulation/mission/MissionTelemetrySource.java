package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.nio.*;

/** Held 10 Hz GPS / 20 Hz barometer observations and exact standard Zephyrus payloads. */
public final class MissionTelemetrySource {
    public record Packet(long timeUs, long ordinal, byte[] payload, JsonObject metadata) { }
    private final JsonObject config, binding, launch;
    private final String seed, id;
    private final long member;
    private final double gpsRate, baroRate, txRate;
    private long gpsOrdinal, baroOrdinal, txOrdinal, gpsTime = -1_000_000, baroTime = -1_000_000;
    private double latitude, longitude, gpsHeight, baroHeight, velocity;
    private boolean hasGps, hasBaro;
    public MissionTelemetrySource(JsonObject config, JsonObject binding, JsonObject launch, String seed, long member) {
        this.config = config; this.binding = binding; this.launch = launch; this.seed = seed; this.member = member; id = binding.getString("id");
        gpsRate = number(config, "gpsRateHz", 10); baroRate = number(config, "barometerRateHz", 20); txRate = number(config, "rateHz", 20);
        for (double rate : new double[] {gpsRate, baroRate, txRate}) if (!Double.isFinite(rate) || rate < .1 || rate > 1000) throw new IllegalArgumentException("Invalid telemetry acquisition/transmit rate");
    }
    private static long deadline(long ordinal, double hz) { return -1_000_000 + Math.round(ordinal * 1e6 / hz); }
    public long next() { return Math.min(deadline(txOrdinal, txRate), Math.min(deadline(gpsOrdinal, gpsRate), deadline(baroOrdinal, baroRate))); }
    public Packet advance(long time, MissionTruth truth) {
        MissionTruth.Sample state = truth.at(binding, time);
        var noise = config.getJsonObject("noise"); if (noise == null) noise = JsonValue.EMPTY_JSON_OBJECT;
        if (deadline(gpsOrdinal, gpsRate) == time) {
            if (state != null) {
                var loc = MissionGeodesy.geodetic(state.ecef());
                double horizontal = number(noise, "gpsHorizontalSigmaM", 0), vertical = number(noise, "gpsVerticalSigmaM", 0);
                latitude = MissionPointer.clamp(loc[0] + horizontal * normal("gps-north", gpsOrdinal) / 111_320, -90, 90);
                longitude = loc[1] + horizontal * normal("gps-east", gpsOrdinal) / (111_320 * Math.max(.001, Math.cos(Math.toRadians(loc[0]))));
                longitude = ((longitude + 540) % 360) - 180;
                gpsHeight = loc[2] - MissionGeoid.bundled().separation(loc[0], loc[1]) - number(launch, "mslHeightM", 0)
                        + vertical * normal("gps-height", gpsOrdinal);
                gpsTime = time; hasGps = true;
            } else hasGps = false;
            gpsOrdinal++;
        }
        if (deadline(baroOrdinal, baroRate) == time) {
            if (state != null) { baroHeight = state.agl() + number(noise, "barometerSigmaM", 0) * normal("barometer", baroOrdinal); velocity = state.velocityUp(); baroTime = time; hasBaro = true; }
            else hasBaro = false;
            baroOrdinal++;
        }
        if (deadline(txOrdinal, txRate) != time) return null;
        long packetOrdinal = txOrdinal++;
        if (state == null) return null;
        byte[] bytes = new byte[128]; var payload = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN);
        bytes[31] = (byte) (hasGps ? 3 : 0); payload.putInt(32, (int) Math.rint(latitude * 1e7)).putInt(36, (int) Math.rint(longitude * 1e7));
        payload.putFloat(40, hasGps ? (float) gpsHeight : Float.NaN).putFloat(59, hasBaro ? (float) baroHeight : Float.NaN);
        bytes[63] = (byte) 255; payload.putInt(80, (int) ((time + 1_000_000) / 1000)).putShort(84, (short) packetOrdinal);
        payload.putFloat(122, hasBaro ? (float) velocity : Float.NaN);
        int sum = 0; for (int i = 0; i < 127; i++) sum += bytes[i] & 255; bytes[127] = (byte) sum;
        var reference = Json.createObjectBuilder().add("convention", "relative-msl").add("baselineMslM", number(launch, "mslHeightM", 0))
                .add("geoidSeparationM", MissionGeoid.bundled().separation(latitude, longitude)).add("epoch", 0);
        var metadata = Json.createObjectBuilder().add("packetId", id + "-" + packetOrdinal).add("transmitterId", id)
                .add("txTimeUs", Long.toString(time)).add("bootTimeUs", Long.toString(time + 1_000_000))
                .add("acquisitionTimeUs", Long.toString(gpsTime)).add("barometerTimeUs", Long.toString(baroTime)).add("heightReference", reference)
                .add("unavailable", Json.createArrayBuilder().add("hardware").add("attitude").add("rates").add("acceleration").add("state")).build();
        return new Packet(time, packetOrdinal, bytes, metadata);
    }
    public static byte[] trailer(JsonObject receiver) {
        var data = ByteBuffer.allocate(14).order(ByteOrder.LITTLE_ENDIAN); data.put((byte) -1).put((byte) 3);
        data.putInt((int) Math.rint(number(receiver, "latitudeDeg", 0) * 1e7)); data.putInt((int) Math.rint(number(receiver, "longitudeDeg", 0) * 1e7));
        data.putInt((int) Math.rint(number(receiver, "mslHeightM", 0))); return data.array();
    }
    private double normal(String phenomenon, long ordinal) { return MissionRandom.normal(seed, member, id, phenomenon, "standard-observation.v1", ordinal, 0); }
    private static double number(JsonObject value, String key, double fallback) { return MissionPointer.number(value, key, fallback); }
}
