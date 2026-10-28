package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.nio.*;
import java.util.*;

/** Bounded declarative fixed-frame telemetry. Physical = raw * scale + offset; no executable expressions. */
public final class TelemetryCodec {
    public static final Set<String> SOURCES = Set.of("uptime_ms", "packet_sequence", "gps.fix", "gps.latitude_deg",
            "gps.longitude_deg", "gps.height_m", "barometer.height_m", "velocity.up_ms", "state.code",
            "gyro.roll_rate_deg_s", "gyro.pitch_rate_deg_s", "gyro.yaw_rate_deg_s",
            "acceleration.x_ms2", "acceleration.y_ms2", "acceleration.z_ms2", "battery.voltage_v");
    private static final Map<String, Integer> WIDTHS = Map.of("i8", 1, "u8", 1, "i16", 2, "u16", 2,
            "i32", 4, "u32", 4, "f32", 4, "f64", 8);
    private record Field(String name, String source, String type, int offset, double scale, double add, boolean optional) { }
    public record Encoded(byte[] bytes, Set<String> unavailable) {
        public Encoded { bytes = bytes.clone(); unavailable = Set.copyOf(unavailable); }
        @Override public byte[] bytes() { return bytes.clone(); }
    }
    private final JsonObject definition;
    private final int length;
    private final ByteOrder order;
    private final List<Field> fields;
    private final String checksum, trailer;
    private final int checksumOffset, start, end, polynomial, initial, xorOut;
    private final boolean reflectIn, reflectOut;
    private final byte[] sync;

    public TelemetryCodec(JsonObject definition) {
        this.definition = Objects.requireNonNull(definition);
        var issues = MissionSchema.validate(MissionJson.resource("codec.schema.json"), definition);
        if (!issues.isEmpty()) throw new IllegalArgumentException("Invalid telemetry codec: " + issues);
        var check = definition.getJsonObject("checksum");
        if (!check.getString("algorithm").equals("none"))
            for (String key : List.of("byteOffset", "start", "endExclusive"))
                if (!check.containsKey(key)) throw new IllegalArgumentException("Missing checksum " + key);
        if (check.getString("algorithm").equals("crc16"))
            for (String key : List.of("polynomial", "initial", "xorOut", "reflectInput", "reflectOutput"))
                if (!check.containsKey(key)) throw new IllegalArgumentException("Missing CRC16 " + key);
        if (!"openrocket.telemetry-codec".equals(definition.getString("format", "")) || integer(definition, "version") != 1)
            throw new IllegalArgumentException("Unsupported codec format/version");
        if (!definition.getString("id").matches("[a-zA-Z0-9][a-zA-Z0-9._-]{0,127}")) throw new IllegalArgumentException("Invalid codec ID");
        length = integer(definition, "payloadBytes");
        if (length < 1 || length > 4096) throw new IllegalArgumentException("Codec payload must be 1..4096 bytes");
        order = switch (definition.getString("byteOrder")) {
            case "little" -> ByteOrder.LITTLE_ENDIAN; case "big" -> ByteOrder.BIG_ENDIAN;
            default -> throw new IllegalArgumentException("Unknown codec byte order");
        };
        String prefix = definition.getString("syncHex");
        if (!prefix.matches("(?:[0-9a-fA-F]{2}){0,16}")) throw new IllegalArgumentException("Sync must be at most 16 hex bytes");
        sync = HexFormat.of().parseHex(prefix);
        trailer = definition.getString("receiverTrailer");
        if (!Set.of("none", "legacy-v1").contains(trailer)) throw new IllegalArgumentException("Unsupported receiver trailer");
        boolean[] occupied = new boolean[length]; var names = new HashSet<String>(); var parsed = new ArrayList<Field>();
        if (definition.getJsonArray("fields").size() > 256) throw new IllegalArgumentException("At most 256 codec fields");
        for (JsonValue value : definition.getJsonArray("fields")) {
            JsonObject f = value.asJsonObject(); String name = f.getString("name"), source = f.getString("source"), type = f.getString("type");
            if (!name.matches("[a-zA-Z0-9][a-zA-Z0-9._-]{0,127}") || !names.add(name)) throw new IllegalArgumentException("Invalid/duplicate codec field");
            if (!SOURCES.contains(source)) throw new IllegalArgumentException("Unknown FC snapshot binding: " + source);
            if (!WIDTHS.containsKey(type)) throw new IllegalArgumentException("Unsupported field type");
            int offset = integer(f, "byteOffset"); reserve(occupied, offset, WIDTHS.get(type));
            double scale = number(f, "scale"), add = number(f, "offset");
            if (scale == 0 || f.getString("unit").length() > 64) throw new IllegalArgumentException("Invalid field scale/unit");
            parsed.add(new Field(name, source, type, offset, scale, add, f.getBoolean("optional", false)));
        }
        fields = List.copyOf(parsed);
        var c = definition.getJsonObject("checksum"); checksum = c.getString("algorithm");
        if (!Set.of("none", "sum8", "crc16").contains(checksum)) throw new IllegalArgumentException("Unsupported checksum");
        if (checksum.equals("none")) {
            checksumOffset = start = end = polynomial = initial = xorOut = 0; reflectIn = reflectOut = false;
        } else {
            checksumOffset = integer(c, "byteOffset"); start = integer(c, "start"); end = integer(c, "endExclusive");
            int width = checksum.equals("sum8") ? 1 : 2;
            reserve(occupied, checksumOffset, width);
            if (start < 0 || end <= start || end > length || (start < checksumOffset + width && end > checksumOffset))
                throw new IllegalArgumentException("Checksum range is invalid or includes its output");
            if (checksum.equals("crc16")) {
                polynomial = integer(c, "polynomial"); initial = integer(c, "initial"); xorOut = integer(c, "xorOut");
                reflectIn = c.getBoolean("reflectInput"); reflectOut = c.getBoolean("reflectOutput");
                if (polynomial < 1 || polynomial > 65535 || initial < 0 || initial > 65535 || xorOut < 0 || xorOut > 65535)
                    throw new IllegalArgumentException("Invalid CRC16 parameters");
            } else { polynomial = initial = xorOut = 0; reflectIn = reflectOut = false; }
        }
    }

    public JsonObject definition() { return definition; }
    public String id() { return definition.getString("id"); }
    public int payloadBytes() { return length; }
    public int frameBytes() { return sync.length + length + (trailer.equals("none") ? 0 : 14); }

    private static int integer(JsonObject d, String key) { return d.getJsonNumber(key).intValueExact(); }
    private static double number(JsonObject d, String key) {
        double n = d.getJsonNumber(key).doubleValue();
        if (!Double.isFinite(n)) throw new IllegalArgumentException("Non-finite codec number");
        return n;
    }
    private static void reserve(boolean[] occupied, int offset, int size) {
        if (offset < 0 || offset > occupied.length - size) throw new IllegalArgumentException("Field outside payload");
        for (int i = offset; i < offset + size; i++) {
            if (occupied[i]) throw new IllegalArgumentException("Overlapping codec fields/checksum"); occupied[i] = true;
        }
    }

    public Encoded encode(Map<String, Double> values) {
        byte[] payload = new byte[length]; var b = ByteBuffer.wrap(payload).order(order); var missing = new HashSet<String>();
        for (Field f : fields) {
            Double value = values.get(f.source());
            if (value == null || !Double.isFinite(value)) {
                if (!f.optional()) throw new IllegalArgumentException("Missing required observation " + f.source());
                missing.add(f.name()); continue;
            }
            double raw = (value - f.add()) / f.scale();
            if (!Double.isFinite(raw)) throw new IllegalArgumentException("Non-finite encoded field " + f.name());
            if (f.type().startsWith("f")) {
                if (f.type().equals("f32")) {
                    if (!Float.isFinite((float) raw)) throw new IllegalArgumentException("Float overflow " + f.name());
                    b.putFloat(f.offset(), (float) raw);
                } else b.putDouble(f.offset(), raw);
            } else {
                int bits = WIDTHS.get(f.type()) * 8; boolean unsigned = f.type().startsWith("u");
                double rounded = Math.rint(raw), min = unsigned ? 0 : -Math.scalb(1, bits - 1), max = Math.scalb(1, unsigned ? bits : bits - 1) - 1;
                if (rounded < min || rounded > max) throw new IllegalArgumentException("Integer overflow " + f.name());
                long n = (long) rounded;
                switch (bits) { case 8 -> b.put(f.offset(), (byte) n); case 16 -> b.putShort(f.offset(), (short) n); case 32 -> b.putInt(f.offset(), (int) n); default -> throw new AssertionError(); }
            }
        }
        if (!checksum.equals("none")) {
            int value = checksum(payload);
            if (checksum.equals("sum8")) b.put(checksumOffset, (byte) value); else b.putShort(checksumOffset, (short) value);
        }
        return new Encoded(payload, missing);
    }

    public Map<String, Double> decode(byte[] payload) {
        if (payload.length != length) throw new IllegalArgumentException("Wrong codec payload size");
        var b = ByteBuffer.wrap(payload).order(order);
        if (!checksum.equals("none")) {
            int actual = checksum.equals("sum8") ? b.get(checksumOffset) & 255 : b.getShort(checksumOffset) & 65535;
            if (checksum(payload) != actual) throw new IllegalArgumentException("Invalid codec checksum");
        }
        var values = new LinkedHashMap<String, Double>();
        for (Field f : fields) {
            double raw = switch (f.type()) {
                case "i8" -> b.get(f.offset()); case "u8" -> b.get(f.offset()) & 255;
                case "i16" -> b.getShort(f.offset()); case "u16" -> b.getShort(f.offset()) & 65535;
                case "i32" -> b.getInt(f.offset()); case "u32" -> Integer.toUnsignedLong(b.getInt(f.offset()));
                case "f32" -> b.getFloat(f.offset()); case "f64" -> b.getDouble(f.offset()); default -> throw new AssertionError();
            };
            double value = raw * f.scale() + f.add(); values.put(f.name(), Double.isFinite(value) ? value : null);
        }
        return Collections.unmodifiableMap(values);
    }

    public byte[] frame(byte[] payload, byte[] receiverTrailer) {
        if (payload.length != length || receiverTrailer.length != (trailer.equals("none") ? 0 : 14))
            throw new IllegalArgumentException("Invalid codec envelope lengths");
        var out = ByteBuffer.allocate(frameBytes()); out.put(sync).put(payload).put(receiverTrailer); return out.array();
    }

    private int checksum(byte[] bytes) {
        if (checksum.equals("sum8")) { int n = 0; for (int i = start; i < end; i++) n += bytes[i] & 255; return n & 255; }
        int crc = initial, poly = reflectIn ? Integer.reverse(polynomial) >>> 16 : polynomial;
        for (int i = start; i < end; i++) {
            if (reflectIn) {
                crc ^= bytes[i] & 255;
                for (int bit = 0; bit < 8; bit++) crc = (crc >>> 1) ^ ((crc & 1) == 0 ? 0 : poly);
            } else {
                crc ^= (bytes[i] & 255) << 8;
                for (int bit = 0; bit < 8; bit++) crc = ((crc << 1) ^ ((crc & 0x8000) == 0 ? 0 : poly)) & 65535;
            }
        }
        if (reflectOut != reflectIn) crc = Integer.reverse(crc) >>> 16;
        return (crc ^ xorOut) & 65535;
    }
}
