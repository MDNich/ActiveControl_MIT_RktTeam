package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.security.*;
import java.util.HexFormat;

/** Hash-pinned, offline GeographicLib EGM96 grid; bilinear interpolation, metres above WGS84. */
public final class MissionGeoid {
    private final byte[] pixels;
    private final int width, height;
    private final double offset, scale;
    private final JsonObject provenance;

    private static final class Holder { private static final MissionGeoid INSTANCE = new MissionGeoid(); }
    public static MissionGeoid bundled() { return Holder.INSTANCE; }
    public JsonObject provenance() { return provenance; }

    private MissionGeoid() {
        try (var meta = MissionGeoid.class.getResourceAsStream("/mission/geoid/provenance.json");
                var grid = MissionGeoid.class.getResourceAsStream("/mission/geoid/egm96-15.pgm")) {
            if (meta == null || grid == null) throw new IOException("Bundled EGM96 grid is missing");
            provenance = MissionJson.read(meta);
            byte[] data = grid.readNBytes(3 * 1024 * 1024);
            if (grid.read() != -1 || !HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(data))
                    .equals(provenance.getString("sha256"))) throw new IOException("Bundled EGM96 grid hash mismatch");
            var input = new ByteArrayInputStream(data);
            if (!line(input).equals("P5")) throw new IOException("Invalid PGM grid");
            double o = Double.NaN, s = Double.NaN;
            String header;
            while ((header = line(input)).startsWith("#")) {
                if (header.startsWith("# Offset ")) o = Double.parseDouble(header.substring(9));
                if (header.startsWith("# Scale ")) s = Double.parseDouble(header.substring(8));
            }
            String[] dimensions = header.split(" ");
            width = Integer.parseInt(dimensions[0]); height = Integer.parseInt(dimensions[1]);
            if (width != 1440 || height != 721 || !line(input).equals("65535") || !Double.isFinite(o) || !(s > 0))
                throw new IOException("Unsupported geoid dimensions/scale");
            offset = o; scale = s; pixels = input.readAllBytes();
            if (pixels.length != width * height * 2) throw new IOException("Truncated geoid");
        } catch (IOException | NoSuchAlgorithmException e) { throw new IllegalStateException("Cannot load mission height model", e); }
    }

    public double separation(double latitudeDeg, double longitudeDeg) {
        if (!Double.isFinite(latitudeDeg) || Math.abs(latitudeDeg) > 90 || !Double.isFinite(longitudeDeg)
                || Math.abs(longitudeDeg) > 180) throw new IllegalArgumentException("Invalid latitude/longitude");
        double x = ((longitudeDeg + 360) % 360) * width / 360;
        double y = (90 - latitudeDeg) * (height - 1) / 180;
        int ix = (int) Math.floor(x), iy = Math.min((int) Math.floor(y), height - 2);
        double dx = x - ix, dy = y - iy;
        return offset + scale * ((1 - dy) * ((1 - dx) * pixel(ix, iy) + dx * pixel((ix + 1) % width, iy))
                + dy * ((1 - dx) * pixel(ix, iy + 1) + dx * pixel((ix + 1) % width, iy + 1)));
    }

    private int pixel(int x, int y) {
        int index = 2 * (y * width + x);
        return ((pixels[index] & 255) << 8) | (pixels[index + 1] & 255);
    }

    private static String line(InputStream input) throws IOException {
        var bytes = new ByteArrayOutputStream(); int c;
        while ((c = input.read()) != '\n') {
            if (c < 0 || bytes.size() > 1024) throw new IOException("Malformed geoid header");
            bytes.write(c);
        }
        return bytes.toString(StandardCharsets.US_ASCII);
    }
}
