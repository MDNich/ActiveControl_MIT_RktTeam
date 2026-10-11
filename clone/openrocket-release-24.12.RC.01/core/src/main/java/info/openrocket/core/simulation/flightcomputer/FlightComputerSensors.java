package info.openrocket.core.simulation.flightcomputer;

import edu.mit.rocket_team.FC.RTFC;
import edu.mit.rocket_team.util.data.*;
import jakarta.json.JsonObject;
import java.util.*;

/**
 * Per-flight sensor acquisition at accepted FC sample boundaries, with stable-ID random streams.
 */
public final class FlightComputerSensors {
    private final FlightComputerDesign design;
    private final Map<String, Random> random = new HashMap<>();
    private final Map<String, Long> last = new HashMap<>();
    private final Map<String, RTFC.Inputs> acquired = new LinkedHashMap<>();

    /**
     * Returns the most recent per-source acquired measurements.
     *
     * @return the most recent per-source acquired measurements
     */
    public Map<String, RTFC.Inputs> acquired() {
        return Map.copyOf(acquired);
    }

    /**
     * Creates a FlightComputerSensors with the supplied dependencies and per-instance state.
     *
     * @param d initial design
     */
    public FlightComputerSensors(FlightComputerDesign d) {
        design = d;
    }

    /**
     * Reads a numeric sensor property from the design with its model fallback.
     *
     * @param n n
     * @param key configuration or lookup key
     * @return computed numeric value
     */
    private double prop(JsonObject n, String key) {
        return n.getJsonObject("properties").containsKey(key)
                ? n.getJsonObject("properties").getJsonNumber(key).doubleValue()
                : 0;
    }

    /**
     * Tests whether a sensor acquisition is due at the supplied boot time.
     *
     * @param n id value
     * @param time time
     * @return true when the condition described above holds
     */
    private boolean due(JsonObject n, long time) {
        String id = n.getString("id");
        if (last.containsKey(id) && time - last.get(id) < prop(n, "periodUs")) return false;
        last.put(id, time);
        return true;
    }

    /**
     * Applies configured bias and Gaussian noise to one measured scalar.
     *
     * @param n n
     * @param x X component or horizontal coordinate
     * @return computed numeric value
     */
    private double value(JsonObject n, double x) {
        var r =
                random.computeIfAbsent(
                        n.getString("id"),
                        id ->
                                new Random(
                                        (long) design.parameter("sensorSeed")
                                                ^ ((long) id.hashCode() << 32)));
        double percent =
                Math.hypot(design.parameter("measurementNoisePercent"), prop(n, "noisePercent"))
                        / 100;
        return x
                + prop(n, "bias")
                + Math.hypot(prop(n, "noise"), Math.abs(x) * percent) * r.nextGaussian();
    }

    /**
     * Applies percentage Gaussian measurement noise using the configured scale.
     *
     * @param n n
     * @param x X component or horizontal coordinate
     * @return computed numeric value
     */
    private double percentValue(JsonObject n, double x) {
        double percent =
                Math.hypot(design.parameter("measurementNoisePercent"), prop(n, "noisePercent"))
                        / 100;
        return x
                + Math.abs(x)
                        * percent
                        * random.computeIfAbsent(
                                        n.getString("id"),
                                        id ->
                                                new Random(
                                                        (long) design.parameter("sensorSeed")
                                                                ^ ((long) id.hashCode() << 32)))
                                .nextGaussian();
    }

    /**
     * Applies the sensor mounting rotation to a three-axis sample.
     *
     * @param n n
     * @param x X component or horizontal coordinate
     * @param y Y component or vertical coordinate
     * @param z Z component
     * @return the resulting double[]
     */
    public static double[] rotate(JsonObject n, double x, double y, double z) {
        double[] v = {x, y, z};
        if (n == null) return v;
        for (int axis = 0; axis < 3; axis++) {
            double
                    a =
                            Math.toRadians(
                                    n.getJsonObject("properties")
                                            .getJsonNumber("rotation" + "XYZ".charAt(axis))
                                            .doubleValue()),
                    c = Math.cos(a),
                    s = Math.sin(a);
            int j = (axis + 1) % 3, k = (axis + 2) % 3;
            double b = v[j], d = v[k];
            v[j] = c * b - s * d;
            v[k] = s * b + c * d;
        }
        return v;
    }

    /**
     * Transforms and perturbs a vector measurement using the sensor model.
     *
     * @param n n
     * @param x X component or horizontal coordinate
     * @param y Y component or vertical coordinate
     * @param z Z component
     * @return the resulting double[]
     */
    private double[] vector(JsonObject n, double x, double y, double z) {
        var v = rotate(n, x, y, z);
        for (int i = 0; i < 3; i++) v[i] = value(n, v[i]);
        return v;
    }

    /**
     * Samples due sensors from accepted simulator inputs using deterministic per-source noise and
     * held values between acquisitions.
     *
     * @param now now
     * @param in b value
     * @return the resulting Inputs
     */
    public RTFC.Inputs sample(long now, RTFC.Inputs in) {
        acquired.clear();
        RTBaroData baro = null;
        RTAccelData accel = null;
        RTGyroData gyro = null;
        RTGPSData gps = null;
        var n = design.active("barometer");
        if (in.baro() != null && due(n, now)) {
            var b = in.baro();
            double pressure = value(n, b.getPressure());
            pressure = Math.max(.001, pressure);
            baro =
                    new RTBaroData(
                            Float.NaN,
                            Float.NaN,
                            (float)
                                    (Math.max(1, percentValue(n, b.getTemperature() + 273.15))
                                            - 273.15),
                            Float.NaN,
                            (float) pressure);
        }
        n = design.active("accelerometer");
        if (in.accel() != null && due(n, now)) {
            var a = in.accel();
            var v = vector(n, a.getAccelX(), a.getAccelY(), a.getAccelZ());
            accel = new RTAccelData((float) v[0], (float) v[1], (float) v[2]);
        }
        n = design.active("gyroscope");
        if (in.gyro() != null && due(n, now)) {
            var g = in.gyro();
            var v = vector(n, g.getGyroX(), g.getGyroY(), g.getGyroZ());
            gyro = new RTGyroData((float) v[0], (float) v[1], (float) v[2]);
        }
        n = design.active("gps");
        if (in.gps() != null && due(n, now)) {
            var g = in.gps();
            gps =
                    new RTGPSData(
                            Math.max(-90, Math.min(90, percentValue(n, g.getLatitude()))),
                            Math.IEEEremainder(percentValue(n, g.getLongitude()), 360),
                            value(n, g.getAltitude()),
                            (double) g.getPDOP(),
                            (double) g.getVDOP(),
                            (double) g.getHDOP(),
                            g.getHasFix());
            gps.setFixType(g.getFixType());
        }
        var result =
                new RTFC.Inputs(in.acquisitionUs(), accel, baro, gps, gyro, in.gpsAcquisitionUs());
        remember(design.active("barometer"), in.acquisitionUs(), null, baro, null, null);
        remember(design.active("accelerometer"), in.acquisitionUs(), accel, null, null, null);
        remember(design.active("gyroscope"), in.acquisitionUs(), null, null, null, gyro);
        remember(design.active("gps"), in.gpsAcquisitionUs(), null, null, gps, null);
        for (var extra : FlightComputerSignals.sources(design)) {
            String type = extra.getString("type");
            var active = design.active(type);
            if (active != null && active.getString("id").equals(extra.getString("id"))) continue;
            if ((type.equals("barometer") && in.baro() == null)
                    || (type.equals("accelerometer") && in.accel() == null)
                    || (type.equals("gyroscope") && in.gyro() == null)
                    || (type.equals("gps") && in.gps() == null)
                    || !due(extra, now)) continue;
            switch (type) {
                case "barometer" -> {
                    var b = in.baro();
                    remember(
                            extra,
                            in.acquisitionUs(),
                            null,
                            new RTBaroData(
                                    Float.NaN,
                                    Float.NaN,
                                    (float)
                                            (Math.max(
                                                            1,
                                                            percentValue(
                                                                    extra,
                                                                    b.getTemperature() + 273.15))
                                                    - 273.15),
                                    Float.NaN,
                                    (float) Math.max(.001, value(extra, b.getPressure()))),
                            null,
                            null);
                }
                case "accelerometer" -> {
                    var a = in.accel();
                    var v = vector(extra, a.getAccelX(), a.getAccelY(), a.getAccelZ());
                    remember(
                            extra,
                            in.acquisitionUs(),
                            new RTAccelData(v[0], v[1], v[2]),
                            null,
                            null,
                            null);
                }
                case "gyroscope" -> {
                    var g = in.gyro();
                    var v = vector(extra, g.getGyroX(), g.getGyroY(), g.getGyroZ());
                    remember(
                            extra,
                            in.acquisitionUs(),
                            null,
                            null,
                            null,
                            new RTGyroData(v[0], v[1], v[2]));
                }
                case "gps" -> {
                    var g = in.gps();
                    var data =
                            new RTGPSData(
                                    Math.max(
                                            -90,
                                            Math.min(90, percentValue(extra, g.getLatitude()))),
                                    Math.IEEEremainder(percentValue(extra, g.getLongitude()), 360),
                                    value(extra, g.getAltitude()),
                                    (double) g.getPDOP(),
                                    (double) g.getVDOP(),
                                    (double) g.getHDOP(),
                                    g.getHasFix());
                    data.setFixType(g.getFixType());
                    remember(extra, in.gpsAcquisitionUs(), null, null, data, null);
                }
            }
        }
        return result;
    }

    /**
     * Retains a source measurement and its actual acquisition timestamp for controller lookup.
     *
     * @param node node
     * @param time time
     * @param a a
     * @param b b
     * @param g graphics context
     * @param r r
     */
    private void remember(
            JsonObject node, long time, RTAccelData a, RTBaroData b, RTGPSData g, RTGyroData r) {
        if (node != null && (a != null || b != null || g != null || r != null))
            acquired.put(node.getString("id"), new RTFC.Inputs(time, a, b, g, r));
    }
}
