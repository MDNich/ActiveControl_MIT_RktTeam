package info.openrocket.core.simulation.flightcomputer;

import edu.mit.rocket_team.FC.RTFC;
import edu.mit.rocket_team.instrument.*;
import edu.mit.rocket_team.util.RTUtilLibrary.Trace;
import edu.mit.rocket_team.util.data.*;
import jakarta.json.JsonObject;
import java.util.*;

/** Measured signal catalogue and per-source held state; reading never samples or draws noise. */
public final class FlightComputerSignals {
    public record Channel(String id, String label, String unit, String derivativeUnit) {
        /**
         * Returns the display representation used by this value in logs or UI controls.
         *
         * @return the display representation used by this value in logs or UI controls
         */
        public String toString() {
            return label + " (" + unit + ")";
        }
    }

    public record Descriptor(String source, String sourceName, Channel quantity, String frame) {
        /**
         * Returns the engineering unit associated with this measured quantity.
         *
         * @return the engineering unit associated with this measured quantity
         */
        public String unit() {
            return quantity.unit();
        }

        /**
         * Returns the engineering unit for the time derivative of this measured quantity.
         *
         * @return the engineering unit for the time derivative of this measured quantity
         */
        public String derivativeUnit() {
            return quantity.derivativeUnit();
        }
    }

    public record Sample(double value, long acquisitionUs, long availableUs, boolean valid) {
        /**
         * Checks validity and acquisition age against the controller's allowed measurement age.
         *
         * @param now now
         * @param staleUs stale in microseconds
         * @return true when the condition described above holds
         */
        public boolean fresh(long now, long staleUs) {
            return valid
                    && Double.isFinite(value)
                    && availableUs <= now
                    && acquisitionUs <= now
                    && now - acquisitionUs <= staleUs;
        }

        /**
         * Creates an explicitly invalid measurement instead of substituting simulator truth.
         *
         * @return the resulting Sample
         */
        public static Sample missing() {
            return new Sample(Double.NaN, -1, -1, false);
        }
    }

    /**
     * Returns the measured quantities exposed by a sensor model.
     *
     * @param type type
     * @return the measured quantities exposed by a sensor model
     */
    public static List<Channel> channels(String type) {
        return switch (type) {
            case "gyroscope" ->
                    List.of(
                            new Channel("roll_rate", "Roll rate (−sensor X)", "deg/s", "deg/s²"),
                            new Channel("pitch_rate", "Pitch rate (sensor Y)", "deg/s", "deg/s²"),
                            new Channel("yaw_rate", "Yaw rate (sensor Z)", "deg/s", "deg/s²"),
                            new Channel("roll", "Integrated roll angle", "deg", "deg/s"),
                            new Channel("pitch", "Integrated pitch angle", "deg", "deg/s"),
                            new Channel("yaw", "Integrated yaw angle", "deg", "deg/s"),
                            new Channel("x", "Raw X angular rate", "deg/s", "deg/s²"),
                            new Channel("y", "Raw Y angular rate", "deg/s", "deg/s²"),
                            new Channel("z", "Raw Z angular rate", "deg/s", "deg/s²"));
            case "accelerometer" ->
                    List.of(
                            new Channel("x", "Specific acceleration X", "m/s²", "m/s³"),
                            new Channel("y", "Specific acceleration Y", "m/s²", "m/s³"),
                            new Channel("z", "Specific acceleration Z", "m/s²", "m/s³"),
                            new Channel("vertical", "X acceleration minus gravity", "m/s²", "m/s³"),
                            new Channel(
                                    "velocity",
                                    "Integrated X velocity (not airspeed)",
                                    "m/s",
                                    "m/s²"));
            case "barometer" ->
                    List.of(
                            new Channel(
                                    "altitude", "Filtered height above calibration", "m", "m/s"),
                            new Channel("pressure", "Pressure", "hPa", "hPa/s"),
                            new Channel("temperature", "Temperature", "°C", "°C/s"));
            case "gps" ->
                    List.of(
                            new Channel("altitude", "Height above calibration", "m", "m/s"),
                            new Channel("msl", "Altitude MSL", "m", "m/s"),
                            new Channel("latitude", "Latitude", "deg", "deg/s"),
                            new Channel("longitude", "Longitude", "deg", "deg/s"));
            default -> List.of();
        };
    }

    /**
     * Lists eligible sensor sources using stable design node IDs.
     *
     * @param d d
     * @return the resulting List&lt;JsonObject&gt;
     */
    public static List<JsonObject> sources(FlightComputerDesign d) {
        return d.nodes().getValuesAs(JsonObject.class).stream()
                .filter(n -> !channels(n.getString("type")).isEmpty())
                .toList();
    }

    /**
     * Resolves a source/channel selection to its units and frame description.
     *
     * @param d node value
     * @param source source text, model or measured-channel selection
     * @param channel logical recovery channel, numbered 0 through 5
     * @return the resulting Descriptor
     * @throws IllegalArgumentException when rejected: Missing controller sensor:
     */
    public static Descriptor descriptor(FlightComputerDesign d, String source, String channel) {
        var node = d.node(source);
        if (node == null)
            throw new IllegalArgumentException("Missing controller sensor: " + source);
        var c =
                channels(node.getString("type")).stream()
                        .filter(x -> x.id().equals(channel))
                        .findFirst()
                        .orElseThrow(
                                () ->
                                        new IllegalArgumentException(
                                                "Unsupported sensor quantity: " + channel));
        return new Descriptor(
                source,
                node.getString("name"),
                c,
                "Configured sensor mounting; FC X longitudinal, Y body X, Z body Y");
    }

    private static final class Instruments {
        final Trace clock = new Trace(s -> {});
        final RTBaro baro;
        final RTAccel accel;
        final RTGyro gyro;
        final RTGPS gps;
        long acquisition = -1;
        boolean valid;
        double msl;

        /**
         * Creates a Instruments value from the supplied node, fc, legacy.
         *
         * @param node node
         * @param fc fc
         * @param legacy legacy
         */
        Instruments(JsonObject node, RTFC fc, boolean legacy) {
            var quiet = clock;
            baro = legacy ? fc.baro : new RTBaro(quiet);
            accel = legacy ? fc.accel : new RTAccel(quiet);
            gyro = legacy ? fc.gyro : new RTGyro(quiet);
            gps = legacy ? fc.gps : new RTGPS(quiet);
            if (!legacy && node.getString("type").equals("barometer"))
                baro.setFilterSamples(node.getJsonObject("properties").getInt("filterSamples"));
        }
    }

    private final FlightComputerDesign design;
    private final Map<String, Instruments> instruments = new LinkedHashMap<>();
    private final Map<String, Sample> samples = new HashMap<>();
    private long calibratedUs = -1;

    /**
     * Creates a FlightComputerSignals with the supplied dependencies and per-instance state.
     *
     * @param d initial design
     */
    public FlightComputerSignals(FlightComputerDesign d) {
        design = d;
    }

    /**
     * Returns the boot-microsecond timestamp of the most recent sensor-reference calibration.
     *
     * @return the boot-microsecond timestamp of the most recent sensor-reference calibration
     */
    public long calibratedUs() {
        return calibratedUs;
    }

    /**
     * Resolves a measured signal by source ID and channel, retaining its acquisition time and
     * validity.
     *
     * @param source source text, model or measured-channel selection
     * @param channel logical recovery channel, numbered 0 through 5
     * @return the resulting Sample
     */
    public Sample read(String source, String channel) {
        return samples.getOrDefault(source + "/" + channel, Sample.missing());
    }

    /**
     * Reads a primary FC instrument through the legacy signal convention.
     *
     * @param node node
     * @return true when the condition described above holds
     */
    private boolean legacy(JsonObject node) {
        var active = design.active(node.getString("type"));
        return active != null && active.getString("id").equals(node.getString("id"));
    }

    /**
     * Publishes held primary and additional sensor readings after FC acquisition processing.
     *
     * @param fc fc
     * @param acquired in value
     * @param now now
     */
    public void publish(RTFC fc, Map<String, RTFC.Inputs> acquired, long now) {
        for (var node : sources(design)) {
            String id = node.getString("id"), type = node.getString("type");
            boolean legacy = legacy(node);
            var state = instruments.computeIfAbsent(id, k -> new Instruments(node, fc, legacy));
            var in = acquired.get(id);
            if (in != null) {
                state.acquisition = in.acquisitionUs();
                state.valid = true;
                if (in.gps() != null) {
                    state.valid = in.gps().getHasFix();
                    state.msl = in.gps().getAltitude();
                }
                if (!legacy) {
                    if (in.baro() != null) state.baro.backdoorFudge(in.baro());
                    if (in.accel() != null) state.accel.backdoorFudge(in.accel());
                    if (in.gyro() != null) state.gyro.backdoorFudge(in.gyro());
                    if (in.gps() != null) state.gps.backdoorFudge(in.gps());
                }
            }
            if (!legacy && state.acquisition >= 0) {
                // Same accepted FC read cadence as legacy estimators; no extra noise reads.
                state.clock.time(now);
                switch (type) {
                    case "barometer" -> state.baro.updateAll();
                    case "accelerometer" -> state.accel.update(fc.getState());
                    case "gyroscope" -> state.gyro.update();
                    case "gps" -> state.gps.updateAndParse();
                }
            }
            snapshot(node, state, now);
        }
    }

    /**
     * Stores a measured value with its acquisition, availability and validity information.
     *
     * @param id stable identifier
     * @param channel logical recovery channel, numbered 0 through 5
     * @param value replacement or input value
     * @param state current flight phase or state
     * @param now now
     */
    private void put(String id, String channel, double value, Instruments state, long now) {
        samples.put(
                id + "/" + channel,
                new Sample(
                        value,
                        state.acquisition,
                        now,
                        state.valid && state.acquisition >= 0 && Double.isFinite(value)));
    }

    /**
     * Returns a frozen view of the measured channels for one controller task start.
     *
     * @param node id value
     * @param state current flight phase or state
     * @param now now
     */
    private void snapshot(JsonObject node, Instruments state, long now) {
        String id = node.getString("id");
        switch (node.getString("type")) {
            case "barometer" -> {
                put(id, "altitude", state.baro.getFilteredAltitude(), state, now);
                put(id, "pressure", state.baro.getPressure(), state, now);
                put(id, "temperature", state.baro.getTemperature(), state, now);
            }
            case "accelerometer" -> {
                put(id, "x", state.accel.getAccelX(), state, now);
                put(id, "y", state.accel.getAccelY(), state, now);
                put(id, "z", state.accel.getAccelZ(), state, now);
                put(id, "vertical", state.accel.getVerticalAccelMinusGravity(), state, now);
                put(id, "velocity", state.accel.getIntegratedVelo(), state, now);
            }
            case "gyroscope" -> {
                put(id, "x", state.gyro.getGyroX(), state, now);
                put(id, "y", state.gyro.getGyroY(), state, now);
                put(id, "z", state.gyro.getGyroZ(), state, now);
                put(id, "roll_rate", state.gyro.getRollRate(), state, now);
                put(id, "pitch_rate", state.gyro.getGyroY(), state, now);
                put(id, "yaw_rate", state.gyro.getGyroZ(), state, now);
                put(id, "roll", state.gyro.getRoll(), state, now);
                put(id, "pitch", state.gyro.getPitch(), state, now);
                put(id, "yaw", state.gyro.getYaw(), state, now);
            }
            case "gps" -> {
                put(id, "altitude", state.gps.getHeight(), state, now);
                put(id, "msl", state.msl, state, now);
                put(id, "latitude", state.gps.getLatitude(), state, now);
                put(id, "longitude", state.gps.getLongitude(), state, now);
            }
        }
    }

    /**
     * Zeros sensor references for preflight or launch and records when calibration became valid.
     *
     * @param fc fc
     * @param now calibrated us value
     * @param launch launch
     */
    public void calibrate(RTFC fc, long now, boolean launch) {
        if (!launch) calibratedUs = now;
        for (var node : sources(design)) {
            var state = instruments.get(node.getString("id"));
            if (state == null) continue;
            if (!legacy(node)) {
                state.gyro.zeroRollPitchYaw();
                if (!launch) {
                    state.accel.zeroIntegratedVelo();
                    state.baro.zeroAlt();
                    state.gps.zeroAlt();
                }
            }
            snapshot(node, state, now);
        }
    }
}
