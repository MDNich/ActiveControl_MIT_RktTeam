package info.openrocket.core.simulation.flightcomputer;

import edu.mit.rocket_team.FC.RTFC;
import info.openrocket.core.simulation.*;
import info.openrocket.core.unit.UnitGroup;
import jakarta.json.*;
import java.util.*;

/**
 * Self-describing, stable per-block channels. Unit codes are saved in symbols for standalone
 * log/result import.
 */
public final class FlightComputerControllerPlots {
    /** Prevents instantiation of this utility class. */
    private FlightComputerControllerPlots() {}

    private static final Map<String, String> LABELS =
            Map.ofEntries(
                    Map.entry("measurement", "measured input"),
                    Map.entry("baseline", "baseline"),
                    Map.entry("error", "error"),
                    Map.entry("p", "P term"),
                    Map.entry("i", "I term"),
                    Map.entry("d", "D term"),
                    Map.entry("weight", "velocity weight"),
                    Map.entry("requested", "requested output"),
                    Map.entry("output", "limited output"),
                    Map.entry("latched", "latched output"),
                    Map.entry("active", "active"),
                    Map.entry("valid", "input valid"),
                    Map.entry("saturated", "saturated"),
                    Map.entry("age", "sample age"));

    /**
     * Maps a controller quantity to its stable log unit code.
     *
     * @param unit unit
     * @return formatted or resolved text
     */
    public static String unitCode(String unit) {
        return switch (unit) {
            case "deg" -> "angle";
            case "deg/s" -> "rate";
            case "m" -> "distance";
            case "m/s" -> "velocity";
            case "m/s²" -> "accel";
            case "hPa" -> "pressure";
            case "°C" -> "temperature";
            default -> "none";
        };
    }

    /**
     * Resolves a stored unit code to an OpenRocket unit group.
     *
     * @param code code
     * @return the resulting UnitGroup
     * @throws IllegalArgumentException when rejected: Unknown controller plot unit
     */
    public static UnitGroup units(String code) {
        return switch (code) {
            case "angle" -> UnitGroup.UNITS_ANGLE;
            case "rate" -> UnitGroup.UNITS_ROLL;
            case "distance" -> UnitGroup.UNITS_DISTANCE;
            case "velocity" -> UnitGroup.UNITS_VELOCITY;
            case "accel" -> UnitGroup.UNITS_ACCELERATION;
            case "pressure" -> UnitGroup.UNITS_PRESSURE;
            case "temperature" -> UnitGroup.UNITS_TEMPERATURE;
            case "time" -> UnitGroup.UNITS_TIME_STEP;
            case "none" -> UnitGroup.UNITS_COEFFICIENT;
            default -> throw new IllegalArgumentException("Unknown controller plot unit");
        };
    }

    /**
     * Tests whether a plot symbol belongs to a controller diagnostic channel.
     *
     * @param s s
     * @return true when the condition described above holds
     */
    public static boolean isSymbol(String s) {
        return s.matches(
                "FC_ctrl_[0-9a-f]{32}_(measurement|baseline|error|p|i|d|weight|requested|output|latched|active|valid|saturated|age)_(angle|rate|distance|velocity|accel|pressure|temperature|time|none)");
    }

    /**
     * Identifies controller channels that must be plotted as held values rather than interpolated
     * curves.
     *
     * @param symbol symbol
     * @return true when the condition described above holds
     */
    public static boolean isDiscrete(String symbol) {
        return symbol.matches(
                "FC_ctrl_[0-9a-f]{32}_(requested|output|latched|active|valid|saturated)_.*");
    }

    /**
     * Creates a plot data type with the specified stable controller identity and units.
     *
     * @param name display name or lookup key
     * @param symbol symbol
     * @return the resulting FlightDataType
     * @throws IllegalArgumentException when rejected: Invalid controller plot symbol
     */
    public static FlightDataType type(String name, String symbol) {
        if (!isSymbol(symbol)) throw new IllegalArgumentException("Invalid controller plot symbol");
        return FlightDataType.getType(
                name, symbol, units(symbol.substring(symbol.lastIndexOf('_') + 1)));
    }

    /**
     * Builds the diagnostic channel set for a controller block.
     *
     * @param design external flight-computer design snapshot
     * @param c selected channel or controller
     * @return the resulting Map&lt;String, FlightDataType&gt;
     */
    public static Map<String, FlightDataType> channels(
            FlightComputerDesign design, FlightComputerController c) {
        var result = new LinkedHashMap<String, FlightDataType>();
        String input =
                c.type() == FlightComputerController.Type.CONSTANT
                        ? "none"
                        : unitCode(
                                FlightComputerSignals.descriptor(
                                                design, c.s("source"), c.s("channel"))
                                        .unit());
        String output = c.mode() == FlightComputerController.Mode.AIRBRAKES ? "none" : "angle";
        for (String key :
                List.of(
                        "measurement",
                        "baseline",
                        "error",
                        "p",
                        "i",
                        "d",
                        "weight",
                        "requested",
                        "output",
                        "latched",
                        "active",
                        "valid",
                        "saturated",
                        "age")) {
            String unit =
                    Set.of("measurement", "baseline", "error").contains(key)
                            ? input
                            : Set.of("p", "i", "d", "requested", "output", "latched").contains(key)
                                    ? output
                                    : key.equals("age") ? "time" : "none";
            String symbol =
                    "FC_ctrl_"
                            + UUID.fromString(c.id()).toString().replace("-", "")
                            + "_"
                            + key
                            + "_"
                            + unit;
            result.put(
                    key,
                    type(
                            "FC: "
                                    + c.name().replace(",", ";")
                                    + " · "
                                    + (c.type() == FlightComputerController.Type.LEGACY_APOGEE
                                                    && key.equals("error")
                                            ? "predicted apogee error"
                                            : LABELS.get(key))
                                    + " ["
                                    + c.id().substring(0, 8)
                                    + "]",
                            symbol));
        }
        return result;
    }

    /**
     * Serializes controller channel identity and unit metadata.
     *
     * @param d d
     * @return the resulting JsonArray
     */
    public static JsonArray metadata(FlightComputerDesign d) {
        var a = Json.createArrayBuilder();
        if (d != null)
            for (var c : d.controllers())
                if (c.enabled())
                    channels(d, c)
                            .values()
                            .forEach(
                                    t ->
                                            a.add(
                                                    Json.createObjectBuilder()
                                                            .add("name", t.getName())
                                                            .add("symbol", t.getSymbol())
                                                            .add("source", c.s("source"))
                                                            .add("channel", c.s("channel"))));
        return a.build();
    }

    /**
     * Converts a controller diagnostic value into the plot channel's base units.
     *
     * @param t s value
     * @param field field
     * @param v v
     * @return computed numeric value
     */
    private static double converted(FlightDataType t, String field, double v) {
        String s = t.getSymbol();
        if (s.endsWith("_angle") || s.endsWith("_rate")) return Math.toRadians(v);
        if (s.endsWith("_pressure")) return v * 100;
        if (s.endsWith("_temperature") && !field.equals("error")) return v + 273.15;
        return v;
    }

    /**
     * Stores the latest controller diagnostics in a flight-data branch.
     *
     * @param branch flight-data branch receiving or providing samples
     * @param fc fc
     * @param d d
     */
    public static void record(FlightDataBranch branch, RTFC fc, FlightComputerDesign d) {
        values(fc, d).forEach(branch::setValue);
    }

    /**
     * Collects controller diagnostic values for structured logging.
     *
     * @param fc values value
     * @param d d
     * @return the resulting Map&lt;FlightDataType, Double&gt;
     */
    public static Map<FlightDataType, Double> values(RTFC fc, FlightComputerDesign d) {
        var result = new LinkedHashMap<FlightDataType, Double>();
        if (fc.getControllers() == null || d == null) return result;
        var values = fc.getControllers().values();
        for (var c : d.controllers()) {
            var v = values.get(c.id());
            if (v == null) continue;
            var channels = fc.getControllers().plotChannels(c.id());
            var samples =
                    Map.ofEntries(
                            Map.entry("measurement", v.measurement()),
                            Map.entry("baseline", v.baseline()),
                            Map.entry("error", v.error()),
                            Map.entry("p", v.p()),
                            Map.entry("i", v.i()),
                            Map.entry("d", v.d()),
                            Map.entry("weight", v.weight()),
                            Map.entry("requested", v.requested()),
                            Map.entry("output", v.output()),
                            Map.entry(
                                    "latched",
                                    fc.getOutput()
                                            .controllerOutputs()
                                            .getOrDefault(c.id(), c.n("trim"))),
                            Map.entry("active", v.active() ? 1.0 : 0.0),
                            Map.entry("valid", v.valid() ? 1.0 : 0.0),
                            Map.entry("saturated", v.saturated() ? 1.0 : 0.0),
                            Map.entry(
                                    "age",
                                    v.sampleUs() < 0
                                            ? Double.NaN
                                            : (fc.trace.bootUs() - v.sampleUs()) / 1e6));
            samples.forEach(
                    (k, n) -> result.put(channels.get(k), converted(channels.get(k), k, n)));
        }
        return result;
    }

    /**
     * Serializes one controller diagnostic sample for later import.
     *
     * @param fc fc
     * @param d d
     * @param time time
     * @return the resulting JsonObject
     */
    public static JsonObject logSnapshot(RTFC fc, FlightComputerDesign d, double time) {
        var a = Json.createObjectBuilder();
        values(fc, d)
                .forEach(
                        (type, value) -> {
                            if (Double.isFinite(value)) a.add(type.getSymbol(), value);
                        });
        return Json.createObjectBuilder().add("time", time).add("values", a).build();
    }
}
