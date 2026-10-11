package info.openrocket.core.simulation.flightcomputer;

import jakarta.json.*;
import java.util.*;

/**
 * Immutable portable controller definition; all control angles/rates use degrees, time uses
 * microseconds.
 */
public record FlightComputerController(JsonObject json) {
    public enum Type {
        PID,
        VELOCITY_PID,
        CONSTANT,
        LEGACY_ROLL,
        LEGACY_APOGEE
    }

    public enum Mode {
        TABS,
        CANT,
        AIRBRAKES
    }

    /**
     * Returns the controller's stable design ID.
     *
     * @return the controller's stable design ID
     */
    public String id() {
        return json.getString("id");
    }

    /**
     * Returns the user-visible controller name.
     *
     * @return the user-visible controller name
     */
    public String name() {
        return json.getString("name");
    }

    /**
     * Returns the selected controller algorithm.
     *
     * @return the selected controller algorithm
     */
    public Type type() {
        return Type.valueOf(s("type"));
    }

    /**
     * Returns the actuator output mode.
     *
     * @return the actuator output mode
     */
    public Mode mode() {
        return Mode.valueOf(s("mode"));
    }

    /**
     * Returns whether this controller participates in scheduling and actuator ownership.
     *
     * @return whether this controller participates in scheduling and actuator ownership
     */
    public boolean enabled() {
        return json.getBoolean("enabled", false);
    }

    /**
     * Reads a string-valued controller setting.
     *
     * @param key configuration or lookup key
     * @return formatted or resolved text
     */
    public String s(String key) {
        return json.getString(key, "");
    }

    /**
     * Reads a numeric controller setting.
     *
     * @param key configuration or lookup key
     * @return computed numeric value
     */
    public double n(String key) {
        return json.getJsonNumber(key).doubleValue();
    }

    /**
     * Reads a controller timing setting in integer microseconds.
     *
     * @param key configuration or lookup key
     * @return computed integer value
     */
    public long us(String key) {
        return json.getJsonNumber(key).longValueExact();
    }

    /**
     * Reads a boolean controller setting.
     *
     * @param key configuration or lookup key
     * @return true when the condition described above holds
     */
    public boolean flag(String key) {
        return json.getBoolean(key, false);
    }

    /**
     * Returns the state IDs in which the block may be active.
     *
     * @return the state IDs in which the block may be active
     */
    public Set<String> states() {
        var result = new LinkedHashSet<String>();
        for (var x : json.getJsonArray("states").getValuesAs(JsonString.class))
            result.add(x.getString());
        return Set.copyOf(result);
    }

    /**
     * Tests whether this block is enabled in the specified design state.
     *
     * @param state current flight phase or state
     * @return true when the condition described above holds
     */
    public boolean active(String state) {
        return enabled() && states().contains(state);
    }

    /**
     * Returns a controller definition with one immutable JSON property replaced.
     *
     * @param value replacement or input value
     * @return a controller definition with one immutable JSON property replaced
     */
    public FlightComputerController with(JsonObject value) {
        return new FlightComputerController(value);
    }

    /**
     * Creates a controller definition with defaults for the selected algorithm.
     *
     * @param design external flight-computer design snapshot
     * @return the resulting FlightComputerController
     */
    public static FlightComputerController create(FlightComputerDesign design) {
        String gyro = source(design, "gyroscope"), accel = source(design, "accelerometer");
        var phases = FlightComputerStateMachine.states(design.stateMachine());
        String flight =
                phases.stream()
                        .filter(x -> x.getString("phase").equals("flight"))
                        .map(x -> x.getString("id"))
                        .findFirst()
                        .orElse(phases.get(0).getString("id"));
        String preflight =
                phases.stream()
                        .filter(x -> x.getString("phase").equals("preflight"))
                        .map(x -> x.getString("id"))
                        .findFirst()
                        .orElse(phases.get(0).getString("id"));
        return new FlightComputerController(
                Json.createObjectBuilder()
                        .add("id", UUID.randomUUID().toString())
                        .add("name", "New controller")
                        .add("enabled", false)
                        .add("type", "PID")
                        .add("mode", "TABS")
                        .add("board", "")
                        .add("source", gyro)
                        .add("channel", "roll_rate")
                        .add("derivativeSource", "")
                        .add("derivativeChannel", "")
                        .add("velocitySource", accel)
                        .add("velocityChannel", "velocity")
                        .add("baseline", "fixed")
                        .add("setpoint", 0)
                        .add("captureState", preflight)
                        .add("captureAverageUs", 0)
                        .add("wrapAngle", false)
                        .add("resetIntegral", true)
                        .add("inactiveHold", false)
                        .add("kp", 0)
                        .add("ki", 0)
                        .add("kd", 0)
                        .add("integralLimit", 1000)
                        .add("filterUs", 20000)
                        .add("constantValue", 0)
                        .add("trim", 0)
                        .add("direction", 1)
                        .add("minimum", -15)
                        .add("maximum", 15)
                        .add("referenceSpeed", 200)
                        .add("speedFloor", 20)
                        .add("exponent", 2)
                        .add("minimumWeight", 0)
                        .add("maximumWeight", 4)
                        .add("activationSpeed", 0)
                        .add("speedHysteresis", 1)
                        .add("periodUs", 10000)
                        .add("executionUs", 0)
                        .add("staleUs", 200000)
                        .add("invalidHoldUs", 0)
                        .add("states", Json.createArrayBuilder().add(flight))
                        .add("layout", Json.createObjectBuilder().add("x", 80).add("y", 80))
                        .build());
    }

    /**
     * Builds a stable sensor source/channel selection.
     *
     * @param d n value
     * @param role role
     * @return formatted or resolved text
     */
    public static String source(FlightComputerDesign d, String role) {
        var n = d.active(role);
        return n == null ? "" : n.getString("id");
    }

    /**
     * Reads the controller definitions from a design JSON object.
     *
     * @param d d
     * @return the resulting List&lt;FlightComputerController&gt;
     */
    public static List<FlightComputerController> list(FlightComputerDesign d) {
        if (!d.json().containsKey("controllers")) return List.of();
        return d.json().getJsonArray("controllers").getValuesAs(JsonObject.class).stream()
                .map(FlightComputerController::new)
                .toList();
    }

    /**
     * Returns design JSON with its controller definitions replaced.
     *
     * @param d n value
     * @param blocks blocks
     * @return design JSON with its controller definitions replaced
     */
    public static FlightComputerDesign replace(
            FlightComputerDesign d, List<FlightComputerController> blocks) {
        var b = Json.createArrayBuilder();
        blocks.forEach(c -> b.add(c.json()));
        var legacy =
                Json.createObjectBuilder(
                        d.json().getJsonObject("legacyInputs") == null
                                ? JsonValue.EMPTY_JSON_OBJECT
                                : d.json().getJsonObject("legacyInputs"));
        for (String role : List.of("barometer", "gyroscope", "accelerometer", "gps")) {
            var n = d.active(role);
            if (n != null) legacy.add(role, n.getString("id"));
        }
        return new FlightComputerDesign(
                Json.createObjectBuilder(d.json())
                        .add("version", 2)
                        .add("controllers", b)
                        .add("legacyInputs", legacy)
                        .build());
    }

    /**
     * Checks controller parameters, signal selections, timing and state references.
     *
     * @param d d
     * @throws IllegalArgumentException when rejected: Controller name is required
     */
    public void validate(FlightComputerDesign d) {
        UUID.fromString(id());
        if (name().isBlank()) throw new IllegalArgumentException("Controller name is required");
        type();
        mode();
        var allowed = create(d).json().keySet();
        if (!allowed.containsAll(json.keySet()) || !json.keySet().containsAll(allowed))
            throw new IllegalArgumentException("Invalid controller fields: " + name());
        if (!s("board").isEmpty()
                && (d.node(s("board")) == null
                        || !FlightComputerModels.isBoard(d.node(s("board")).getString("type"))))
            throw new IllegalArgumentException("Controller parent board is missing");
        for (var entry : json.entrySet())
            if (entry.getValue() instanceof JsonNumber value
                    && !Double.isFinite(value.doubleValue()))
                throw new IllegalArgumentException("Nonfinite " + entry.getKey());
        for (String key :
                List.of(
                        "periodUs",
                        "executionUs",
                        "staleUs",
                        "invalidHoldUs",
                        "filterUs",
                        "captureAverageUs"))
            if (us(key) < 0 || us(key) > 86_400_000_000L)
                throw new IllegalArgumentException("Invalid controller time: " + key);
        if (us("periodUs") == 0 || us("staleUs") == 0)
            throw new IllegalArgumentException(
                    "Controller period and stale timeout must be positive");
        if (n("minimum") >= n("maximum") || n("integralLimit") < 0 || Math.abs(n("direction")) != 1)
            throw new IllegalArgumentException("Invalid controller limits or direction");
        double lo = mode() == Mode.AIRBRAKES ? 0 : -90, hi = mode() == Mode.AIRBRAKES ? 1 : 90;
        if (n("minimum") < lo
                || n("maximum") > hi
                || n("trim") < n("minimum")
                || n("trim") > n("maximum"))
            throw new IllegalArgumentException("Output limits/neutral outside " + lo + ".." + hi);
        if (n("referenceSpeed") <= 0
                || n("speedFloor") <= 0
                || n("exponent") < 0
                || n("exponent") > 8
                || n("minimumWeight") < 0
                || n("maximumWeight") < n("minimumWeight")
                || n("activationSpeed") < 0
                || n("speedHysteresis") < 0)
            throw new IllegalArgumentException("Invalid velocity weighting");
        if (!Set.of("fixed", "ground", "entry").contains(s("baseline")))
            throw new IllegalArgumentException("Unknown baseline mode");
        var stateIds = new HashSet<String>(FlightComputerStateMachine.PHASES);
        if (d.hasStateMachine()) {
            stateIds.clear();
            FlightComputerStateMachine.states(d.stateMachine())
                    .forEach(x -> stateIds.add(x.getString("id")));
        }
        if (!stateIds.containsAll(states())
                || (s("baseline").equals("entry") && !stateIds.contains(s("captureState"))))
            throw new IllegalArgumentException("Unknown controller state");
        if (enabled() && states().isEmpty())
            throw new IllegalArgumentException("Select at least one active state");
        if (type() != Type.CONSTANT) {
            var signal = FlightComputerSignals.descriptor(d, s("source"), s("channel"));
            if (flag("wrapAngle") && !signal.unit().equals("deg"))
                throw new IllegalArgumentException("Angle wrapping requires an angle input");
            if (!s("derivativeSource").isBlank()) {
                var derivative =
                        FlightComputerSignals.descriptor(
                                d, s("derivativeSource"), s("derivativeChannel"));
                if (!derivative.unit().equals(signal.derivativeUnit()))
                    throw new IllegalArgumentException("Derivative input has incompatible units");
            }
        }
        if ((type() == Type.VELOCITY_PID
                        || type() == Type.LEGACY_ROLL
                        || type() == Type.LEGACY_APOGEE)
                && !FlightComputerSignals.descriptor(d, s("velocitySource"), s("velocityChannel"))
                        .unit()
                        .equals("m/s"))
            throw new IllegalArgumentException("Weighting input must be a velocity");
        if (type() == Type.LEGACY_ROLL && !s("channel").equals("roll"))
            throw new IllegalArgumentException(
                    "Existing roll controller requires roll angle input");
        if (type() == Type.LEGACY_APOGEE
                && (mode() != Mode.AIRBRAKES || !s("channel").equals("altitude")))
            throw new IllegalArgumentException(
                    "Existing apogee controller requires barometric altitude and airbrake output");
    }
}
