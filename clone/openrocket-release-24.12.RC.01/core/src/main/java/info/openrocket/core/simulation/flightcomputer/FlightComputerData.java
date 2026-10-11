package info.openrocket.core.simulation.flightcomputer;

import edu.mit.rocket_team.FC.RTFC;
import info.openrocket.core.simulation.*;
import info.openrocket.core.unit.UnitGroup;
import jakarta.json.*;
import java.util.*;
import static info.openrocket.core.unit.UnitGroup.*;

/**
 * FC plot channels are actual held readings and latched outputs, sampled on accepted simulation
 * points.
 */
public final class FlightComputerData {
    /** Prevents instantiation of this utility class. */
    private FlightComputerData() {}

    private static final List<FlightDataType> TYPES = new ArrayList<>();

    /**
     * Creates an FC plot data type with its stable symbol and engineering units.
     *
     * @param name display name or lookup key
     * @param symbol symbol
     * @param unit unit
     * @return the resulting FlightDataType
     */
    private static FlightDataType type(String name, String symbol, UnitGroup unit) {
        var t = FlightDataType.getType("FC: " + name, "FC_" + symbol, unit);
        TYPES.add(t);
        return t;
    }

    public static final FlightDataType STATE = type("state code", "state", UNITS_NONE);
    public static final FlightDataType AIRBRAKE =
            type("airbrake exposed fraction", "airbrake", UNITS_COEFFICIENT);
    public static final FlightDataType AIRBRAKE_ANGLE =
            type("airbrake servo angle", "air_angle", UNITS_ANGLE);
    public static final FlightDataType ROLL_ANGLE =
            type("roll servo angle", "roll_angle", UNITS_ANGLE);
    public static final FlightDataType ROLL_TAB_ANGLE =
            type("physical roll tab angle", "roll_tab_angle", UNITS_ANGLE);
    public static final FlightDataType AIRBRAKE_PHYSICAL =
            type("physical airbrake exposed fraction", "airbrake_physical", UNITS_COEFFICIENT);
    public static final FlightDataType[] PWM = {
        type("airbrake PWM pulse", "pwm0", UNITS_TIME_STEP),
        type("roll servo 2 PWM pulse", "pwm2", UNITS_TIME_STEP),
        type("roll servo 3 PWM pulse", "pwm3", UNITS_TIME_STEP)
    };
    public static final FlightDataType[] RECOVERY_ARMED = recoveryChannels("armed", "armed");
    public static final FlightDataType[] RECOVERY_PULSE = recoveryChannels("active pulse", "pulse");

    /**
     * Returns the logical recovery-channel plot types.
     *
     * @param label label
     * @param symbol symbol
     * @return the logical recovery-channel plot types
     */
    private static FlightDataType[] recoveryChannels(String label, String symbol) {
        var result = new FlightDataType[6];
        for (int i = 0; i < 6; i++)
            result[i] =
                    type(
                            "recovery channel " + i + " " + label,
                            "recovery_" + symbol + i,
                            UNITS_COEFFICIENT);
        return result;
    }

    /**
     * Returns a plot type for one physical recovery device's actual deployment state.
     *
     * @param id stable identifier
     * @param name display name or lookup key
     * @return a plot type for one physical recovery device's actual deployment state
     */
    public static FlightDataType recoveryDevice(String id, String name) {
        return FlightDataType.getType(
                "FC: recovery deployed ["
                        + name.replace(",", ";")
                        + " · "
                        + id.substring(0, 8)
                        + "]",
                "FC_deployed_" + UUID.fromString(id).toString().replace("-", ""),
                UNITS_COEFFICIENT);
    }

    /**
     * Tests whether a plot type represents recovery-channel or device state.
     *
     * @param type type
     * @return true when the condition described above holds
     */
    public static boolean isRecoveryType(FlightDataType type) {
        return type.getSymbol().matches("FC_(recovery_(armed|pulse)[0-5]|deployed_[0-9a-f]{32})");
    }

    /**
     * Identifies FC states and commands that require held-value plotting.
     *
     * @param type type
     * @return true when the condition described above holds
     */
    public static boolean isDiscrete(FlightDataType type) {
        return type.equals(STATE)
                || isRecoveryType(type)
                || (FlightComputerControllerPlots.isSymbol(type.getSymbol())
                        ? FlightComputerControllerPlots.isDiscrete(type.getSymbol())
                        : isActuatorSymbol(type.getSymbol()));
    }

    public static final FlightDataType BARO_ALTITUDE =
            type("measured barometric height (filtered and zeroed)", "baro_h", UNITS_DISTANCE);
    public static final FlightDataType GPS_ALTITUDE =
            type("measured GPS height (zeroed)", "gps_h", UNITS_DISTANCE);
    public static final FlightDataType VELOCITY =
            type("estimated velocity (integrated sensor X)", "velocity", UNITS_VELOCITY);

    public record Pair(String label, FlightDataType measured, FlightDataType truth) {}

    /**
     * Builds a measured-versus-physical plot channel pair.
     *
     * @param label label
     * @param symbol symbol
     * @param unit unit
     * @return the resulting Pair
     */
    private static Pair pair(String label, String symbol, UnitGroup unit) {
        return new Pair(
                label,
                type("measured " + label, symbol + "_measured", unit),
                type("physical " + label, symbol + "_truth", unit));
    }

    public static final Pair PRESSURE = pair("pressure", "pressure", UNITS_PRESSURE),
            TEMPERATURE = pair("temperature", "temperature", UNITS_TEMPERATURE);
    public static final Pair LATITUDE = pair("GPS latitude", "latitude", UNITS_ANGLE),
            LONGITUDE = pair("GPS longitude", "longitude", UNITS_ANGLE);
    public static final Pair[] ACCEL = {
        pair("accelerometer X", "ax", UNITS_ACCELERATION),
        pair("accelerometer Y", "ay", UNITS_ACCELERATION),
        pair("accelerometer Z", "az", UNITS_ACCELERATION)
    };
    public static final Pair[] GYRO = {
        pair("gyroscope X", "gx", UNITS_ROLL),
        pair("gyroscope Y", "gy", UNITS_ROLL),
        pair("gyroscope Z", "gz", UNITS_ROLL)
    };
    public static final List<Pair> PAIRS;
    public static final FlightDataType[] ALL_TYPES;

    static {
        var pairs = new ArrayList<Pair>();
        pairs.add(PRESSURE);
        pairs.add(TEMPERATURE);
        pairs.add(LATITUDE);
        pairs.add(LONGITUDE);
        pairs.addAll(List.of(ACCEL));
        pairs.addAll(List.of(GYRO));
        PAIRS = List.copyOf(pairs);
        ALL_TYPES = TYPES.toArray(FlightDataType[]::new);
    }

    public record ActuatorChannels(FlightDataType command, FlightDataType physical) {}

    /**
     * Creates commanded and physical plot channels for an independently addressed actuator.
     *
     * @param port logical controller port name
     * @param id stable identifier
     * @param name display name or lookup key
     * @return the resulting ActuatorChannels
     * @throws IllegalArgumentException when rejected: Unknown actuator port
     */
    public static ActuatorChannels actuatorChannels(String port, String id, String name) {
        if (!Set.of("roll", "cant", "airbrakes").contains(port))
            throw new IllegalArgumentException("Unknown actuator port " + port);
        String key = port + "_" + java.util.UUID.fromString(id).toString().replace("-", "");
        UnitGroup units = !port.equals("airbrakes") ? UNITS_ANGLE : UNITS_COEFFICIENT;
        String label =
                port.equals("roll")
                        ? "roll tab angle"
                        : port.equals("cant") ? "fin cant angle" : "airbrake fraction";
        String suffix = " [" + name.replace(",", ";") + " · " + id.substring(0, 8) + "]";
        return new ActuatorChannels(
                FlightDataType.getType("FC: latched " + label + suffix, "FC_cmd_" + key, units),
                FlightDataType.getType("FC: physical " + label + suffix, "FC_phys_" + key, units));
    }

    /**
     * Identifies structured actuator symbols during log import.
     *
     * @param symbol symbol
     * @return true when the condition described above holds
     */
    public static boolean isActuatorSymbol(String symbol) {
        return FlightComputerControllerPlots.isSymbol(symbol)
                || symbol.matches("FC_((cmd|phys)_(roll|cant|airbrakes)|deployed)_[0-9a-f]{32}");
    }

    /**
     * Reconstructs an actuator plot type from its symbol and metadata.
     *
     * @param name display name or lookup key
     * @param symbol symbol
     * @return the resulting FlightDataType
     * @throws IllegalArgumentException when rejected: Invalid FC actuator symbol
     */
    public static FlightDataType actuatorType(String name, String symbol) {
        if (FlightComputerControllerPlots.isSymbol(symbol))
            return FlightComputerControllerPlots.type(name, symbol);
        if (!isActuatorSymbol(symbol))
            throw new IllegalArgumentException("Invalid FC actuator symbol");
        return FlightDataType.getType(
                name,
                symbol,
                (symbol.contains("_roll_") || symbol.contains("_cant_"))
                        ? UNITS_ANGLE
                        : UNITS_COEFFICIENT);
    }

    /**
     * Maps a design state ID to its saved numeric plot code.
     *
     * @param design external flight-computer design snapshot
     * @param id stable identifier
     * @param phase phase
     * @return computed integer value
     */
    public static int stateCode(FlightComputerDesign design, String id, int phase) {
        if (design == null || !design.hasStateMachine()) return phase;
        int builtin = FlightComputerStateMachine.PHASES.indexOf(id);
        if (builtin >= 0) return builtin;
        return 100
                + FlightComputerStateMachine.states(design.stateMachine()).stream()
                        .map(s -> s.getString("id"))
                        .sorted()
                        .toList()
                        .indexOf(id);
    }

    /**
     * Builds the state-code-to-name mapping used by replay and 3D labels.
     *
     * @param design external flight-computer design snapshot
     * @return the resulting Map&lt;String, String&gt;
     */
    public static Map<String, String> stateNames(FlightComputerDesign design) {
        var names = new LinkedHashMap<String, String>();
        String[] defaults = {"GROUND", "PREFLIGHT", "FLIGHT", "APOGEE", "MAIN", "END"};
        for (int i = 0; i < defaults.length; i++) names.put("state" + i, defaults[i]);
        names.put("stateMixed", "MIXED STATES");
        if (design != null && design.hasStateMachine())
            for (var state : FlightComputerStateMachine.states(design.stateMachine()))
                names.put(
                        "state" + stateCode(design, state.getString("id"), 0),
                        state.getString("name"));
        return names;
    }

    /**
     * Records the FC sample alongside simulator truth without substituting truth for measured
     * signals.
     *
     * @param branch flight-data branch receiving or providing samples
     * @param fc out value
     * @param truth a value
     * @param design external flight-computer design snapshot
     */
    public static void record(
            FlightDataBranch branch, RTFC fc, RTFC.Inputs truth, FlightComputerDesign design) {
        for (int i = 0; i < 6; i++) {
            branch.setValue(RECOVERY_ARMED[i], fc.pyroController.isArmed(i) ? 1 : 0);
            branch.setValue(RECOVERY_PULSE[i], fc.pyroController.isFired(i) ? 1 : 0);
        }
        var out = fc.getOutput();
        branch.setValue(STATE, stateCode(design, fc.getDesignStateId(), fc.getState().ID));
        branch.setValue(AIRBRAKE, out.exposedFraction());
        branch.setValue(AIRBRAKE_ANGLE, Math.toRadians(out.selectedAngle()));
        branch.setValue(ROLL_ANGLE, Math.toRadians(out.rollAngle()));
        branch.setValue(PWM[0], out.airbrakePulseUs() / 1e6);
        branch.setValue(PWM[1], out.servo2Us() / 1e6);
        branch.setValue(PWM[2], out.servo3Us() / 1e6);
        branch.setValue(BARO_ALTITUDE, fc.baro.getFilteredAltitude());
        branch.setValue(GPS_ALTITUDE, fc.gps.getHeight());
        branch.setValue(VELOCITY, fc.accel.getIntegratedVelo());
        pair(branch, PRESSURE, fc.baro.getPressure() * 100, truth.baro().getPressure() * 100);
        pair(
                branch,
                TEMPERATURE,
                fc.baro.getTemperature() + 273.15,
                truth.baro().getTemperature() + 273.15);
        pair(
                branch,
                LATITUDE,
                Math.toRadians(fc.gps.getLatitude()),
                Math.toRadians(truth.gps().getLatitude()));
        pair(
                branch,
                LONGITUDE,
                Math.toRadians(fc.gps.getLongitude()),
                Math.toRadians(truth.gps().getLongitude()));
        var a = truth.accel();
        var g = truth.gyro();
        double[] av =
                FlightComputerSensors.rotate(
                        design == null ? null : design.active("accelerometer"),
                        a.getAccelX(),
                        a.getAccelY(),
                        a.getAccelZ());
        double[] gv =
                FlightComputerSensors.rotate(
                        design == null ? null : design.active("gyroscope"),
                        g.getGyroX(),
                        g.getGyroY(),
                        g.getGyroZ());
        double[] measuredA = {fc.accel.getAccelX(), fc.accel.getAccelY(), fc.accel.getAccelZ()},
                measuredG = {fc.gyro.getGyroX(), fc.gyro.getGyroY(), fc.gyro.getGyroZ()};
        for (int i = 0; i < 3; i++) {
            pair(branch, ACCEL[i], measuredA[i], av[i]);
            pair(branch, GYRO[i], Math.toRadians(measuredG[i]), Math.toRadians(gv[i]));
        }
    }

    /**
     * Builds a measured-versus-physical plot channel pair.
     *
     * @param b b
     * @param p p
     * @param measured measured
     * @param truth truth
     */
    private static void pair(FlightDataBranch b, Pair p, double measured, double truth) {
        b.setValue(p.measured(), measured);
        b.setValue(p.truth(), truth);
    }

    /**
     * Serializes an event-time FC snapshot for the structured run log.
     *
     * @param event event to process
     * @return the resulting JsonObject
     */
    public static JsonObject eventSnapshot(FlightEvent event) {
        var json =
                Json.createObjectBuilder()
                        .add("type", event.getType().name())
                        .add("time", event.getTime());
        if (event.getData() instanceof RecoveryDeployment recovery)
            json.add("recovery", recovery.json());
        if (event.getData() instanceof info.openrocket.core.logging.SimulationAbort abort)
            json.add("abortCause", abort.getCause().name());
        return json.build();
    }

    /**
     * Serializes the current FC measurements, commands and physical comparison values.
     *
     * @param branch flight-data branch receiving or providing samples
     * @return the resulting JsonObject
     */
    public static JsonObject snapshot(FlightDataBranch branch) {
        var values = Json.createObjectBuilder();
        for (var t : branch.getTypes()) {
            double v = branch.getLast(t);
            if (Double.isFinite(v)) values.add(t.getSymbol(), v);
        }
        return Json.createObjectBuilder().add("version", 1).add("values", values).build();
    }

    /**
     * Resolves a stored FC plot symbol to its data type.
     *
     * @return the resulting Map&lt;String, FlightDataType&gt;
     */
    public static Map<String, FlightDataType> bySymbol() {
        var map = new HashMap<String, FlightDataType>();
        for (var t : FlightDataType.ALL_TYPES) map.put(t.getSymbol(), t);
        for (var t : ALL_TYPES) map.put(t.getSymbol(), t);
        return map;
    }
}
