package edu.mit.rocket_team.FC;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Objects;
import info.openrocket.core.simulation.flightcomputer.*;
import jakarta.json.JsonObject;
import edu.mit.rocket_team.control.*;
import edu.mit.rocket_team.control.airbrakes.RTAirbrakesController;
import edu.mit.rocket_team.instrument.*;
import edu.mit.rocket_team.internal.*;
import edu.mit.rocket_team.telemetry.RTTelemetryEngine;
import edu.mit.rocket_team.telemetry.TelemetryLinkSettings;
import edu.mit.rocket_team.AV.*;
import edu.mit.rocket_team.util.*;
import edu.mit.rocket_team.util.data.*;
import static edu.mit.rocket_team.util.RTRocketState.*;
import static edu.mit.rocket_team.util.RTUtilLibrary.*;

/** FC/FC.ino translated in place. No simulator truth enters the control path. */
public class RTFC {
    public static final long LOOP_US = 10_000, PWM_US = 20_000;
    public static final float CLOSED_ANGLE = -67.0f, OPEN_ANGLE = -117.0f;
    public final Trace trace;
    public final RTAccel accel;
    public final RTBaro baro;
    public final RTGPS gps;
    public final RTGyro gyro;
    public final RTRollController rollController;
    public final RTAirbrakesController airbrakesController;
    public final RTPyroController pyroController;
    public final RTPowerBoard powerBoard;
    public final RTFlashDriver flashDriver;
    public final RTTelemetryEngine telemetry;
    private final RTCamera cam0, cam1;
    private final RTVTX vtx;
    private RTRocketState currentState = GROUND_TESTING, recState = GROUND_TESTING;
    private Inputs pendingInput;
    private FlightComputerStateMachine stateMachine;
    private JavaBoardRuntime boards;
    private FlightComputerSignals controllerSignals;
    private FlightComputerControllerRuntime controllers;
    private java.util.function.BiConsumer<String, String> controllerWriteCheck =
            (port, target) -> {};

    /**
     * Returns the optional per-run controller-block scheduler.
     *
     * @return the optional per-run controller-block scheduler
     */
    public FlightComputerControllerRuntime getControllers() {
        return controllers;
    }

    /**
     * Installs the ownership check used before custom Java boards write physical actuator targets.
     *
     * @param check output ownership validator
     */
    public void setControllerWriteCheck(java.util.function.BiConsumer<String, String> check) {
        controllerWriteCheck = check;
    }

    /**
     * Runs controller starts and completions due at this boot-clock instant.
     *
     * @param now now
     */
    public void dispatchControllers(long now) {
        if (controllers != null) controllers.dispatch(now, this, controllerSignals);
    }

    /**
     * Returns the measured-signal registry shared by controller blocks, or null for legacy designs.
     *
     * @return the measured-signal registry shared by controller blocks, or null for legacy designs
     */
    public FlightComputerSignals getControllerSignals() {
        return controllerSignals;
    }

    private long lastSensorAcquisitionUs, lastBoardCommandUs = -1, lastBoardSampleUs;

    /**
     * Returns the earliest Java-board or controller deadline in boot microseconds, or
     * Long.MAX_VALUE when none remain.
     *
     * @return the earliest Java-board or controller deadline in boot microseconds, or
     *     Long.MAX_VALUE when none remain
     */
    public long getNextBoardDeadlineUs() {
        return Math.min(
                boards == null ? Long.MAX_VALUE : boards.nextDeadlineUs(),
                controllers == null ? Long.MAX_VALUE : controllers.nextDeadlineUs());
    }

    /**
     * Returns the boot microsecond timestamp of the last connected Java-board servo command.
     *
     * @return the boot microsecond timestamp of the last connected Java-board servo command
     */
    public long getLastBoardCommandUs() {
        return lastBoardCommandUs;
    }

    /**
     * Returns the acquisition timestamp associated with the last connected Java-board servo
     * command.
     *
     * @return the acquisition timestamp associated with the last connected Java-board servo command
     */
    public long getLastBoardSampleUs() {
        return lastBoardSampleUs;
    }

    private String recoveryOrigin = "built_in";

    /**
     * Dispatches independent Java-board tasks using held measured inputs and publishes completed
     * commands.
     *
     * @param nowUs now in microseconds
     */
    public void dispatchBoards(long nowUs) {
        if (boards == null) return;
        boards.dispatch(
                nowUs,
                lastSensorAcquisitionUs,
                getDesignStateId(),
                key ->
                        switch (key) {
                            case "flight_ms" -> elapsed32(trace.millis(), flightBeginTime);
                            case "apogee_ms" -> elapsed32(trace.millis(), apogeeTime);
                            default -> signal(key);
                        },
                this::connected,
                (board, port, target, value, sampleUs) -> {
                    String previous = recoveryOrigin;
                    recoveryOrigin = "java_board:" + board;
                    try {
                        boardOutput(port, target, value);
                    } finally {
                        recoveryOrigin = previous;
                    }
                    if (connected(port) && !port.equals("recovery")) {
                        lastBoardCommandUs = trace.bootUs();
                        lastBoardSampleUs = sampleUs;
                    }
                },
                message -> trace.log("java.board", message));
    }

    /**
     * Returns the active design state ID, falling back to the firmware phase when no custom state
     * graph exists.
     *
     * @return the active design state ID, falling back to the firmware phase when no custom state
     *     graph exists
     */
    public String getDesignStateId() {
        return stateMachine == null
                ? (currentState.ID < 5
                        ? FlightComputerStateMachine.PHASES.get(currentState.ID)
                        : "end")
                : stateMachine.id();
    }

    /**
     * Tests whether the design exposes a component on the named controller port; legacy designs
     * expose all ports.
     *
     * @param port logical controller port name
     * @return true when the condition described above holds
     */
    public boolean connected(String port) {
        return design == null || design.active(port) != null;
    }

    private final java.util.Map<String, Float> namedRoll = new java.util.LinkedHashMap<>();
    private final java.util.Map<String, Float> namedAirbrakes = new java.util.LinkedHashMap<>();

    /**
     * Accepts a Java-board or state-action output after checking ownership. Missing hardware
     * discards commands; named targets preserve independent servo commands until PWM latching.
     *
     * @param port logical controller port name
     * @param value replacement or input value
     */
    private void boardOutput(String port, Double value) {
        boardOutput(port, null, value);
    }

    /**
     * Accepts a Java-board or state-action output after checking ownership. Missing hardware
     * discards commands; named targets preserve independent servo commands until PWM latching.
     *
     * @param port logical controller port name
     * @param target destination component or output target
     * @param value replacement or input value
     * @throws IllegalArgumentException when rejected: Named output unsupported:
     */
    private void boardOutput(String port, String target, Double value) {
        controllerWriteCheck.accept(port, target);
        if (!connected(port)) {
            trace.log(
                    "output.discard", "port=" + port + " value=" + value + " reason=no_component");
            return;
        }
        if (target != null) {
            switch (port) {
                case "roll":
                    namedRoll.put(target, value.floatValue());
                    rollSignalEnabled = true;
                    break;
                case "airbrakes":
                    namedAirbrakes.put(
                            target, (float) Math.min(value, setting("airbrakeLimit", 1)));
                    break;
                default:
                    throw new IllegalArgumentException("Named output unsupported: " + port);
            }
            trace.log("output.command", "port=" + port + " target=" + target + " value=" + value);
            return;
        }
        switch (port) {
            case "airbrakes":
                namedAirbrakes.clear();
                airbrakesEnabled = false;
                airbrakesSetAngle = dpToDeg((float) Math.min(value, setting("airbrakeLimit", 1)));
                break;
            case "roll":
                namedRoll.clear();
                rollControlEnabled = false;
                rollControlSetAngle = value.floatValue();
                rollSignalEnabled = true;
                break;
            case "recovery":
                fireChannels(value.intValue());
                break;
            default:
                throw new IllegalArgumentException("Unknown output " + port);
        }
        trace.log("output.command", "port=" + port + " value=" + value);
    }

    /**
     * Executes a state-entry action while tagging any recovery command with its state-machine
     * origin.
     *
     * @param action action identifier or state-entry action
     */
    private void entryAction(JsonObject action) {
        String previous = recoveryOrigin;
        recoveryOrigin = "state_machine:" + action.getString("type");
        try {
            applyEntryAction(action);
        } finally {
            recoveryOrigin = previous;
        }
    }

    /**
     * Dispatches a validated state action to calibration, flight transitions, outputs, recovery, or
     * logging.
     *
     * @param action action identifier or state-entry action
     * @throws IllegalArgumentException when rejected: Unknown state action
     */
    private void applyEntryAction(JsonObject action) {
        trace.log("state.action", action.toString());
        switch (action.getString("type")) {
            case "prepare":
                prepareFlight();
                break;
            case "launch":
                beginFlight();
                break;
            case "apogee":
                detectApogee();
                break;
            case "main":
                deployMain();
                break;
            case "finish":
                finishFlight();
                break;
            case "fire_recovery":
                fireChannels(action.getInt("channel"));
                break;
            case "close_controllers":
                closeControllers();
                break;
            case "airbrakes":
            case "roll":
                boardOutput(action.getString("type"), action.getJsonNumber("value").doubleValue());
                break;
            case "log":
                trace.log("state.message", action.getString("message"));
                break;
            default:
                throw new IllegalArgumentException("Unknown state action");
        }
    }

    private info.openrocket.core.simulation.flightcomputer.FlightComputerDesign design;
    private info.openrocket.core.simulation.flightcomputer.FlightComputerSensors sensors;

    /**
     * Installs a validated design, sensor models, PID gains, task schedulers and optional state
     * graph before boot.
     *
     * @param value replacement or input value
     * @throws IllegalStateException when rejected: Configure before FC setup
     */
    public void configure(
            info.openrocket.core.simulation.flightcomputer.FlightComputerDesign value) {
        if (value.telemetryCodec() != null)
            throw new IllegalArgumentException("Custom telemetry codecs require Mission Simulation");
        configureMission(value);
    }

    /** Configure a mission whose router owns declared codec encoding at the FC transmit boundary. */
    public void configureMission(
            info.openrocket.core.simulation.flightcomputer.FlightComputerDesign value) {
        if (initialized) throw new IllegalStateException("Configure before FC setup");
        value.requireRunnable();
        design = value;
        sensors = new info.openrocket.core.simulation.flightcomputer.FlightComputerSensors(value);
        baro.setFilterSamples(
                (int)
                        value.active("barometer")
                                .getJsonObject("properties")
                                .getJsonNumber("filterSamples")
                                .doubleValue());
        airbrakesController.setPidGains(
                (float) value.parameter("airbrakeKp"),
                (float) value.parameter("airbrakeKi"),
                (float) value.parameter("airbrakeKd"));
        rollController.setPidGains(
                (float) value.parameter("rollKp"),
                (float) value.parameter("rollKi"),
                (float) value.parameter("rollKd"));
        boards = new JavaBoardRuntime(value);
        if (!value.controllers().isEmpty()) {
            controllerSignals = new FlightComputerSignals(value);
            controllers = new FlightComputerControllerRuntime(value, trace);
        }
        if (value.hasStateMachine())
            stateMachine =
                    new FlightComputerStateMachine(
                            value.stateMachine(),
                            new FlightComputerStateMachine.Host() {
                                /**
                                 * Maps the design phase to the firmware state and records entry in
                                 * the run trace.
                                 *
                                 * @param id stable identifier
                                 * @param name display name or lookup key
                                 * @param phase phase
                                 */
                                public void enter(String id, String name, String phase) {
                                    currentState =
                                            switch (phase) {
                                                case "ground" -> GROUND_TESTING;
                                                case "preflight" -> PRE_FLIGHT;
                                                case "flight" -> FLIGHT;
                                                case "apogee" -> APOGEE;
                                                case "main" -> MAIN;
                                                default ->
                                                        throw new IllegalArgumentException(phase);
                                            };
                                    trace.log(
                                            "state.enter",
                                            "id=" + id + " name=" + name + " phase=" + phase);
                                }

                                /**
                                 * Forwards a state-machine action to the FC action dispatcher.
                                 *
                                 * @param action action identifier or state-entry action
                                 */
                                public void action(JsonObject action) {
                                    entryAction(action);
                                }

                                /**
                                 * Records the completed state transition and its firmware phase.
                                 *
                                 * @param from source state identifier
                                 * @param to destination state identifier
                                 */
                                public void transition(String from, String to) {
                                    trace.log(
                                            "fc.transition",
                                            "from="
                                                    + from
                                                    + " to="
                                                    + to
                                                    + " phase="
                                                    + currentState);
                                }
                            });
    }

    /**
     * Reads a design parameter, using the firmware default only when no design is installed.
     *
     * @param key configuration or lookup key
     * @param fallback value used when no explicit setting is available
     * @return computed numeric value
     */
    private double setting(String key, double fallback) {
        return design == null ? fallback : design.parameter(key);
    }

    /**
     * Evaluates a named design rule against measured FC signals, or uses the firmware condition for
     * a legacy run.
     *
     * @param id stable identifier
     * @param fallback value used when no explicit setting is available
     * @return true when the condition described above holds
     */
    private boolean rule(String id, boolean fallback) {
        return design == null ? fallback : design.test(id, this::signal);
    }

    /**
     * Resolves a legacy signal name to a measured sensor value, command, or virtual-clock duration.
     *
     * @param name display name or lookup key
     * @return computed numeric value
     */
    private double signal(String name) {
        return switch (name) {
            case "command" -> recState.ID;
            case "flight_ms" -> elapsed32(FCtime, flightBeginTime);
            case "apogee_ms" -> elapsed32(FCtime, apogeeTime);
            case "accel_vertical" -> accel.getVerticalAccelMinusGravity();
            case "baro_drop" -> baro.getMaxAlt() - baro.getFilteredAltitude();
            case "gps_drop" -> gps.getMaxAlt() - gps.getHeight();
            case "gps_fix" -> gps.getFixType();
            case "baro_altitude" -> baro.getFilteredAltitude();
            case "gps_altitude" -> gps.getHeight();
            case "state_ms" -> stateMachine == null ? 0 : stateMachine.stateAgeMs(trace.bootUs());
            case "time_ms" -> trace.bootUs() / 1000;
            case "velocity" -> accel.getIntegratedVelo();
            case "roll_deg" -> gyro.getRoll();
            case "pitch_deg" -> gyro.getPitch();
            case "yaw_deg" -> gyro.getYaw();
            default -> throw new IllegalArgumentException(name);
        };
    }

    private int loopPhase;
    private boolean initialized, prepared, airbrakesEnabled, rollControlEnabled, loggingEnabled;
    private boolean bpFired1, bpFired2, baroMaxAltReset, rollSignalEnabled = true;
    private float airbrakesSetAngle = CLOSED_ANGLE, rollControlSetAngle;
    private long FCtime, flightBeginTime, apogeeTime, lastTelem, lastPowerPkt, lastLoopUs = -1;
    private long loopCount, airbrakeUpdateCount, rollUpdateCount;
    private int packetNum, badPackets;
    private Output output =
            new Output(0, CLOSED_ANGLE, degToUsAirbrakes(CLOSED_ANGLE), 0, 0, 1495, 1430);

    /** Null accel/GPS means no new sample. Barometer/gyro retain last delivered readings. */
    public record Inputs(
            long acquisitionUs,
            RTAccelData accel,
            RTBaroData baro,
            RTGPSData gps,
            RTGyroData gyro,
            long gpsAcquisitionUs) {
        /**
         * Creates a Inputs value from the supplied acquisition us, accel, baro, gps, gyro.
         *
         * @param acquisitionUs acquisition in microseconds
         * @param accel accel
         * @param baro baro
         * @param gps gps
         * @param gyro gyro
         */
        public Inputs(
                long acquisitionUs,
                RTAccelData accel,
                RTBaroData baro,
                RTGPSData gps,
                RTGyroData gyro) {
            this(acquisitionUs, accel, baro, gps, gyro, acquisitionUs);
        }
    }

    public record Output(
            float requestedDeployment,
            float selectedAngle,
            int airbrakePulseUs,
            float exposedFraction,
            float rollAngle,
            int servo2Us,
            int servo3Us,
            java.util.Map<String, Float> rollAngles,
            java.util.Map<String, Float> airbrakeFractions,
            java.util.Map<String, Double> controllerOutputs,
            java.util.Set<String> activeControllers) {
        /**
         * Creates a Output value from the supplied requested deployment, selected angle, airbrake
         * pulse us, exposed fraction, roll angle, servo2 us, servo3 us, roll angles, airbrake
         * fractions, controller outputs, active controllers.
         *
         * @param requestedDeployment requested deployment
         * @param selectedAngle selected angle
         * @param airbrakePulseUs airbrake pulse in microseconds
         * @param exposedFraction exposed fraction
         * @param rollAngle roll angle
         * @param servo2Us servo2 in microseconds
         * @param servo3Us servo3 in microseconds
         * @param rollAngles roll angles
         * @param airbrakeFractions airbrake fractions
         * @param controllerOutputs controller outputs
         * @param activeControllers active controllers
         */
        public Output {
            controllerOutputs = java.util.Map.copyOf(controllerOutputs);
            activeControllers = java.util.Set.copyOf(activeControllers);
            rollAngles = java.util.Map.copyOf(rollAngles);
            airbrakeFractions = java.util.Map.copyOf(airbrakeFractions);
        }

        /**
         * Creates a Output value from the supplied requested, selected angle, airbrake pulse us,
         * exposed fraction, roll angle, servo2 us, servo3 us, roll angles, airbrake fractions.
         *
         * @param requested requested
         * @param selectedAngle selected angle
         * @param airbrakePulseUs airbrake pulse in microseconds
         * @param exposedFraction exposed fraction
         * @param rollAngle roll angle
         * @param servo2Us servo2 in microseconds
         * @param servo3Us servo3 in microseconds
         * @param rollAngles roll angles
         * @param airbrakeFractions airbrake fractions
         */
        public Output(
                float requested,
                float selectedAngle,
                int airbrakePulseUs,
                float exposedFraction,
                float rollAngle,
                int servo2Us,
                int servo3Us,
                java.util.Map<String, Float> rollAngles,
                java.util.Map<String, Float> airbrakeFractions) {
            this(
                    requested,
                    selectedAngle,
                    airbrakePulseUs,
                    exposedFraction,
                    rollAngle,
                    servo2Us,
                    servo3Us,
                    rollAngles,
                    airbrakeFractions,
                    java.util.Map.of(),
                    java.util.Set.of());
        }

        /**
         * Creates a Output value from the supplied requested, selected angle, airbrake pulse us,
         * exposed fraction, roll angle, servo2 us, servo3 us.
         *
         * @param requested requested
         * @param selectedAngle selected angle
         * @param airbrakePulseUs airbrake pulse in microseconds
         * @param exposedFraction exposed fraction
         * @param rollAngle roll angle
         * @param servo2Us servo2 in microseconds
         * @param servo3Us servo3 in microseconds
         */
        public Output(
                float requested,
                float selectedAngle,
                int airbrakePulseUs,
                float exposedFraction,
                float rollAngle,
                int servo2Us,
                int servo3Us) {
            this(
                    requested,
                    selectedAngle,
                    airbrakePulseUs,
                    exposedFraction,
                    rollAngle,
                    servo2Us,
                    servo3Us,
                    java.util.Map.of(),
                    java.util.Map.of());
        }
    }

    /** Creates a RTFC using the supplied settings and defaults for omitted options. */
    public RTFC() {
        this(new Trace());
    }

    /**
     * Creates a RTFC using the supplied settings and defaults for omitted options.
     *
     * @param trace run-local virtual clock and action-log sink
     */
    public RTFC(Trace trace) {
        this(trace, TelemetryLinkSettings.DEFAULT);
    }

    /**
     * Creates a RTFC with the supplied dependencies and per-instance state.
     *
     * @param trace run-local virtual clock and action-log sink
     * @param link link
     */
    public RTFC(Trace trace, TelemetryLinkSettings link) {
        this.trace = Objects.requireNonNull(trace);
        accel = new RTAccel(trace);
        baro = new RTBaro(trace);
        gps = new RTGPS(trace);
        gyro = new RTGyro(trace);
        rollController = new RTRollController(trace);
        airbrakesController = new RTAirbrakesController(trace);
        pyroController = new RTPyroController(trace);
        powerBoard = new RTPowerBoard(trace);
        flashDriver = new RTFlashDriver(trace);
        telemetry = new RTTelemetryEngine(trace, link);
        cam0 = new RTCamera(trace);
        cam1 = new RTCamera(trace);
        vtx = new RTVTX(trace);
    }

    /** FC.ino setup; fresh objects reproduce the firmware globals' startup initialization. */
    public void init() {
        if (initialized) throw new IllegalStateException("Create a fresh RTFC for a new boot");
        initialized = true;
        trace.log(
                "fc.setup",
                "source=FC.ino sketch=4cd2660 library=1db1f22 airbrake_time=integer_seconds pyro=mapped_in_simulation");
        if (getNextBoardDeadlineUs() != Long.MAX_VALUE)
            trace.log("java.board_model", JavaBoardProgram.MODEL_NOTICE);
        pyroController.setup();
        latchPwm();
        telemetry.setup();
        accel.setup();
        baro.setup();
        gyro.setup();
        flashDriver.setup();
        cam0.setup();
        cam1.setup();
        vtx.setup();
        gps.setup();
        powerBoard.setup();
        airbrakesController.setup();
        rollController.setup();
        if (stateMachine != null) stateMachine.start(trace.bootUs());
    }

    /**
     * Snapshots a sensor acquisition for one iteration and advances the boot clock. Rejects future
     * samples, duplicate ticks and overlapping iterations.
     *
     * @param bootUs boot in microseconds
     * @param input input snapshot for this operation
     * @throws IllegalStateException when rejected: Call init first
     * @throws IllegalArgumentException when rejected: Duplicate/backwards FC tick
     */
    public void pre_loop(long bootUs, Inputs input) {
        if (!initialized) throw new IllegalStateException("Call init first");
        if (bootUs <= lastLoopUs) throw new IllegalArgumentException("Duplicate/backwards FC tick");
        if (input.acquisitionUs() > bootUs)
            throw new IllegalArgumentException("Cannot deliver a future sample");
        if (prepared || loopPhase != 0)
            throw new IllegalStateException("Previous FC iteration still active");
        trace.time(bootUs);
        pendingInput = sensors == null ? input : sensors.sample(bootUs, input);
        prepared = true;
    }

    /** Immediate execution remains available for firmware semantic/replay tests. */
    public void loop() {
        beginLoop();
        readSensors();
        completeLoop();
    }

    /** The existing listener schedules these three phases in virtual time. */
    public void beginLoop() {
        if (!prepared) throw new IllegalStateException("Call pre_loop once per tick");
        prepared = false;
        loopPhase = 1;
        lastLoopUs = trace.bootUs();
        loopCount++;
        trace.log("fc.loop_begin", "count=" + loopCount + " state=" + currentState);
    }

    /**
     * Delivers the pending acquisition, updates instruments and measured controller channels, then
     * advances the loop to its completion phase.
     *
     * @throws IllegalStateException when rejected: FC sensors out of order
     */
    public void readSensors() {
        if (loopPhase != 1) throw new IllegalStateException("FC sensors out of order");
        Inputs input = pendingInput;
        if (input.accel() != null) accel.backdoorFudge(input.accel());
        if (input.baro() != null) baro.backdoorFudge(input.baro());
        if (input.gps() != null) { gps.backdoorFudge(input.gps()); lastGpsAcquisitionUs = input.gpsAcquisitionUs(); }
        if (input.gyro() != null) gyro.backdoorFudge(input.gyro());
        trace.log(
                "sensors.deliver",
                "acquisition_us="
                        + input.acquisitionUs()
                        + " age_us="
                        + (trace.bootUs() - input.acquisitionUs())
                        + " accel_new="
                        + (input.accel() != null)
                        + " gps_new="
                        + (input.gps() != null));
        baro.updateAll();
        accel.update(currentState);
        gyro.update();
        gps.updateAndParse();
        pyroController.pyroMonitor(currentState);
        powerBoard.update();
        if (controllerSignals != null)
            controllerSignals.publish(this, sensors.acquired(), trace.bootUs());
        lastSensorAcquisitionUs = input.acquisitionUs();
        loopPhase = 2;
    }

    /**
     * Completes state handling and controller work, schedules firmware telemetry/power messages,
     * consumes commands and releases the pending input.
     *
     * @throws IllegalStateException when rejected: FC completion out of order
     */
    public void completeLoop() {
        if (loopPhase != 2) throw new IllegalStateException("FC completion out of order");
        FCtime = trace.millis();
        if (stateMachine == null) handleState();
        else {
            if (currentState == FLIGHT
                    && elapsed32(FCtime, flightBeginTime) > setting("apogeeLockoutMs", 26000)
                    && !baroMaxAltReset) {
                baro.resetMaxAlt();
                baroMaxAltReset = true;
            }
            if (currentState == APOGEE
                    && design.stateMachine().getBoolean("automaticRecovery", true))
                delayedRecovery();
            stateMachine.step(trace.bootUs(), this::signal, setting("apogeeLockoutMs", 26000));
        }
        if (rollControlEnabled) updateRollControl();
        if (airbrakesEnabled) updateAirbrakes();
        if (elapsed32(trace.millis(), lastTelem) > setting("telemetryIntervalMs", 50)) {
            lastTelem = trace.millis();
            byte[] packet = constructTelemetryPacket();
            telemetry.send(packet);
            if (loggingEnabled) {
                if (design != null && design.telemetryCodec() != null) flashDriver.appendCustom(packet);
                else flashDriver.append(packet);
            }
        }
        readTelem();
        // FC.ino advances the timer before sending; retain its strict >100 ms comparison.
        if (elapsed32(trace.millis(), lastPowerPkt) > setting("powerIntervalMs", 100)) {
            lastPowerPkt = trace.millis();
            powerBoard.sendCommand();
        }
        telemetry.deliverDue(trace.bootUs());
        loopPhase = 0;
        pendingInput = null;
        trace.log(
                "fc.loop_end",
                "state="
                        + currentState
                        + " flight_ms="
                        + elapsed32(FCtime, flightBeginTime)
                        + " airbrakes_enabled="
                        + airbrakesEnabled
                        + " roll_enabled="
                        + rollControlEnabled);
    }

    /** FC.ino handleState; repeated preflight is deliberately not a complete reset. */
    private void handleState() {
        RTRocketState previous = currentState;
        trace.log("fc.handle_state", "state=" + currentState + " received=" + recState);
        switch (currentState) {
            case GROUND_TESTING:
                if (rule("preflight", recState == PRE_FLIGHT)) {
                    currentState = PRE_FLIGHT;
                    prepareFlight();
                }
                break;
            case PRE_FLIGHT:
                if (rule(
                        "launch",
                        recState == FLIGHT || accel.getVerticalAccelMinusGravity() > 30)) {
                    currentState = FLIGHT;
                    beginFlight();
                }
                break;
            case FLIGHT:
                if (elapsed32(FCtime, flightBeginTime) > setting("apogeeLockoutMs", 26000)) {
                    if (!baroMaxAltReset) {
                        baro.resetMaxAlt();
                        baroMaxAltReset = true;
                    }
                    if (rule(
                            "apogee",
                            recState == APOGEE
                                    || (gps.getFixType() == 3
                                            && gps.getMaxAlt() > gps.getHeight() + 20)
                                    || baro.getMaxAlt() > baro.getFilteredAltitude() + 20))
                        currentState = APOGEE;
                }
                if (rule("apogeeOverride", elapsed32(FCtime, flightBeginTime) > 35000))
                    currentState = APOGEE;
                if (currentState == APOGEE) detectApogee();
                break;
            case APOGEE:
                delayedRecovery();
                if (rule(
                        "main",
                        elapsed32(FCtime, apogeeTime) > 55000
                                && (recState == MAIN
                                        || (gps.getFixType() == 3 && gps.getHeight() < 457)
                                        || baro.getFilteredAltitude() < 457))) {
                    currentState = MAIN;
                    deployMain();
                }
                break;
            case MAIN:
                if (rule("end", recState == END)) {
                    currentState = GROUND_TESTING;
                    finishFlight();
                }
                break;
            default:
                break;
        }
        if (previous != currentState)
            trace.log(
                    "fc.transition",
                    "from="
                            + previous
                            + " to="
                            + currentState
                            + " flight_begin_ms="
                            + flightBeginTime
                            + " apogee_ms="
                            + apogeeTime);
    }

    /**
     * Zeros measured flight references, allocates logging, closes controllers and enables the
     * modeled power rails.
     */
    private void prepareFlight() {
        flashDriver.allocate();
        loggingEnabled = true;
        gyro.update();
        gyro.zeroRollPitchYaw();
        accel.update(currentState);
        accel.zeroIntegratedVelo();
        gps.zeroAlt();
        baro.zeroAlt();
        if (controllerSignals != null) controllerSignals.calibrate(this, trace.bootUs(), false);
        closeControllers();
        powerBoard.enableAll();
        trace.log("logging.enable", "enabled=true");
    }

    /**
     * Captures launch time, resets gyro and PID history, updates controllers and enables flight
     * control outputs.
     */
    private void beginFlight() {
        flightBeginTime = trace.millis();
        FCtime = trace.millis();
        gyro.zeroRollPitchYaw();
        if (controllerSignals != null) controllerSignals.calibrate(this, trace.bootUs(), true);
        airbrakesController.resetPidHistory();
        rollController.resetPidHistory();
        updateAirbrakes();
        updateRollControl();
        airbrakesEnabled = rollControlEnabled = true;
        vtx.setPower(3);
        trace.log("controllers.enable", "airbrakes=true roll=true");
    }

    /** Captures apogee time, fires the initial recovery channels and closes flight controllers. */
    private void detectApogee() {
        apogeeTime = trace.millis();
        FCtime = trace.millis();
        fireChannels(0, 1);
        closeControllers();
    }

    /**
     * Issues each delayed apogee recovery action once, using the firmware's strict elapsed-time
     * comparisons.
     */
    private void delayedRecovery() {
        if (elapsed32(FCtime, apogeeTime) > setting("recovery1Ms", 3000) && !bpFired1) {
            fireChannels(3, 4);
            bpFired1 = true;
        }
        if (elapsed32(FCtime, apogeeTime) > setting("recovery2Ms", 5000) && !bpFired2) {
            fireChannels(5);
            bpFired2 = true;
        }
    }

    /** Fires recovery channel 2, disables the roll PWM signal and switches off converter rail 4. */
    private void deployMain() {
        fireChannels(2);
        rollSignalEnabled = false;
        powerBoard.setConverter(4, false);
        trace.log("roll.disable_signal", "servo2=false servo3=false");
    }

    /** Disables firmware flash logging at the end of the flight sequence. */
    private void finishFlight() {
        loggingEnabled = false;
        trace.log("logging.enable", "enabled=false");
    }

    /**
     * Disables the legacy controllers, neutralizes commands and invalidates pending
     * controller-block work.
     */
    private void closeControllers() {
        if (controllers != null) controllers.close();
        namedRoll.clear();
        namedAirbrakes.clear();
        rollControlSetAngle = 0;
        airbrakesSetAngle = CLOSED_ANGLE;
        airbrakesEnabled = rollControlEnabled = false;
        trace.log("controllers.close", "airbrakes_angle=" + airbrakesSetAngle + " roll_angle=0");
    }

    /**
     * Arms and fires each requested logical recovery channel through the pyro controller.
     *
     * @param channels channels
     */
    private void fireChannels(int... channels) {
        if (!connected("recovery")) {
            trace.log("output.discard", "port=recovery reason=no_component");
            return;
        }
        for (int c : channels) {
            pyroController.armPyro(c);
            pyroController.firePyro(c, recoveryOrigin);
        }
    }

    /**
     * Updates the airbrake predictor with measured barometric altitude, integrated velocity,
     * sensor-Z acceleration and integer flight seconds.
     */
    private void updateAirbrakes() {
        airbrakeUpdateCount++;
        float t = elapsed32(FCtime, flightBeginTime) / 1000; // Preserve the FC's integer division.
        airbrakesController.update(
                t,
                new RTFudgedAirbrakesData(
                        baro.getFilteredAltitude(),
                        accel.getIntegratedVelo(),
                        accel.getAccelZ(),
                        currentState.ID > FLIGHT.ID));
    }

    /**
     * Updates the legacy roll controller from measured angular rate, altitude and integrated
     * velocity.
     */
    private void updateRollControl() {
        rollUpdateCount++;
        rollController.update(
                (float) (elapsed32(FCtime, flightBeginTime) / 1000.0),
                baro.getFilteredAltitude(),
                accel.getIntegratedVelo(),
                gyro.getRoll(),
                gyro.getRollRate());
    }

    /**
     * Converts normalized airbrake deployment to the firmware servo angle in degrees.
     *
     * @param dp dp
     * @return computed numeric value
     */
    public static float dpToDeg(float dp) {
        return (OPEN_ANGLE - CLOSED_ANGLE) * dp + CLOSED_ANGLE;
    }

    /**
     * Converts an airbrake servo angle in degrees to its firmware PWM pulse width in microseconds.
     *
     * @param degrees degrees
     * @return computed integer value
     */
    public static int degToUsAirbrakes(float degrees) {
        return ((int) (1500.0 + (degrees / 60.0) * 500.0)) & 0xffff;
    }

    /**
     * Converts a roll servo angle in degrees to its firmware PWM pulse width in microseconds.
     *
     * @param degrees degrees
     * @return computed integer value
     */
    public static int degToUsRollControl(float degrees) {
        return ((int) (1500.0 + (degrees / 50.0) * 500.0)) & 0xffff;
    }

    /**
     * FC Update_IT_callback; source integer PWM with an ideal, endpoint-calibrated linear linkage.
     */
    public void latchPwm() {
        float requested = airbrakesController.getDeployment();
        if (design != null) requested = (float) Math.min(requested, setting("airbrakeLimit", 1));
        float angle = airbrakesEnabled ? dpToDeg(requested) : airbrakesSetAngle;
        float roll = rollControlEnabled ? rollController.getAngle() : rollControlSetAngle;
        if (!Float.isFinite(angle) || !Float.isFinite(roll)) {
            trace.log("fc.error", "reason=nonfinite_actuator");
            throw new IllegalStateException(
                    "Nonfinite FC actuator output at boot_us=" + trace.bootUs());
        }
        if (!connected("airbrakes")) {
            trace.log(
                    "output.discard",
                    "port=airbrakes reason=no_component requested="
                            + requested
                            + " angle="
                            + angle);
            requested = 0;
            angle = CLOSED_ANGLE;
        }
        if (!connected("roll")) {
            trace.log("output.discard", "port=roll reason=no_component angle=" + roll);
            roll = 0;
        }
        int pulse = degToUsAirbrakes(angle);
        float fraction =
                (float) (degToUsAirbrakes(CLOSED_ANGLE) - pulse)
                        / (degToUsAirbrakes(CLOSED_ANGLE) - degToUsAirbrakes(OPEN_ANGLE));
        fraction = Math.max(0, Math.min(1, fraction));
        var latchedAirbrakes = new java.util.LinkedHashMap<String, Float>();
        namedAirbrakes.forEach(
                (name, deployment) -> {
                    int namedPulse = degToUsAirbrakes(dpToDeg(deployment));
                    float exposed =
                            (float) (degToUsAirbrakes(CLOSED_ANGLE) - namedPulse)
                                    / (degToUsAirbrakes(CLOSED_ANGLE)
                                            - degToUsAirbrakes(OPEN_ANGLE));
                    latchedAirbrakes.put(name, Math.max(0, Math.min(1, exposed)));
                });
        output =
                new Output(
                        requested,
                        angle,
                        connected("airbrakes") ? pulse : 0,
                        fraction,
                        roll,
                        rollSignalEnabled && connected("roll")
                                ? degToUsRollControl(roll - 0.5f)
                                : 0,
                        rollSignalEnabled && connected("roll")
                                ? degToUsRollControl(roll - 7.0f)
                                : 0,
                        namedRoll,
                        latchedAirbrakes,
                        controllers == null ? java.util.Map.of() : controllers.outputs(),
                        controllers == null ? java.util.Set.of() : controllers.activeIds());
        trace.log(
                "pwm.latch",
                "requested="
                        + requested
                        + " enabled="
                        + airbrakesEnabled
                        + " angle_deg="
                        + angle
                        + " pulse_us="
                        + pulse
                        + " exposed="
                        + fraction
                        + " roll_deg="
                        + roll
                        + " servo2_us="
                        + output.servo2Us()
                        + " servo3_us="
                        + output.servo3Us());
    }

    /**
     * Returns the last PWM-latched output snapshot; newly computed commands wait for the next
     * latch.
     *
     * @return the last PWM-latched output snapshot; newly computed commands wait for the next latch
     */
    public Output getOutput() {
        return output;
    }

    /**
     * Returns the firmware phase associated with the current design state.
     *
     * @return the firmware phase associated with the current design state
     */
    public RTRocketState getState() {
        return currentState;
    }

    /**
     * Returns the captured launch time on the wrapping 32-bit millisecond clock.
     *
     * @return the captured launch time on the wrapping 32-bit millisecond clock
     */
    public long getFlightBeginTime() {
        return flightBeginTime;
    }

    /**
     * Returns the captured apogee time on the wrapping 32-bit millisecond clock.
     *
     * @return the captured apogee time on the wrapping 32-bit millisecond clock
     */
    public long getApogeeTime() {
        return apogeeTime;
    }

    /**
     * Returns the number of started firmware iterations in this run.
     *
     * @return the number of started firmware iterations in this run
     */
    public long getLoopCount() {
        return loopCount;
    }

    /**
     * Returns the number of legacy airbrake controller updates in this run.
     *
     * @return the number of legacy airbrake controller updates in this run
     */
    public long getAirbrakeUpdateCount() {
        return airbrakeUpdateCount;
    }

    /**
     * Returns the number of legacy roll controller updates in this run.
     *
     * @return the number of legacy roll controller updates in this run
     */
    public long getRollUpdateCount() {
        return rollUpdateCount;
    }

    /**
     * Reports whether automatic legacy airbrake updates are enabled.
     *
     * @return true when the condition described above holds
     */
    public boolean isAirbrakesEnabled() {
        return airbrakesEnabled;
    }

    /**
     * Returns the number of rejected received command packets.
     *
     * @return the number of rejected received command packets
     */
    public int getBadPackets() {
        return badPackets;
    }

    /**
     * Queues command bytes for processing by the firmware receive path.
     *
     * @param packet firmware-format packet bytes
     */
    public void enqueueCommand(byte[] packet) {
        telemetry.enqueue(packet);
    }

    /** Test/scenario convenience: still traverses the FC's real byte decoder and loop ordering. */
    public static byte[] stateCommand(RTRocketState state) {
        return command(2, state.ID);
    }

    /**
     * Builds a firmware state-command packet with its checksum.
     *
     * @param opcode opcode
     * @param value replacement or input value
     * @return the resulting byte[]
     */
    public static byte[] command(int opcode, int value) {
        byte[] p = new byte[16];
        p[0] = (byte) 0xaa;
        p[12] = (byte) value;
        p[13] = (byte) opcode;
        checksumCommand(p);
        return p;
    }

    /**
     * Builds a firmware manual-angle command packet with its checksum.
     *
     * @param opcode opcode
     * @param value replacement or input value
     * @return the resulting byte[]
     */
    public static byte[] angleCommand(int opcode, float value) {
        byte[] p = command(opcode, 0);
        ByteBuffer.wrap(p).order(ByteOrder.LITTLE_ENDIAN).putFloat(9, value);
        checksumCommand(p);
        return p;
    }

    /**
     * Fills the command packet checksum using the firmware byte convention.
     *
     * @param p p
     */
    public static void checksumCommand(byte[] p) {
        int sum = 0;
        for (int i = 1; i < 14; i++) sum += p[i] & 255;
        p[14] = (byte) (sum >> 8);
        p[15] = (byte) sum;
    }

    /** FC readTelem: process one received packet at the end of a loop. */
    private void readTelem() {
        byte[] p = telemetry.receive();
        if (p == null) return;
        if (p.length != 16 || (p[0] & 255) != 0xaa) {
            reject("length_or_header");
            return;
        }
        int sum = 0;
        for (int i = 1; i < 14; i++) sum += p[i] & 255;
        if (sum != ((p[14] & 255) << 8) + (p[15] & 255)) {
            reject("checksum");
            return;
        }
        int op = p[13] & 255, value = p[12] & 255;
        int lastReserved =
                (op == 3 || op == 5)
                        ? 8
                        : ((op == 1 || op == 2 || op == 6 || op == 0x14 || op == 0x15 || op == 0x16)
                                ? 11
                                : 12);
        for (int i = 1; i <= lastReserved; i++)
            if (p[i] != 0) {
                trace.log("command.ignore", "reason=reserved_bytes opcode=" + op);
                return;
            }
        boolean ground = currentState == GROUND_TESTING;
        if ((op == 1 || op == 6) && (value & 0xc0) != 0) {
            reject("pyro_mask_out_of_range");
            return;
        }
        if (op == 2 && value >= RTRocketState.values().length) {
            reject("invalid_state");
            return;
        }
        trace.log("command.decode", "opcode=" + op + " value=" + value + " ground=" + ground);
        if ((op == 1 || op == 6) && !connected("recovery")) {
            trace.log("output.discard", "port=recovery reason=no_component opcode=" + op);
            return;
        }
        String previousOrigin = recoveryOrigin;
        recoveryOrigin = "radio";
        try {
            switch (op) {
                case 1:
                    for (int i = 0; i < 6; i++)
                        if ((value & (1 << i)) != 0) pyroController.armPyro(i);
                    break;
                case 2:
                    recState = RTRocketState.values()[value];
                    trace.log("command.state", "received=" + recState);
                    break;
                case 6:
                    for (int i = 0; i < 6; i++)
                        if ((value & (1 << i)) != 0) pyroController.firePyro(i, "radio");
                    break;
                case 3:
                case 5:
                    if (ground) {
                        float a = ByteBuffer.wrap(p).order(ByteOrder.LITTLE_ENDIAN).getFloat(9);
                        if (!Float.isFinite(a) || a < -180 || a > 180) {
                            reject("invalid_manual_angle");
                            break;
                        }
                        if (op == 3) {
                            airbrakesSetAngle = a;
                            airbrakesEnabled = false;
                        } else {
                            rollControlSetAngle = a;
                            rollControlEnabled = false;
                        }
                        trace.log("command.manual_servo", "opcode=" + op + " degrees=" + a);
                    }
                    break;
                case 4:
                    if (ground) {
                        updateRollControl();
                        rollControlEnabled = true;
                    }
                    break;
                case 8:
                    if (ground) closeControllers();
                    break;
                case 9:
                    if (ground) gyro.zeroRollPitchYaw();
                    break;
                case 10:
                    if (ground) {
                        baro.zeroAlt();
                        gps.zeroAlt();
                    }
                    break;
                case 11:
                    if (ground) accel.zeroIntegratedVelo();
                    break;
                case 0x10:
                    fireChannels(0, 1);
                    break;
                case 0x11:
                    fireChannels(3, 4);
                    break;
                case 0x12:
                    fireChannels(2);
                    break;
                case 0x13:
                    fireChannels(0, 1, 2, 3, 4);
                    break; // Source excludes channel 5 here.
                case 0x14:
                    if (ground) vtx.setPower(Math.min(3, value));
                    break;
                case 0x15:
                    if (ground)
                        for (int i = 0; i < 6; i++)
                            powerBoard.setConverter(i, (value & (1 << i)) != 0);
                    break;
                case 0x16:
                    if (ground) powerBoard.setProtections(value != 0);
                    break;
                default:
                    trace.log("command.ignore", "reason=unknown_opcode opcode=" + op);
                    break;
            }
        } finally {
            recoveryOrigin = previousOrigin;
        }
        if (!ground && ((op >= 3 && op <= 11 && op != 6) || op >= 0x14))
            trace.log("command.ignore", "reason=requires_ground opcode=" + op);
    }

    /**
     * Counts and traces a malformed or unsupported received command.
     *
     * @param reason reason
     */
    private void reject(String reason) {
        badPackets = (badPackets + 1) & 255;
        trace.log("command.reject", "reason=" + reason + " bad_packets=" + badPackets);
    }

    /**
     * Stores the low 24 bits of a value in the telemetry byte order.
     *
     * @param p p
     * @param offset offset
     * @param value replacement or input value
     */
    private static void put24(byte[] p, int offset, int value) {
        p[offset] = (byte) value;
        p[offset + 1] = (byte) (value >> 8);
        p[offset + 2] = (byte) (value >> 16);
    }

    /**
     * Bounds a numeric telemetry field to the representable integer range.
     *
     * @param value replacement or input value
     * @param low lower allowed bound
     * @param high upper allowed bound
     * @return computed integer value
     */
    private static int clampCount(double value, int low, int high) {
        return (int) Math.max(low, Math.min(high, Math.round(value)));
    }

    private java.util.Set<String> missionMissingFields = java.util.Set.of();
    public java.util.Set<String> getMissionMissingFields() { return missionMissingFields; }
    private long lastGpsAcquisitionUs;
    public long getLastGpsAcquisitionUs() { return lastGpsAcquisitionUs; }
    public long getLastSensorAcquisitionUs() { return lastSensorAcquisitionUs; }

    /** Declarative codec inputs are held measured device values, never physics truth. */
    public java.util.Map<String, Double> telemetryMeasurements() {
        var values = new java.util.LinkedHashMap<String, Double>();
        values.put("uptime_ms", (double) trace.millis()); values.put("packet_sequence", (double) packetNum);
        values.put("state.code", (double) currentState.ID);
        if (connected("gps")) {
            values.put("gps.fix", (double) gps.getFixType()); values.put("gps.latitude_deg", gps.getLatitude());
            values.put("gps.longitude_deg", gps.getLongitude()); values.put("gps.height_m", (double) gps.getHeight());
        }
        if (connected("barometer")) values.put("barometer.height_m", (double) baro.getFilteredAltitude());
        if (connected("accelerometer")) {
            values.put("velocity.up_ms", (double) accel.getIntegratedVelo());
            values.put("acceleration.x_ms2", (double) accel.getAccelX()); values.put("acceleration.y_ms2", (double) accel.getAccelY());
            values.put("acceleration.z_ms2", (double) accel.getAccelZ());
        }
        if (connected("gyro")) {
            values.put("gyro.roll_rate_deg_s", (double) gyro.getGyroX()); values.put("gyro.pitch_rate_deg_s", (double) gyro.getGyroY());
            values.put("gyro.yaw_rate_deg_s", (double) gyro.getGyroZ());
        }
        if (connected("power")) values.put("battery.voltage_v", (double) (powerBoard.getCellVoltage(0) + powerBoard.getCellVoltage(1) + powerBoard.getCellVoltage(2)));
        return java.util.Collections.unmodifiableMap(values);
    }

    /** FC constructTelemetryPacket: raw counts synthesized at the declared engineering boundary. */
    private byte[] constructTelemetryPacket() {
        packetNum = (packetNum + 1) & 0xffff;
        if (design != null && design.telemetryCodec() != null) {
            var encoded = design.telemetryCodec().encode(telemetryMeasurements());
            missionMissingFields = encoded.unavailable();
            return encoded.bytes();
        }
        byte[] p = new byte[128];
        ByteBuffer out = ByteBuffer.wrap(p).order(ByteOrder.LITTLE_ENDIAN);
        out.putShort(0, (short) pyroController.getPyrosStatus());
        for (int i = 0; i < 6; i++) {
            if (pyroController.isArmed(i)) p[2] |= 1 << i;
            if (pyroController.isFired(i)) p[3] |= 1 << i;
        }
        int s0 = output.airbrakePulseUs(),
                s1 = 1500,
                s2 = output.servo2Us(),
                s3 = output.servo3Us();
        p[10] = (byte) s0;
        p[11] = (byte) ((s0 >> 8) | ((s1 & 15) << 4));
        p[12] = (byte) (s1 >> 4);
        p[13] = (byte) s2;
        p[14] = (byte) ((s2 >> 8) | ((s3 & 15) << 4));
        p[15] = (byte) (s3 >> 4);
        float[] av = {accel.getAccelX(), accel.getAccelY(), accel.getAccelZ()};
        for (int i = 0; i < 3; i++)
            put24(
                    p,
                    16 + 3 * i,
                    clampCount(
                            av[i] / (9.80665 * (i == 0 ? 1.060 : 1.0)) * 12800, -524288, 524287));
        double[] gv = {
            (gyro.getGyroX() + 0.2525f) / 0.03051757812,
            (gyro.getGyroY() + 0.2441f) / 0.03051757812,
            (gyro.getGyroZ() + 0.4376f) / 0.03051757812
        };
        for (int i = 0; i < 3; i++)
            out.putShort(25 + 2 * i, (short) clampCount(gv[i], -32768, 32767));
        int rawT =
                clampCount(
                        (baro.getTemperature() * 100.0 - 2000) * (1 << 23) / 0x6D91
                                + 0x8405 * 256.0,
                        0,
                        0xffffff);
        float dT = rawT - (float) 0x8405 * 256;
        long off = (long) (((long) 0x953A) * (1L << 17) + dT * (float) 0x6305 / (1 << 6));
        long sens = (long) ((float) 0xA579 * (1L << 16) + dT * (float) 0x68AC / (1 << 7));
        int rawP =
                clampCount(
                        (baro.getPressure() * 100.0 + off / (double) (1 << 15))
                                * (1 << 15)
                                / (sens / (double) (1 << 21)),
                        0,
                        0xffffff);
        put24(p, 53, rawP);
        put24(p, 56, rawT);
        p[31] = (byte) gps.getFixType();
        out.putInt(32, gps.getLatE7());
        out.putInt(36, gps.getLonE7());
        out.putFloat(40, gps.getHeight());
        out.putFloat(59, baro.getFilteredAltitude());
        p[63] = (byte) currentState.ID;
        out.putFloat(64, gyro.getRoll());
        out.putFloat(68, gyro.getPitch());
        out.putFloat(72, gyro.getYaw());
        out.putShort(76, (short) baro.getMaxAlt());
        out.putShort(78, (short) gps.getMaxAlt());
        out.putInt(80, (int) FCtime);
        out.putShort(84, (short) packetNum);
        for (int i = 0; i < 3; i++)
            out.putShort(87 + 2 * i, (short) (powerBoard.getCellVoltage(i) * 1000));
        p[95] = 40;
        p[97] = (byte) (powerBoard.protectionsEnabled() ? 1 : 0);
        for (int i = 0; i < 6; i++)
            out.putShort(98 + 2 * i, (short) Math.round(powerBoard.getVoltageByRail(i) / 0.0016));
        out.putFloat(122, accel.getIntegratedVelo());
        for (int i = 0; i < 127; i++) p[127] += p[i];
        trace.log(
                "telemetry.construct",
                "packet=" + packetNum + " raw_sensors=synthesized unmodeled_fields=zero");
        return p;
    }
}
