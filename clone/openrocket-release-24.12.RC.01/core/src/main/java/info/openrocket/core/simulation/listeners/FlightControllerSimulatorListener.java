package info.openrocket.core.simulation.listeners;

import edu.mit.rocket_team.FC.RTFC;
import edu.mit.rocket_team.FC.FlightComputerTimingSettings;
import edu.mit.rocket_team.telemetry.TelemetryLinkSettings;
import edu.mit.rocket_team.telemetry.FlightComputerOutputSettings;
import edu.mit.rocket_team.util.RTSimulationCommunicator;
import edu.mit.rocket_team.util.RTUtilLibrary.Trace;
import edu.mit.rocket_team.util.data.*;
import info.openrocket.core.models.atmosphere.AtmosphericConditions;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.exception.SimulationException;
import info.openrocket.core.util.*;
import java.util.Iterator;
import java.util.List;
import java.nio.file.Path;
import java.util.function.Consumer;
import static edu.mit.rocket_team.util.RTUtilLibrary.convertToImuAngles;
import static java.lang.Math.*;
import info.openrocket.core.simulation.flightcomputer.*;

/** Existing FC listener: virtual execution phases and an independent PWM timer. */
public class FlightControllerSimulatorListener extends AbstractSimulationListener {
    // Compatibility-only fields for historical callers. Not used by the FC path below.
    public static SimulationStatus initialStatus = null;
    public static SimulationStatus latestStatus = null;
    public static double latestTimeStep = -1;

    public static TabControlledTrapezoidFinSet theFinsToModify = null;

    public static double overrideCNA = 0;
    public static double TIME_DELAY_MOTOR = 0;

    public static ArrayList<Double> pastOmegaZ;
    public static ArrayList<Double> pastThetaZ;
    public static ArrayList<Double> finTabAngleLog;
    public static ArrayList<Double> rktVelMagLog;
    public static ArrayList<Double> rktAltLog;
    public static ArrayList<Double> CldLog;
    public static ArrayList<Double> Qlog;
    public static ArrayList<Double> CldArefDLog;

    public static SimulationStatus lastStat = null;

    // Randomness controls
    public static ArrayList<Double> altitudeMeasuredList = null;
    public static double amplitude_randomness_size = 5;
    public static boolean ABORT_AT_APOGEE = false;

    // Simulation loop timing
    public static double loopStart = 0;

    // Simulation of servo lag
    public static double servoStepCount = 4097.0; // number of discrete steps the servo can make
    public static final double servoRangeAngleDeg = 120.0; // total range of motion of the servo
    public static double SERVO_REFRESH_TIME = 2e-10; // seconds
    public static double lastServoCommandTimestamp = 0;
    public static boolean flagPrintDebugMsg = false;
    public static boolean roundToNearest5 = true;

    private static final long WARMUP_US = 1_000_000;
    private final Consumer<String> console;
    private final double maxPhysicsStep;
    private final boolean holdAirbrakesClosed;
    private final TelemetryLinkSettings linkSettings;
    private final FlightComputerOutputSettings outputSettings;
    private final FlightComputerTimingSettings timing;
    private Run run;
    private FlightComputerControllerSettings controllerSettings =
            FlightComputerControllerSettings.EMPTY;

    /**
     * Attaches validated controller-to-rocket assignments to this listener before the run starts.
     *
     * @param settings validated configuration for this operation
     * @return the resulting FlightControllerSimulatorListener
     */
    public FlightControllerSimulatorListener withControllers(
            FlightComputerControllerSettings settings) {
        controllerSettings = java.util.Objects.requireNonNull(settings);
        return this;
    }

    private FlightComputerRecoverySettings recoverySettings = FlightComputerRecoverySettings.EMPTY;

    /**
     * Attaches the FC recovery-channel assignments used to override automatic OpenRocket deployment
     * rules.
     *
     * @param settings validated configuration for this operation
     * @return the resulting FlightControllerSimulatorListener
     */
    public FlightControllerSimulatorListener withRecovery(FlightComputerRecoverySettings settings) {
        recoverySettings = java.util.Objects.requireNonNull(settings);
        return this;
    }

    /**
     * Checks whether a queued physical recovery event belongs to an accepted FC command and is
     * still eligible.
     *
     * @param status stat in microseconds
     * @param event event to process
     * @return true when the condition described above holds
     */
    public boolean allowRecoveryEvent(SimulationStatus status, FlightEvent event) {
        return run.recovery.allow(status, event);
    }

    /**
     * Records why a queued FC recovery event was rejected by the engine.
     *
     * @param event event to process
     * @param reason reason
     */
    public void rejectRecoveryEvent(FlightEvent event, String reason) {
        run.recovery.rejected(event, reason);
    }

    /**
     * Acknowledges an actual physical recovery deployment for logging and plot data.
     *
     * @param status stat in microseconds
     * @param event event to process
     */
    public void recoveryDeployed(SimulationStatus status, FlightEvent event) {
        run.recovery.deployed(status, event);
    }

    private info.openrocket.core.simulation.flightcomputer.FlightComputerLibrary.Resolved design;

    /**
     * Attaches the frozen external flight-computer design used by this run.
     *
     * @param value replacement or input value
     * @return the resulting FlightControllerSimulatorListener
     */
    public FlightControllerSimulatorListener withDesign(
            info.openrocket.core.simulation.flightcomputer.FlightComputerLibrary.Resolved value) {
        design = value;
        return this;
    }

    private info.openrocket.core.simulation.mission.MissionFlightHook missionHook;

    public FlightControllerSimulatorListener withMission(info.openrocket.core.simulation.mission.MissionFlightHook hook) {
        missionHook = hook; return this;
    }
    private long nextDeadlineUs() {
        long next = run.nextDeadlineUs();
        if (missionHook == null) return next;
        long mission = missionHook.nextMissionDeadlineUs();
        return Math.min(next, mission == Long.MAX_VALUE ? Long.MAX_VALUE : Math.addExact(mission, WARMUP_US));
    }

    /** Shared only across framework copies belonging to this flight, never across new starts. */
    private static final class Run {
        final RTFC fc;
        final RTSimulationCommunicator communicator;
        FlightComputerRecoveryBridge recovery;
        long nextCpuUs, nextPwmUs, nextGpsUs, lastPublishedUs = -1, lastPlotLogUs = -1000000;
        long loopStartUs, workUs, sampleUs, gpsFixUs = -1, deliveredGpsUs = -1;
        long completedLoops, overruns, maxExecutionUs, lastControlUs = -1, lastControlSampleUs = -1;
        long maxSensorAgeUs, maxPwmSampleAgeUs;
        int phase;
        boolean armed;
        RTGPSData gpsFix;
        final java.util.Random timingRandom;

        /**
         * Returns the earliest pending virtual task, PWM, telemetry or recovery deadline for this
         * run context.
         *
         * @return the earliest pending virtual task, PWM, telemetry or recovery deadline for this
         *     run context
         */
        long nextDeadlineUs() {
            return Math.min(
                    fc.getNextBoardDeadlineUs(),
                    Math.min(nextCpuUs, Math.min(nextPwmUs, nextGpsUs)));
        }

        double stepStart;
        RTFC.Inputs candidate, latest;
        boolean intervalOpen, closed;
        private Consumer<String> fileLog = line -> {};

        /**
         * Creates a Run value from the supplied console, link, timing.
         *
         * @param console console
         * @param link link
         * @param timing initial next pwm us
         */
        Run(
                Consumer<String> console,
                TelemetryLinkSettings link,
                FlightComputerTimingSettings timing) {
            timingRandom = new java.util.Random(timing.randomSeed());
            nextPwmUs = timing.pwmPhaseUs() == 0 ? RTFC.PWM_US : timing.pwmPhaseUs();
            fc =
                    new RTFC(
                            new Trace(
                                    line -> {
                                        console.accept(line);
                                        fileLog.accept(line);
                                    }),
                            link);
            fileLog = fc.telemetry::logLine;
            communicator = new RTSimulationCommunicator(fc.trace);
        }
    }

    /**
     * Creates a FlightControllerSimulatorListener using the supplied settings and defaults for
     * omitted options.
     */
    public FlightControllerSimulatorListener() {
        this(line -> System.out.println(line), 0.0025, false);
    }

    /**
     * Creates a FlightControllerSimulatorListener using the supplied settings and defaults for
     * omitted options.
     *
     * @param link link
     */
    public FlightControllerSimulatorListener(TelemetryLinkSettings link) {
        this(link, FlightComputerOutputSettings.DEFAULT);
    }

    /**
     * Creates a FlightControllerSimulatorListener using the supplied settings and defaults for
     * omitted options.
     *
     * @param link link
     * @param output output snapshot or destination
     */
    public FlightControllerSimulatorListener(
            TelemetryLinkSettings link, FlightComputerOutputSettings output) {
        this(System.out::println, 0.0025, false, link, output);
    }

    /**
     * Creates a FlightControllerSimulatorListener using the supplied settings and defaults for
     * omitted options.
     *
     * @param link link
     * @param output output snapshot or destination
     * @param timing timing
     */
    public FlightControllerSimulatorListener(
            TelemetryLinkSettings link,
            FlightComputerOutputSettings output,
            FlightComputerTimingSettings timing) {
        this(System.out::println, 0.0025, false, link, output, timing);
    }

    /**
     * Creates a FlightControllerSimulatorListener using the supplied settings and defaults for
     * omitted options.
     *
     * @param console console
     * @param maxPhysicsStep max physics step
     * @param holdAirbrakesClosed hold airbrakes closed
     */
    public FlightControllerSimulatorListener(
            Consumer<String> console, double maxPhysicsStep, boolean holdAirbrakesClosed) {
        this(console, maxPhysicsStep, holdAirbrakesClosed, TelemetryLinkSettings.DEFAULT);
    }

    /**
     * Creates a FlightControllerSimulatorListener using the supplied settings and defaults for
     * omitted options.
     *
     * @param console console
     * @param maxPhysicsStep max physics step
     * @param holdAirbrakesClosed hold airbrakes closed
     * @param link link
     */
    public FlightControllerSimulatorListener(
            Consumer<String> console,
            double maxPhysicsStep,
            boolean holdAirbrakesClosed,
            TelemetryLinkSettings link) {
        this(
                console,
                maxPhysicsStep,
                holdAirbrakesClosed,
                link,
                FlightComputerOutputSettings.DEFAULT);
    }

    /**
     * Creates a FlightControllerSimulatorListener using the supplied settings and defaults for
     * omitted options.
     *
     * @param console console
     * @param maxPhysicsStep max physics step
     * @param holdAirbrakesClosed hold airbrakes closed
     * @param link link
     * @param output output snapshot or destination
     */
    public FlightControllerSimulatorListener(
            Consumer<String> console,
            double maxPhysicsStep,
            boolean holdAirbrakesClosed,
            TelemetryLinkSettings link,
            FlightComputerOutputSettings output) {
        this(
                console,
                maxPhysicsStep,
                holdAirbrakesClosed,
                link,
                output,
                FlightComputerTimingSettings.DEFAULT);
    }

    /**
     * Creates a FlightControllerSimulatorListener after validating the supplied configuration.
     *
     * @param console initial console
     * @param maxPhysicsStep initial max physics step
     * @param holdAirbrakesClosed initial hold airbrakes closed
     * @param link link
     * @param output output snapshot or destination
     * @param timing timing
     * @throws IllegalArgumentException when rejected: FC physics step must be in (0, 0.0025]
     *     seconds
     */
    public FlightControllerSimulatorListener(
            Consumer<String> console,
            double maxPhysicsStep,
            boolean holdAirbrakesClosed,
            TelemetryLinkSettings link,
            FlightComputerOutputSettings output,
            FlightComputerTimingSettings timing) {
        this.timing = java.util.Objects.requireNonNull(timing);
        this.outputSettings = java.util.Objects.requireNonNull(output);
        if (!(maxPhysicsStep > 0 && maxPhysicsStep <= 0.0025))
            throw new IllegalArgumentException("FC physics step must be in (0, 0.0025] seconds");
        this.linkSettings = java.util.Objects.requireNonNull(link);
        this.console = console;
        this.maxPhysicsStep = maxPhysicsStep;
        this.holdAirbrakesClosed = holdAirbrakesClosed;
    }

    public record BenchPoint(
            double seconds,
            double altitude,
            double velocity,
            double output,
            int state,
            long sampleAgeUs) {}

    public record BenchResult(
            java.util.List<BenchPoint> points,
            TimingSummary timing,
            java.nio.file.Path csv,
            java.nio.file.Path log) {}

    /**
     * Bench/replay uses the same dispatcher as flight integration, without rocket truth or physical
     * actuation.
     */
    public static BenchResult bench(
            info.openrocket.core.simulation.flightcomputer.FlightComputerDesign definition,
            long endUs,
            java.util.function.LongFunction<RTFC.Inputs> samples,
            Consumer<String> console) {
        definition.requireRunnable();
        var listener =
                new FlightControllerSimulatorListener(
                        console,
                        .0025,
                        false,
                        TelemetryLinkSettings.DEFAULT,
                        FlightComputerOutputSettings.DEFAULT,
                        definition.timing());
        listener.design =
                new info.openrocket.core.simulation.flightcomputer.FlightComputerLibrary.Resolved(
                        definition,
                        info.openrocket.core.simulation.flightcomputer.FlightComputerLibrary
                                .directory()
                                .resolve("unsaved-bench.fc"),
                        "unsaved");
        listener.run = new Run(console, TelemetryLinkSettings.DEFAULT, definition.timing());
        var fc = listener.run.fc;
        fc.configure(definition);
        var points = new java.util.ArrayList<BenchPoint>();
        try {
            fc.telemetry.open(
                    java.nio.file.Path.of(
                            System.getProperty("openrocket.fc.telemetryDir", "fc-telemetry")),
                    FlightComputerOutputSettings.DEFAULT);
            fc.init();
            long lastCompleted = -1;
            while (listener.run.nextDeadlineUs() <= endUs) {
                if (Thread.currentThread().isInterrupted())
                    throw new java.util.concurrent.CancellationException();
                long now = listener.run.nextDeadlineUs();
                var input = samples.apply(now);
                if (input.acquisitionUs() > now)
                    throw new IllegalArgumentException("Replay contains a future sample");
                listener.dispatch(now, input);
                if (lastCompleted != listener.run.completedLoops) {
                    lastCompleted = listener.run.completedLoops;
                    points.add(
                            new BenchPoint(
                                    now / 1e6,
                                    fc.baro.getFilteredAltitude(),
                                    fc.accel.getIntegratedVelo(),
                                    fc.getOutput().exposedFraction(),
                                    FlightComputerData.stateCode(
                                            definition, fc.getDesignStateId(), fc.getState().ID),
                                    now - input.acquisitionUs()));
                }
            }
            fc.telemetry.finish(true);
            return new BenchResult(
                    java.util.List.copyOf(points),
                    listener.getTimingSummary(),
                    fc.telemetry.getCsvPath(),
                    fc.telemetry.getLogPath());
        } finally {
            fc.telemetry.finish(false);
        }
    }

    /**
     * Returns the per-run flight-computer instance for inspection.
     *
     * @return the per-run flight-computer instance for inspection
     */
    public RTFC getFlightComputer() {
        return run == null ? null : run.fc;
    }

    /**
     * Returns the listener's maximum allowed physics timestep in seconds.
     *
     * @return the listener's maximum allowed physics timestep in seconds
     */
    public double getMaximumPhysicsStep() {
        return maxPhysicsStep;
    }

    /**
     * Creates the isolated FC run, binds physical outputs, opens telemetry and warms up the virtual
     * computer before launch.
     *
     * @param status stat in microseconds
     * @throws SimulationException if the requested operation cannot be completed
     */
    @Override
    public void startSimulation(SimulationStatus status) throws SimulationException {
        run = new Run(missionHook == null ? console : line -> { console.accept(line); missionHook.diagnostic(line); }, linkSettings, timing);
        if (design != null) {
            if (missionHook == null) run.fc.configure(design.design());
            else run.fc.configureMission(design.design());
        }
        if (status.getConfiguration().getActiveStageCount() != 1)
            throw new SimulationException(
                    "The flight computer currently supports one active stage");
        if (status.getSimulationTime() != 0)
            throw new SimulationException(
                    "The flight computer requires pad startup; mid-flight checkpoints are not implemented");
        run.communicator.bind(status, run.fc.connected("airbrakes"), run.fc.connected("roll"));
        run.communicator.bindControllers(
                status, design == null ? null : design.design(), controllerSettings, run.fc);
        status.getSimulationConditions().setTimeStep(maxPhysicsStep);
        var simulation = status.getSimulationConditions().getSimulation();
        run.fc.telemetry.open(
                missionHook == null ? Path.of(System.getProperty("openrocket.fc.telemetryDir", "fc-telemetry")) : missionHook.outputDirectory(),
                missionHook == null ? outputSettings.forRun(simulation == null ? null : simulation.getRunOutputTag()) : FlightComputerOutputSettings.DEFAULT);
        if (status.getSimulationConditions().getSimulation() != null) {
            status.getSimulationConditions()
                    .getSimulation()
                    .setFlightComputerTelemetryPath(run.fc.telemetry.getCsvPath());
            status.getSimulationConditions()
                    .getSimulation()
                    .setFlightComputerLogPath(run.fc.telemetry.getLogPath());
        }
        try {
            run.recovery =
                    new FlightComputerRecoveryBridge(
                            status,
                            recoverySettings,
                            run.fc.pyroController,
                            run.fc.trace,
                            WARMUP_US);
            run.fc.trace.log(
                    "simulation.start",
                    "rocket="
                            + status.getConfiguration().getRocket().getName()
                            + " max_step_s="
                            + maxPhysicsStep
                            + " mounting=X:bodyZ,Y:bodyX,Z:bodyY noise=off recovery=FC_exclusive pyro=mapped_in_simulation roll_physics=tab_fin_sets ideal_roll_linkage=1_to_1 hold_closed="
                            + holdAirbrakesClosed);
            if (design != null)
                run.fc.trace.log(
                        "design.loaded",
                        "path="
                                + design.path()
                                + " id="
                                + design.design().id()
                                + " model="
                                + design.design().model()
                                + " semantic_hash="
                                + design.design().fingerprint()
                                + " file_hash="
                                + design.exactHash());
            var stateNames = jakarta.json.Json.createObjectBuilder();
            FlightComputerData.stateNames(design == null ? null : design.design())
                    .forEach(stateNames::add);
            run.fc.trace.log(
                    "fc.plot_metadata",
                    jakarta.json.Json.createObjectBuilder()
                            .add("version", 1)
                            .add("states", stateNames)
                            .add("actuators", run.communicator.plotMetadata())
                            .add("recovery", run.recovery.metadata())
                            .add(
                                    "controllers",
                                    FlightComputerControllerPlots.metadata(
                                            design == null ? null : design.design()))
                            .build()
                            .toString());
            if (missionHook != null) missionHook.start(run.fc, status);
            run.fc.init();
            run.fc.trace.log(
                    "timing.settings",
                    "sensor_read_us="
                            + timing.sensorReadUs()
                            + " extra_work_us="
                            + timing.extraWorkUs()
                            + " work_jitter_us="
                            + timing.workJitterUs()
                            + " pwm_phase_us="
                            + timing.pwmPhaseUs()
                            + " seed="
                            + timing.randomSeed()
                            + " calibration=unmeasured lumped_execution=true sample=held_at_loop_start pwm_tie=interrupt_first servo_travel=ideal");
            while (nextDeadlineUs() <= WARMUP_US) {
                long us = nextDeadlineUs();
                dispatch(us, sample(status, Coordinate.ZERO, us));
                run.recovery.drain(status);
            }
            run.latest = sample(status, Coordinate.ZERO, WARMUP_US);
            run.communicator.apply(run.fc.getOutput(), holdAirbrakesClosed);
            run.fc.trace.log("simulation.release", "simulation_s=0 boot_us=" + WARMUP_US);
        } catch (RuntimeException e) {
            finish();
            throw e;
        }
    }

    /** Service a real virtual-time boundary; never replay missed loops using future samples. */
    private void dispatch(long bootUs, RTFC.Inputs input) {
        if (missionHook != null) missionHook.before(bootUs);
        run.fc.trace.time(bootUs);
        if (!run.armed && bootUs >= 500_000) {
            run.armed = true;
            run.fc.enqueueCommand(
                    RTFC.stateCommand(edu.mit.rocket_team.util.RTRocketState.PRE_FLIGHT));
        }
        if (bootUs == run.nextGpsUs) {
            run.gpsFix = input.gps();
            run.gpsFixUs = input.acquisitionUs();
            run.nextGpsUs +=
                    design == null
                            ? 100_000
                            : (long)
                                    design.design()
                                            .active("gps")
                                            .getJsonObject("properties")
                                            .getJsonNumber("periodUs")
                                            .doubleValue();
            run.fc.trace.log("gps.fix_available", "acquisition_us=" + run.gpsFixUs);
        }
        // Independent interrupt: on an exact tie, it sees the previous completed controller result.
        if (bootUs == run.nextPwmUs) {
            run.fc.latchPwm();
            run.nextPwmUs += RTFC.PWM_US;
            if (run.lastControlUs >= 0) {
                long age = bootUs - run.lastControlSampleUs;
                run.maxPwmSampleAgeUs = Math.max(run.maxPwmSampleAgeUs, age);
                run.fc.trace.log(
                        "pwm.latency",
                        "sample_age_us=" + age + " command_age_us=" + (bootUs - run.lastControlUs));
            }
        }
        // Zero-cost phases and an overrun's immediate next loop can share this boundary.
        while (bootUs == run.nextCpuUs) {
            switch (run.phase) {
                case 0 -> {
                    run.loopStartUs = bootUs;
                    run.sampleUs = input.acquisitionUs();
                    run.workUs =
                            timing.extraWorkUs()
                                    + (timing.workJitterUs() == 0
                                            ? 0
                                            : run.timingRandom.nextInt(timing.workJitterUs() + 1));
                    boolean freshGps = run.gpsFixUs > run.deliveredGpsUs;
                    run.fc.pre_loop(
                            bootUs,
                            new RTFC.Inputs(
                                    input.acquisitionUs(),
                                    input.accel(),
                                    input.baro(),
                                    freshGps ? run.gpsFix : null,
                                    input.gyro(),
                                    run.gpsFixUs));
                    if (freshGps) {
                        run.deliveredGpsUs = run.gpsFixUs;
                        run.fc.trace.log(
                                "gps.consume",
                                "fix_acquisition_us="
                                        + run.gpsFixUs
                                        + " age_us="
                                        + (bootUs - run.gpsFixUs));
                    }
                    run.fc.beginLoop();
                    run.phase = 1;
                    run.nextCpuUs = bootUs + timing.sensorReadUs();
                }
                case 1 -> {
                    run.fc.readSensors();
                    run.maxSensorAgeUs = Math.max(run.maxSensorAgeUs, bootUs - run.sampleUs);
                    run.phase = 2;
                    run.nextCpuUs = bootUs + run.workUs;
                }
                case 2 -> {
                    run.fc.completeLoop();
                    run.lastControlUs = bootUs;
                    run.lastControlSampleUs = run.sampleUs;
                    long execution = bootUs - run.loopStartUs;
                    // FC.ino waits on millis(), not micros(): preserve its millisecond
                    // quantization.
                    long earliestStart = (run.loopStartUs / 1000 + 10) * 1000;
                    long overrun = Math.max(0, bootUs - earliestStart);
                    run.completedLoops++;
                    if (overrun > 0) run.overruns++;
                    run.maxExecutionUs = Math.max(run.maxExecutionUs, execution);
                    run.nextCpuUs = Math.max(bootUs, earliestStart);
                    run.phase = 0;
                    run.fc.trace.log(
                            "timing.loop",
                            "start_us="
                                    + run.loopStartUs
                                    + " execution_us="
                                    + execution
                                    + " overrun_us="
                                    + overrun
                                    + " next_start_us="
                                    + run.nextCpuUs);
                }
                default -> throw new IllegalStateException("Invalid FC execution phase");
            }
        }
        run.fc.dispatchControllers(bootUs);
        run.fc.dispatchBoards(bootUs);
        if (run.fc.getLastBoardCommandUs() == bootUs) {
            run.lastControlUs = bootUs;
            run.lastControlSampleUs = run.fc.getLastBoardSampleUs();
        }
        if (bootUs >= WARMUP_US && run.fc.getControllers() != null)
            run.fc.trace.log(
                    "fc.controller_values",
                    FlightComputerControllerPlots.logSnapshot(
                                    run.fc, design.design(), (bootUs - WARMUP_US) / 1e6)
                            .toString());
        run.fc.telemetry.deliverDue(bootUs);
        if (missionHook != null) missionHook.after(bootUs);
    }

    public record TimingSummary(
            long completedLoops,
            long overruns,
            long maxExecutionUs,
            long maxSensorAgeUs,
            long maxPwmSampleAgeUs) {}

    /**
     * Returns accumulated scheduler timing statistics for the run.
     *
     * @return accumulated scheduler timing statistics for the run
     */
    public TimingSummary getTimingSummary() {
        return run == null
                ? new TimingSummary(0, 0, 0, 0, 0)
                : new TimingSummary(
                        run.completedLoops,
                        run.overruns,
                        run.maxExecutionUs,
                        run.maxSensorAgeUs,
                        run.maxPwmSampleAgeUs);
    }

    /**
     * Dispatches due work and limits the next physics step so no virtual hardware deadline is
     * skipped.
     *
     * @param status stat in microseconds
     * @return true when the condition described above holds
     * @throws SimulationException if the requested operation cannot be completed
     */
    @Override
    public boolean preStep(SimulationStatus status) throws SimulationException {
        run.stepStart = status.getSimulationTime();
        run.candidate = null;
        run.intervalOpen = true;
        run.communicator.bind(status, run.fc.connected("airbrakes"), run.fc.connected("roll"));
        run.communicator.apply(run.fc.getOutput(), holdAirbrakesClosed);
        run.fc.trace.log("physics.begin", "simulation_s=" + run.stepStart);
        return true;
    }

    /**
     * Captures the accepted acceleration candidate used to form the next measured sensor
     * acquisition.
     *
     * @param status stat in microseconds
     * @param acceleration acceleration
     * @return the resulting AccelerationData
     */
    @Override
    public AccelerationData postAccelerationCalculation(
            SimulationStatus status, AccelerationData acceleration) {
        if (run != null
                && run.intervalOpen
                && run.candidate == null
                && abs(status.getSimulationTime() - run.stepStart) < 1e-10) {
            run.candidate =
                    sample(
                            status,
                            acceleration.getLinearAccelerationWC(),
                            WARMUP_US + Math.round(status.getSimulationTime() * 1_000_000));
            recordData(status, run.candidate);
            long plotUs = Math.round(status.getSimulationTime() * 1_000_000);
            if (plotUs - run.lastPlotLogUs >= 10000) {
                run.lastPlotLogUs = plotUs;
                run.fc.trace.log(
                        "fc.plot",
                        FlightComputerData.snapshot(status.getFlightDataBranch()).toString());
            }
        }
        return null; // Observe; never override the derivative.
    }

    /**
     * Publishes sensor truth from an accepted physics step, advances due virtual work and records
     * synchronized FC plot data.
     *
     * @param status stat in microseconds
     * @throws SimulationException if the requested operation cannot be completed
     */
    @Override
    public void postStep(SimulationStatus status) throws SimulationException {
        run.intervalOpen = false;
        if (missionHook != null) missionHook.accepted(status);
        double time = status.getSimulationTime();
        if (time <= run.stepStart) {
            run.fc.trace.log("physics.no_advance", "simulation_s=" + time);
            return;
        }
        long us = Math.round(time * 1_000_000);
        if (us == run.lastPublishedUs) return;
        run.lastPublishedUs = us;
        if (run.candidate == null) {
            if (!status.isLanded())
                throw new SimulationException(
                        "Missing coherent FC acceleration sample at " + run.stepStart);
            run.candidate = sample(status, Coordinate.ZERO, WARMUP_US + us);
        }
        run.latest = run.candidate;
        run.fc.trace.log(
                "physics.accept",
                "simulation_s="
                        + time
                        + " sample_us="
                        + run.latest.acquisitionUs()
                        + " truth_altitude_m="
                        + status.getRocketWorldPosition().getAltitude()
                        + " truth_velocity_mps="
                        + status.getRocketVelocity().z);
        long bootUs = WARMUP_US + us;
        if (bootUs > nextDeadlineUs())
            throw new SimulationException(
                    "FC deadline skipped: expected " + nextDeadlineUs() + " us, got " + bootUs);
        if (bootUs == nextDeadlineUs()) {
            dispatch(bootUs, run.latest);
            run.recovery.drain(status);
            run.communicator.apply(run.fc.getOutput(), holdAirbrakesClosed);
        }
    }

    /**
     * Called after storing a physical point, and refined with its first coherent acceleration
     * stage.
     */
    public void recordData(SimulationStatus status) {
        if (run == null) return;
        recordData(
                status,
                sample(
                        status,
                        Coordinate.ZERO,
                        WARMUP_US + Math.round(status.getSimulationTime() * 1_000_000)));
        if (!status.isLanded())
            for (var pair : FlightComputerData.ACCEL)
                status.getFlightDataBranch().setValue(pair.truth(), Double.NaN);
    }

    /**
     * Records measured inputs, actuator outputs, controller diagnostics and state alongside the
     * current physics sample.
     *
     * @param status stat in microseconds
     * @param truth truth
     */
    private void recordData(SimulationStatus status, RTFC.Inputs truth) {
        FlightComputerData.record(
                status.getFlightDataBranch(),
                run.fc,
                truth,
                design == null ? null : design.design());
        run.communicator.recordData(status.getFlightDataBranch(), run.fc.getOutput());
        FlightComputerControllerPlots.record(
                status.getFlightDataBranch(), run.fc, design == null ? null : design.design());
        if (run.recovery != null) run.recovery.record(status.getFlightDataBranch(), status);
    }

    /** Coherent engineering-unit observation; simulation body Z is the longitudinal axis. */
    public static RTFC.Inputs sample(
            SimulationStatus status, Coordinate accelerationWC, long acquisitionUs) {
        double gravity =
                status.getSimulationConditions()
                        .getGravityModel()
                        .getGravity(status.getRocketWorldPosition());
        Coordinate coriolis =
                status.getSimulationConditions()
                        .getGeodeticComputation()
                        .getCoriolisAcceleration(
                                status.getRocketWorldPosition(), status.getRocketVelocity());
        Coordinate body =
                status.getRocketOrientationQuaternion()
                        .invRotate(accelerationWC.sub(coriolis).add(0, 0, gravity));
        Coordinate omega =
                status.getRocketOrientationQuaternion()
                        .invRotate(status.getRocketRotationVelocity());
        AtmosphericConditions atmosphere =
                status.getSimulationConditions()
                        .getAtmosphericModel()
                        .getConditions(status.getRocketWorldPosition().getAltitude());
        WorldCoordinate location = status.getRocketWorldPosition();
        return new RTFC.Inputs(
                acquisitionUs,
                new RTAccelData(body.z, body.x, body.y),
                new RTBaroData(
                        Float.NaN,
                        Float.NaN,
                        (float) (atmosphere.getTemperature() - 273.15),
                        Float.NaN,
                        (float) (atmosphere.getPressure() / 100.0)),
                new RTGPSData(
                        location.getLatitudeDeg(),
                        location.getLongitudeDeg(),
                        location.getAltitude(),
                        0.0,
                        0.0,
                        0.0,
                        true),
                new RTGyroData(toDegrees(omega.z), toDegrees(omega.x), toDegrees(omega.y)));
    }

    /**
     * Finds the active FC listener on a simulation status, rejecting duplicate FC listeners and
     * returning null when none is installed.
     *
     * @param status stat in microseconds
     * @return the resulting FlightControllerSimulatorListener
     * @throws IllegalArgumentException when rejected: Register only one FC listener per simulation
     */
    public static FlightControllerSimulatorListener active(SimulationStatus status) {
        FlightControllerSimulatorListener found = null;
        for (SimulationListener l : status.getSimulationConditions().getSimulationListenerList())
            if (l instanceof FlightControllerSimulatorListener fc) {
                if (found != null)
                    throw new IllegalArgumentException(
                            "Register only one FC listener per simulation");
                found = fc;
            }
        return found;
    }

    /** Used by the existing engine/steppers only when this listener is active. */
    public double limitStep(SimulationStatus status, double eventLimit) {
        double untilTick =
                (nextDeadlineUs() - WARMUP_US) / 1_000_000.0 - status.getSimulationTime();
        if (!(untilTick > 0))
            throw new IllegalStateException(
                    "FC deadline must execute before another physics interval");
        return Math.min(Math.min(eventLimit, maxPhysicsStep), untilTick);
    }

    /**
     * Finalizes the run and closes telemetry after success, cancellation or simulation failure.
     *
     * @param status stat in microseconds
     * @param exception exception
     */
    @Override
    public void endSimulation(SimulationStatus status, SimulationException exception) {
        boolean complete =
                exception == null
                        && status.getFlightDataBranch().getEvents().stream()
                                .noneMatch(
                                        e ->
                                                e.getType() == FlightEvent.Type.SIM_ABORT
                                                        || e.getType()
                                                                == FlightEvent.Type.EXCEPTION);
        if (run != null) {
            if (status.getFlightDataBranch().getLength() > 0)
                run.fc.trace.log(
                        "fc.plot",
                        FlightComputerData.snapshot(status.getFlightDataBranch()).toString());
            var visible =
                    java.util.EnumSet.of(
                            FlightEvent.Type.LAUNCH,
                            FlightEvent.Type.IGNITION,
                            FlightEvent.Type.LIFTOFF,
                            FlightEvent.Type.BURNOUT,
                            FlightEvent.Type.APOGEE,
                            FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT,
                            FlightEvent.Type.GROUND_HIT,
                            FlightEvent.Type.STAGE_SEPARATION,
                            FlightEvent.Type.TUMBLE,
                            FlightEvent.Type.SIM_ABORT);
            for (var event : status.getFlightDataBranch().getEvents())
                if (visible.contains(event.getType())
                        && !(event.getData() instanceof RecoveryDeployment))
                    run.fc.trace.log(
                            "fc.plot_event", FlightComputerData.eventSnapshot(event).toString());
        }
        if (missionHook != null) missionHook.end(status, complete);
        finish(complete);
    }

    /**
     * Finalizes run resources and records completion information without retaining a previous
     * flight's data.
     */
    public void finish() {
        finish(false);
    }

    /**
     * Finalizes run resources and records completion information without retaining a previous
     * flight's data.
     *
     * @param complete whether the run completed successfully
     */
    private void finish(boolean complete) {
        if (run != null && !run.closed) {
            run.closed = true;
            if (run.recovery != null) run.recovery.close();
            run.fc.trace.log(
                    "timing.summary", getTimingSummary() + " unfinished_loop=" + (run.phase != 0));
            run.fc.trace.log("simulation.end", "loops=" + run.fc.getLoopCount());
            if (missionHook == null) run.fc.telemetry.finish(complete);
            else run.fc.telemetry.finishAt(complete, run.fc.trace.bootUs());
        }
    }

    // don't worry about it
    /**
     * Finds the active tab-controlled fin set used by the legacy compatibility API.
     *
     * @param status stat in microseconds
     * @return the resulting TabControlledTrapezoidFinSet
     */
    public static TabControlledTrapezoidFinSet getTheFinsToModifyTabs(SimulationStatus status) {
        ArrayList<TabControlledTrapezoidFinSet> finSets = new ArrayList<>();
        Rocket rocket = status.getConfiguration().getRocket();
        for (Iterator<RocketComponent> it = rocket.iterator(true); it.hasNext(); ) {
            RocketComponent component = it.next();

            if (component instanceof TabControlledTrapezoidFinSet) {
                finSets.add((TabControlledTrapezoidFinSet) component);
            }
        }
        return finSets.get(0);
    }

    /**
     * Sets a simulated fin-tab angle through the legacy compatibility API.
     *
     * @param newAngle new angle
     */
    public static void setFinTabAngle(double newAngle) {
        double stepSize = servoRangeAngleDeg / servoStepCount;
        double numStepsFromZero = (int) (newAngle / stepSize);
        if (latestStatus.getSimulationTime() - lastServoCommandTimestamp < SERVO_REFRESH_TIME) {
            return; // no command allowed.
        }
        theFinsToModify.setTabAngle(Math.PI / 180 * numStepsFromZero * stepSize);
        lastServoCommandTimestamp = latestStatus.getSimulationTime();
        if (flagPrintDebugMsg) {
            System.out.println(
                    "[JAVA] Actuated a servo change to "
                            + newAngle
                            + " degrees, which is "
                            + numStepsFromZero
                            + " steps.");
        }
    }

    /**
     * Writes the supplied angle to the tab-controlled fin geometry.
     *
     * @param newAngle new angle
     */
    public static void setFinTabAngleLowlevel(double newAngle) {
        if (newAngle > 10) {
            newAngle = 10;
        }
        if (newAngle < -10) {
            newAngle = -10;
        }
        theFinsToModify.setTabAngle(Math.PI / 180 * newAngle);
    }

    /**
     * Reads the simulated fin-tab angle in degrees.
     *
     * @return computed numeric value
     */
    public static double getFinTabAngleDeg() {
        return theFinsToModify.getTabAngle() * 180 / PI;
    }

    // Master Fudger

    /**
     * Converts the accepted OpenRocket state to engineering-unit FC inputs.
     *
     * @param fudgedStatus fudged stat in microseconds
     */
    public static void fudgeSimulationStatus(SimulationStatus fudgedStatus) {
        // accel
        List<Double> accelZ =
                fudgedStatus.getFlightDataBranch().get(FlightDataType.TYPE_ACCELERATION_Z);
        List<Double> accelXY =
                fudgedStatus.getFlightDataBranch().get(FlightDataType.TYPE_ACCELERATION_XY);

        Coordinate realAccel =
                new Coordinate(
                        accelXY.get(accelXY.size() - 1),
                        accelXY.get(accelXY.size() - 1),
                        accelZ.get(accelZ.size() - 1));
        Coordinate fudgedAccel = fudgeAccel(realAccel);
        fudgedStatus.putExtraData("fudged_accel", fudgedAccel);

        // baro

        double alt = fudgedStatus.getRocketWorldPosition().getAltitude();
        AtmosphericConditions atmos =
                fudgedStatus.getSimulationConditions().getAtmosphericModel().getConditions(alt);

        double rP0 = atmos.getPressure();
        double rT0 = atmos.getTemperature();
        double t0 = atmos.getTemperature();
        double a0 = alt;
        double p0 = atmos.getPressure();

        double rP1 = fudgeRawPressure(rP0);
        double rT1 = fudgeRawTemperature(rT0);
        double t1 = fudgeTemperature(t0);
        double a1 = fudgeAltitude(alt);
        double p1 = fudgePressure(p0);

        fudgedStatus.putExtraData("fudged_rawPressure", rP1);
        fudgedStatus.putExtraData("fudged_rawTemperature", rT1);
        fudgedStatus.putExtraData("fudged_temperature", t1);
        fudgedStatus.putExtraData("fudged_altitude", a1);
        fudgedStatus.putExtraData("fudged_pressure", p1);

        // GPS
        WorldCoordinate realLocation = fudgedStatus.getRocketWorldPosition();
        WorldCoordinate fudgedLocation = fudgeGPS(realLocation);
        boolean gps_has_fix = true; // possibly fudge this too.
        fudgedStatus.putExtraData("fudged_gps_has_fix", gps_has_fix);
        fudgedStatus.putExtraData("fudged_gps_position", fudgedLocation);

        // Fudge the orientation

        List<Double> rollAngles =
                fudgedStatus.getFlightDataBranch().get(FlightDataType.TYPE_ORIENTATION_PHI);
        List<Double> pitchAngles =
                fudgedStatus.getFlightDataBranch().get(FlightDataType.TYPE_ORIENTATION_THETA);

        double rollAngle = rollAngles.get(rollAngles.size() - 1);
        double pitchAngle = pitchAngles.get(pitchAngles.size() - 1);
        Coordinate worldAngle = convertToImuAngles(rollAngle, pitchAngle);
        Coordinate fudgedWorldAngle = fudgeWorldAngle(worldAngle);
        fudgedStatus.putExtraData("fudged_world_angle", fudgedWorldAngle);

        List<Double> rollRates =
                fudgedStatus.getFlightDataBranch().get(FlightDataType.TYPE_ROLL_RATE);
        List<Double> pitchRates =
                fudgedStatus.getFlightDataBranch().get(FlightDataType.TYPE_PITCH_RATE);

        double rollRate = rollRates.get(rollRates.size() - 1);
        double pitchRate = pitchRates.get(pitchRates.size() - 1);
        Coordinate worldAngRate = convertToImuAngles(rollRate, pitchRate);
        Coordinate fudgedWorldAngleRate = fudgeWorldAngleRate(worldAngRate);
        fudgedStatus.putExtraData("fudged_world_angle_rate", fudgedWorldAngleRate);

        // no return, its a shared reference
    }

    // Fudgers per sensor
    // Accel
    /**
     * Transforms simulated specific force into the FC sensor frame: longitudinal X corresponds to
     * OpenRocket body Z.
     *
     * @param accel accel
     * @return the resulting Coordinate
     */
    public static Coordinate fudgeAccel(Coordinate accel) {
        // for the moment no-op.
        return accel;
    }

    // Baro
    /**
     * Applies the legacy uniform altitude perturbation; active sensor noise is configured through
     * the design sensor models.
     *
     * @param altitude altitude
     * @return computed numeric value
     */
    public static double fudgeAltitude(double altitude) {
        // fudge with ± amplitude_randomness_size meters
        return altitude + (0.5 - random()) * 2 * amplitude_randomness_size;
    }

    /**
     * Returns the legacy raw-pressure input unchanged.
     *
     * @param rP0 r p0
     * @return the legacy raw-pressure input unchanged
     */
    public static double fudgeRawPressure(double rP0) {
        // for the moment no-op.
        return rP0;
    }

    /**
     * Returns the legacy raw-temperature input unchanged.
     *
     * @param rT0 r t0
     * @return the legacy raw-temperature input unchanged
     */
    public static double fudgeRawTemperature(double rT0) {
        // for the moment no-op.
        return rT0;
    }

    /**
     * Returns the legacy temperature input unchanged.
     *
     * @param t0 t0
     * @return the legacy temperature input unchanged
     */
    public static double fudgeTemperature(double t0) {
        // for the moment no-op.
        return t0;
    }

    /**
     * Returns the legacy pressure input unchanged.
     *
     * @param p0 p0
     * @return the legacy pressure input unchanged
     */
    public static double fudgePressure(double p0) {
        // for the moment no-op.
        return p0;
    }

    // GPS
    /**
     * Builds a GPS observation from the simulated position and modeled fix information.
     *
     * @param location location
     * @return the resulting WorldCoordinate
     */
    public static WorldCoordinate fudgeGPS(WorldCoordinate location) {
        // for the moment no-op.
        return new WorldCoordinate(
                location.getLatitudeRad(),
                location.getLongitudeRad(),
                location.getAltitude() // GPS-specific altitude fudging, not shared with baro.
                );
    }

    // Mag / IMU
    /**
     * Transforms the simulated orientation into the FC angular convention.
     *
     * @param worldAngle world angle
     * @return the resulting Coordinate
     */
    public static Coordinate fudgeWorldAngle(Coordinate worldAngle) {
        // for the moment no-op.
        return worldAngle;
    }

    // Gyro
    /**
     * Transforms simulated angular velocity into the FC gyro frame and units.
     *
     * @param worldAngleRate world angle rate
     * @return the resulting Coordinate
     */
    public static Coordinate fudgeWorldAngleRate(Coordinate worldAngleRate) {
        // for the moment no-op.
        return worldAngleRate;
    }

    /**
     * Converts a rotation quaternion to the Euler-angle convention used by the rocket sensor
     * adapter.
     *
     * @param q q
     * @return the resulting Coordinate
     */
    public static Coordinate toEulerAngles_rocketCoord(Quaternion q) {

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.getW() * q.getX() + q.getY() * q.getZ());
        double cosr_cosp = 1 - 2 * (q.getX() * q.getX() + q.getY() * q.getY());
        double angleX = atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = sqrt(1 + 2 * (q.getW() * q.getY() - q.getX() * q.getZ()));
        double cosp = sqrt(1 - 2 * (q.getW() * q.getY() - q.getX() * q.getZ()));
        double angleY = 2 * atan2(sinp, cosp) - PI / 2;

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.getW() * q.getZ() + q.getX() * q.getY());
        double cosy_cosp = 1 - 2 * (q.getY() * q.getY() + q.getZ() * q.getZ());
        double angleZ = atan2(siny_cosp, cosy_cosp);

        if (roundToNearest5) {
            angleX = round(angleX / (Math.PI / 180 * 1)); // * (Math.PI / 180 * 5);
            angleY = round(angleY / (Math.PI / 180 * 1)); // * (Math.PI / 180 * 5);
            angleZ = round(angleZ / (Math.PI / 180 * 1)); // * (Math.PI / 180 * 5);
        }

        return new Coordinate(angleX, angleY, angleZ);
    }

    /**
     * Finds the active physical airbrake component for the legacy compatibility API.
     *
     * @param status stat in microseconds
     * @return the resulting AirbrakeSet
     */
    public static AirbrakeSet getAirbrakes(SimulationStatus status) {
        java.util.ArrayList<AirbrakeSet> sets = new java.util.ArrayList<>();
        Rocket rocket = status.getConfiguration().getRocket();
        for (Iterator<RocketComponent> it = rocket.iterator(true); it.hasNext(); ) {
            RocketComponent component = it.next();

            if (component instanceof AirbrakeSet) {
                sets.add((AirbrakeSet) component);
            }
        }
        return sets.get(0);
    }
}
