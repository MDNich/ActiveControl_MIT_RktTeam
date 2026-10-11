package info.openrocket.core.simulation.flightcomputer;

import edu.mit.rocket_team.FC.RTFC;
import edu.mit.rocket_team.RTFCVerificationRocket;
import edu.mit.rocket_team.util.RTUtilLibrary.Trace;
import edu.mit.rocket_team.util.data.*;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener;
import info.openrocket.core.util.BaseTestCase;
import jakarta.json.*;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import java.nio.file.*;
import java.util.*;
import static org.junit.jupiter.api.Assertions.*;

class FlightComputerControllerTest extends BaseTestCase {
    @TempDir Path temp;
    String telemetry;

    /** Initializes application services and isolated state for this regression fixture. */
    @BeforeEach
    void setup() {
        telemetry = System.getProperty("openrocket.fc.telemetryDir");
        System.setProperty("openrocket.fc.telemetryDir", temp.toString());
    }

    /** Restores preferences, properties and test resources after each regression case. */
    @AfterEach
    void restore() {
        FlightComputerDesignTest.set("openrocket.fc.telemetryDir", telemetry);
    }

    /**
     * Builds or reads the block fixture for the regression assertions.
     *
     * @param type type
     * @param mode mode
     * @param command command
     * @param states states
     * @return the resulting FlightComputerController
     */
    private FlightComputerController block(
            String type, String mode, double command, String... states) {
        return new FlightComputerController(
                Json.createObjectBuilder(
                                FlightComputerController.create(FlightComputerDesign.zephyrus())
                                        .json())
                        .add("enabled", true)
                        .add("type", type)
                        .add("mode", mode)
                        .add("constantValue", command)
                        .add("minimum", mode.equals("AIRBRAKES") ? 0 : -90)
                        .add("maximum", mode.equals("AIRBRAKES") ? 1 : 90)
                        .add("states", Json.createArrayBuilder(Arrays.asList(states)))
                        .build());
    }

    /**
     * Builds or reads the changed fixture for the regression assertions.
     *
     * @param b b
     * @param key configuration or lookup key
     * @param value replacement or input value
     * @return the resulting FlightComputerController
     */
    private FlightComputerController changed(FlightComputerController b, String key, double value) {
        return b.with(Json.createObjectBuilder(b.json()).add(key, value).build());
    }

    /**
     * Builds or reads the design fixture for the regression assertions.
     *
     * @param blocks blocks
     * @return the resulting FlightComputerDesign
     */
    private FlightComputerDesign design(FlightComputerController... blocks) {
        var d =
                FlightComputerController.replace(
                        FlightComputerDesign.zephyrus(), Arrays.asList(blocks));
        d.requireRunnable();
        return d;
    }

    /**
     * Builds or reads the computer fixture for the regression assertions.
     *
     * @param d d
     * @param log log
     * @return the resulting RTFC
     */
    private RTFC computer(FlightComputerDesign d, List<String> log) {
        var fc = new RTFC(new Trace(log::add));
        fc.configure(d);
        fc.init();
        return fc;
    }

    /**
     * Builds one timestamped sensor input for deterministic controller verification.
     *
     * @param t t
     * @param roll roll
     * @return the resulting Inputs
     */
    private RTFC.Inputs sample(long t, float roll) {
        return new RTFC.Inputs(
                t,
                new RTAccelData(9.8065f, 0f, 0f),
                new RTBaroData(Float.NaN, Float.NaN, 20f, Float.NaN, 1013.25f),
                new RTGPSData(0d, 0d, 0d, 0d, 0d, 0d, true),
                new RTGyroData(-roll, 0f, 0f));
    }

    /**
     * Builds or reads the read fixture for the regression assertions.
     *
     * @param fc fc
     * @param t t
     * @param roll roll
     */
    private void read(RTFC fc, long t, float roll) {
        fc.pre_loop(t, sample(t, roll));
        fc.loop();
    }

    /**
     * Advances scheduled controller work to the test instant.
     *
     * @param fc fc
     * @param t t
     */
    private void dispatch(RTFC fc, long t) {
        fc.trace.time(t);
        fc.dispatchControllers(t);
    }

    /**
     * Reads the current controller output for assertions.
     *
     * @param fc fc
     * @param b b
     * @return computed numeric value
     */
    private double command(RTFC fc, FlightComputerController b) {
        return fc.getControllers().outputs().get(b.id());
    }

    /**
     * Verifies that legacy files remain version one and version two round trips.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void legacyFilesRemainVersionOneAndVersionTwoRoundTrips() throws Exception {
        var old = FlightComputerDesign.zephyrus();
        assertEquals(1, old.json().getInt("version"));
        assertTrue(old.controllers().isEmpty());
        var b = block("PID", "TABS", 0, "flight");
        var d = design(b);
        Path file = temp.resolve("controller.fc");
        d.write(file);
        assertEquals(d.json(), FlightComputerDesign.read(file).json());
        assertEquals(2, d.json().getInt("version"));
        assertEquals(old.active("gyroscope"), d.active("gyroscope"));
    }

    /** Verifies that completion and interrupt first pwm tie are exact. */
    @Test
    void completionAndInterruptFirstPwmTieAreExact() {
        var b =
                changed(
                        changed(block("CONSTANT", "TABS", 30, "ground"), "periodUs", 1000),
                        "executionUs",
                        20000);
        var fc = computer(design(b), new ArrayList<>());
        dispatch(fc, 0);
        assertEquals(20000, fc.getNextBoardDeadlineUs());
        assertEquals(0, command(fc, b));
        fc.trace.time(20000);
        fc.latchPwm();
        dispatch(fc, 20000);
        assertEquals(0, fc.getOutput().controllerOutputs().get(b.id()));
        assertEquals(30, command(fc, b));
        assertEquals(40000, fc.getNextBoardDeadlineUs());
        fc.trace.time(40000);
        fc.latchPwm();
        assertEquals(30, fc.getOutput().controllerOutputs().get(b.id()));
    }

    /** Verifies that measured input is frozen at start and integral uses elapsed seconds. */
    @Test
    void measuredInputIsFrozenAtStartAndIntegralUsesElapsedSeconds() {
        var b =
                changed(
                        changed(changed(block("PID", "TABS", 0, "ground"), "kp", 1), "ki", 2),
                        "executionUs",
                        5000);
        var fc = computer(design(b), new ArrayList<>());
        read(fc, 0, 10);
        dispatch(fc, 0);
        read(fc, 4000, 70);
        dispatch(fc, 5000);
        assertEquals(-10, command(fc, b), 1e-6);
        read(fc, 10000, 20);
        dispatch(fc, 10000);
        dispatch(fc, 15000);
        assertEquals(-20.4, command(fc, b), 1e-5);
        assertEquals(10000, fc.getControllers().values().get(b.id()).sampleUs());
    }

    /** Verifies that anti windup stops against asymmetric target and unwinds. */
    @Test
    void antiWindupStopsAgainstAsymmetricTargetAndUnwinds() {
        var b =
                changed(
                        changed(
                                changed(block("PID", "TABS", 0, "ground"), "ki", 1),
                                "periodUs",
                                1000000),
                        "staleUs",
                        2000000);
        var fc = computer(design(b), new ArrayList<>());
        fc.getControllers().constrain(b.id(), v -> Math.abs(v - Math.max(0, Math.min(3, v))));
        for (int i = 0; i < 5; i++) {
            read(fc, i * 1000000L, -1);
            dispatch(fc, i * 1000000L);
        }
        assertEquals(3, command(fc, b), 1e-6);
        read(fc, 5000000, 1);
        dispatch(fc, 5000000);
        assertEquals(2, command(fc, b), 1e-6);
    }

    /** Verifies that invalid input expires to neutral and constant needs no sensor. */
    @Test
    void invalidInputExpiresToNeutralAndConstantNeedsNoSensor() {
        var b = changed(changed(block("PID", "TABS", 0, "ground"), "kp", 1), "staleUs", 5000);
        var fc = computer(design(b), new ArrayList<>());
        read(fc, 0, 12);
        dispatch(fc, 0);
        assertEquals(-12, command(fc, b), 1e-6);
        dispatch(fc, 10000);
        assertEquals(0, command(fc, b));
        assertFalse(fc.getControllers().values().get(b.id()).valid());
        var c = block("CONSTANT", "TABS", 8, "flight");
        var inactive = computer(design(c), new ArrayList<>());
        dispatch(inactive, 0);
        assertEquals(0, command(inactive, c), "Inactive constant must use neutral");
    }

    /** Verifies that velocity weight remains finite at zero and respects activation. */
    @Test
    void velocityWeightRemainsFiniteAtZeroAndRespectsActivation() {
        var b =
                changed(
                        changed(block("VELOCITY_PID", "TABS", 0, "ground"), "kp", 1),
                        "maximumWeight",
                        3);
        var fc = computer(design(b), new ArrayList<>());
        read(fc, 0, 2);
        dispatch(fc, 0);
        assertEquals(-6, command(fc, b), 1e-6);
        assertEquals(3, fc.getControllers().values().get(b.id()).weight());
        b = changed(b, "activationSpeed", 1);
        var inactive = computer(design(b), new ArrayList<>());
        read(inactive, 0, 2);
        dispatch(inactive, 0);
        assertEquals(0, command(inactive, b));
    }

    /** Verifies that additional sensors have independent bias and stable selections. */
    @Test
    void additionalSensorsHaveIndependentBiasAndStableSelections() {
        var base = FlightComputerDesign.zephyrus();
        var node = FlightComputerModels.newNode("gyroscope", "", 10, 10);
        node =
                Json.createObjectBuilder(node)
                        .add("id", "second-gyro")
                        .add(
                                "properties",
                                Json.createObjectBuilder(node.getJsonObject("properties"))
                                        .add("bias", 7))
                        .build();
        var b = block("PID", "TABS", 0, "ground");
        b =
                b.with(
                        Json.createObjectBuilder(b.json())
                                .add("source", "second-gyro")
                                .add("kp", 1)
                                .build());
        var d =
                FlightComputerController.replace(
                        base.with("nodes", Json.createArrayBuilder(base.nodes()).add(node).build()),
                        List.of(b));
        d.requireRunnable();
        var fc = computer(d, new ArrayList<>());
        read(fc, 0, 10);
        dispatch(fc, 0);
        assertEquals(3, fc.getControllers().values().get(b.id()).measurement(), 1e-6);
        assertEquals(10, fc.gyro.getRollRate(), 1e-6);
        assertEquals(-3, command(fc, b), 1e-6);
        var first = fc.getControllerSignals().read("second-gyro", "roll_rate");
        dispatch(fc, 10000);
        assertEquals(
                first,
                fc.getControllerSignals().read("second-gyro", "roll_rate"),
                "Controller reads cannot acquire another sensor sample");
    }

    /** Verifies that state changes discard in flight work and recovery states can own blocks. */
    @Test
    void stateChangesDiscardInFlightWorkAndRecoveryStatesCanOwnBlocks() {
        var b =
                changed(
                        changed(
                                block("CONSTANT", "TABS", 30, "ground", "apogee"),
                                "periodUs",
                                1000),
                        "executionUs",
                        8000);
        var d = design(b);
        var states = Json.createArrayBuilder();
        for (String state : List.of("ground", "flight", "apogee"))
            states.add(
                    Json.createObjectBuilder()
                            .add("id", state)
                            .add("name", state)
                            .add("phase", state)
                            .add("actions", Json.createArrayBuilder()));
        var transitions =
                Json.createArrayBuilder()
                        .add(
                                Json.createObjectBuilder()
                                        .add("id", "lift")
                                        .add("from", "ground")
                                        .add("to", "flight")
                                        .add("priority", 0)
                                        .add(
                                                "condition",
                                                FlightComputerStateMachine.condition(
                                                        "state_ms", ">=", 5)))
                        .add(
                                Json.createObjectBuilder()
                                        .add("id", "recover")
                                        .add("from", "flight")
                                        .add("to", "apogee")
                                        .add("priority", 0)
                                        .add(
                                                "condition",
                                                FlightComputerStateMachine.condition(
                                                        "state_ms", ">=", 20)));
        d =
                d.with(
                        "stateMachine",
                        Json.createObjectBuilder()
                                .add("initial", "ground")
                                .add("automaticRecovery", false)
                                .add("states", states)
                                .add("transitions", transitions)
                                .build());
        d.requireRunnable();
        var log = new ArrayList<String>();
        var fc = computer(d, log);
        read(fc, 0, 0);
        dispatch(fc, 0);
        read(fc, 5000, 0);
        dispatch(fc, 5000);
        dispatch(fc, 8000);
        assertEquals(0, command(fc, b));
        assertTrue(log.stream().anyMatch(x -> x.contains("controller.discard")));
        dispatch(fc, 16000);
        dispatch(fc, 24000);
        read(fc, 25000, 0);
        dispatch(fc, 25000);
        dispatch(fc, 32000);
        dispatch(fc, 40000);
        assertEquals(30, command(fc, b), "An explicitly enabled recovery state must execute");
    }

    /** Verifies that ground capture waits for calibration and wrapped angles cross dateline. */
    @Test
    void groundCaptureWaitsForCalibrationAndWrappedAnglesCrossDateline() {
        var b = block("PID", "TABS", 0, "ground", "preflight");
        b =
                b.with(
                        Json.createObjectBuilder(b.json())
                                .add("baseline", "ground")
                                .add("setpoint", 2)
                                .add("kp", 1)
                                .build());
        var fc = computer(design(b), new ArrayList<>());
        read(fc, 0, 7);
        dispatch(fc, 0);
        assertFalse(fc.getControllers().values().get(b.id()).valid());
        assertTrue(Double.isNaN(fc.getControllers().values().get(b.id()).baseline()));
        fc.enqueueCommand(RTFC.stateCommand(edu.mit.rocket_team.util.RTRocketState.PRE_FLIGHT));
        read(fc, 10000, 7);
        dispatch(fc, 10000);
        assertTrue(Double.isNaN(fc.getControllers().values().get(b.id()).baseline()));
        read(fc, 20000, 7);
        dispatch(fc, 20000);
        assertEquals(9, fc.getControllers().values().get(b.id()).baseline(), 1e-6);
        assertEquals(2, command(fc, b), 1e-6);
        var angle = block("PID", "TABS", 0, "ground");
        angle =
                angle.with(
                        Json.createObjectBuilder(angle.json())
                                .add(
                                        "source",
                                        FlightComputerController.source(
                                                FlightComputerDesign.zephyrus(), "gps"))
                                .add("channel", "longitude")
                                .add("wrapAngle", true)
                                .add("setpoint", 179)
                                .add("kp", 1)
                                .build());
        fc = computer(design(angle), new ArrayList<>());
        var input = sample(0, 0);
        fc.pre_loop(
                0,
                new RTFC.Inputs(
                        0,
                        input.accel(),
                        input.baro(),
                        new RTGPSData(0d, -179d, 0d, 0d, 0d, 0d, true),
                        input.gyro()));
        fc.loop();
        dispatch(fc, 0);
        assertEquals(-2, command(fc, angle), 1e-6);
    }

    /** Verifies that controller noise is seeded and gps retains fix acquisition time. */
    @Test
    void controllerNoiseIsSeededAndGpsRetainsFixAcquisitionTime() {
        var b = changed(block("PID", "TABS", 0, "ground"), "kp", 1);
        var d = design(b);
        d =
                d.with(
                        "parameters",
                        Json.createObjectBuilder(d.parameters())
                                .add("measurementNoisePercent", 10)
                                .build());
        var a = computer(d, new ArrayList<>());
        var z = computer(d, new ArrayList<>());
        for (int t = 0; t <= 50000; t += 10000) {
            read(a, t, 10);
            read(z, t, 10);
            dispatch(a, t);
            dispatch(z, t);
            assertEquals(command(a, b), command(z, b));
        }
        assertNotEquals(-10, command(a, b));
        var fc = computer(design(b), new ArrayList<>());
        var in = sample(10000, 0);
        fc.pre_loop(
                10000, new RTFC.Inputs(10000, in.accel(), in.baro(), in.gps(), in.gyro(), 1234));
        fc.loop();
        assertEquals(
                1234,
                fc.getControllerSignals()
                        .read(FlightComputerController.source(design(b), "gps"), "altitude")
                        .acquisitionUs());
    }

    /** Verifies that cant changes forcing moment in an already created aerodynamic calculator. */
    @Test
    void cantChangesForcingMomentInAnAlreadyCreatedAerodynamicCalculator() {
        var r = rocket(false);
        var fin = (FinSet) target(r, "Aft fins");
        var calc = new info.openrocket.core.aerodynamics.barrowman.FinSetCalc(fin);
        var conditions =
                new info.openrocket.core.aerodynamics.FlightConditions(
                        r.getSelectedConfiguration());
        conditions.setMach(.2);
        conditions.setAOA(0);
        var forces = new info.openrocket.core.aerodynamics.AerodynamicForces();
        for (double angle : new double[] {2, 0, -2}) {
            fin.setCantAngle(Math.toRadians(angle));
            forces.zero();
            calc.calculateNonaxialForces(
                    conditions,
                    info.openrocket.core.util.Transformation.IDENTITY,
                    forces,
                    new info.openrocket.core.logging.WarningSet());
            assertEquals(Math.signum(angle), Math.signum(forces.getCrollForce()));
        }
    }

    /**
     * Builds the rocket fixture used by this regression case.
     *
     * @param two two
     * @return the resulting Rocket
     */
    private Rocket rocket(boolean two) {
        var r = RTFCVerificationRocket.rocket();
        var original =
                (TrapezoidFinSet)
                        r.getSelectedConfiguration().getActiveComponents().stream()
                                .filter(x -> x instanceof TrapezoidFinSet)
                                .findFirst()
                                .orElseThrow();
        var tab = new TabControlledTrapezoidFinSet(original);
        tab.setName("Aft fins");
        tab.setTabSpan(.08);
        tab.setTabChord(.04);
        tab.setTabOffset(.04);
        tab.setCNALPHA(2);
        if (two) {
            var f = new TabControlledTrapezoidFinSet(4, .3, .15, .1, .18, .08, .04, .04, 0);
            f.setName("Forward fins");
            f.setThickness(.005);
            f.setCNALPHA(2);
            tab.getParent().addChild(f);
            f.setAxialOffset(.8);
        }
        return r;
    }

    /**
     * Builds or reads the target fixture for the regression assertions.
     *
     * @param r r
     * @param name display name or lookup key
     * @return the resulting RocketComponent
     */
    private RocketComponent target(Rocket r, String name) {
        return r.getSelectedConfiguration().getActiveComponents().stream()
                .filter(x -> x.getName().equals(name))
                .findFirst()
                .orElseThrow();
    }

    /**
     * Builds or reads the binding fixture for the regression assertions.
     *
     * @param b b
     * @param r r
     * @param direction direction
     * @return the resulting Binding
     */
    private FlightComputerControllerSettings.Binding binding(
            FlightComputerController b, RocketComponent r, double direction) {
        return new FlightComputerControllerSettings.Binding(
                b.id(), r.getID(), direction, 0, b.n("minimum"), b.n("maximum"), false);
    }

    private record Result(Simulation sim, FlightDataBranch data, Path log) {}

    /**
     * Runs the configured flight fixture and returns recorded results.
     *
     * @param rocket rocket model for this operation
     * @param d d
     * @param bindings bindings
     * @param duration duration
     * @param roll roll
     * @return the resulting Result
     * @throws Exception if the requested operation cannot be completed
     */
    private Result flight(
            Rocket rocket,
            FlightComputerDesign d,
            List<FlightComputerControllerSettings.Binding> bindings,
            double duration,
            double roll)
            throws Exception {
        var fc =
                new FlightControllerSimulatorListener(s -> {}, .0025, false)
                        .withDesign(
                                new FlightComputerLibrary.Resolved(
                                        d, temp.resolve("test.fc"), "test"))
                        .withControllers(new FlightComputerControllerSettings(d.id(), bindings));
        var sim = RTFCVerificationRocket.simulation(rocket);
        sim.getOptions().setMaxSimulationTime(duration);
        sim.getOptions().setInitialRollRate(roll);
        sim.simulate(fc);
        return new Result(
                sim,
                sim.getSimulatedData().getBranch(0),
                fc.getFlightComputer().telemetry.getLogPath());
    }

    /**
     * Verifies that independent tabs and cant drive physics without mutating rocket.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void independentTabsAndCantDrivePhysicsWithoutMutatingRocket() throws Exception {
        var r = rocket(true);
        var a = block("CONSTANT", "TABS", 20, "preflight", "flight");
        var b = block("CONSTANT", "CANT", -2, "preflight", "flight");
        var d = design(a, b);
        var result =
                flight(
                        r,
                        d,
                        List.of(
                                binding(a, target(r, "Aft fins"), 1),
                                binding(b, target(r, "Forward fins"), 1)),
                        1.5,
                        0);
        assertEquals(
                Math.toRadians(20),
                result.data.getLast(
                        FlightComputerData.actuatorChannels(
                                        "roll",
                                        target(r, "Aft fins").getID().toString(),
                                        "Aft fins")
                                .physical()),
                1e-7);
        assertEquals(
                Math.toRadians(-2),
                result.data.getLast(
                        FlightComputerData.actuatorChannels(
                                        "cant",
                                        target(r, "Forward fins").getID().toString(),
                                        "Forward fins")
                                .physical()),
                1e-7);
        assertTrue(Math.abs(result.data.getLast(FlightDataType.TYPE_ROLL_RATE)) > .1);
        assertEquals(0, ((FinSet) target(r, "Forward fins")).getCantAngle());
        var imported = FlightComputerLogReader.read(result.log).get(0).data().getBranch(0);
        var channels = FlightComputerControllerPlots.channels(d, a);
        assertEquals(Math.toRadians(20), imported.getLast(channels.get("latched")), 1e-7);
        assertEquals(1, imported.getLast(channels.get("active")), 1e-7);
        var document = info.openrocket.core.document.OpenRocketDocumentFactory.createNewRocket();
        document.addSimulation(
                new Simulation(
                        document,
                        document.getRocket(),
                        Simulation.Status.EXTERNAL,
                        "Controller log",
                        new SimulationOptions(),
                        List.of(),
                        new FlightData(imported)));
        var storage = new info.openrocket.core.document.StorageOptions();
        storage.setSaveSimulationData(true);
        var out = new java.io.ByteArrayOutputStream();
        new info.openrocket.core.file.openrocket.OpenRocketSaver()
                .save(
                        out,
                        document,
                        storage,
                        new info.openrocket.core.logging.WarningSet(),
                        new info.openrocket.core.logging.ErrorSet());
        var context = new info.openrocket.core.file.DocumentLoadingContext();
        context.setOpenRocketDocument(
                info.openrocket.core.document.OpenRocketDocumentFactory.createEmptyRocket());
        context.setAttachmentFactory(
                new info.openrocket.core.file.FileSystemAttachmentFactory(temp.toFile()));
        new info.openrocket.core.file.openrocket.importt.OpenRocketLoader()
                .loadFromStream(
                        context,
                        new java.io.ByteArrayInputStream(out.toByteArray()),
                        "controller.ork");
        assertEquals(
                imported.get(channels.get("latched")),
                context.getOpenRocketDocument()
                        .getSimulation(0)
                        .getSimulatedData()
                        .getBranch(0)
                        .get(channels.get("latched")));
        assertFalse(FlightComputerData.isDiscrete(channels.get("error")));
        assertTrue(FlightComputerData.isDiscrete(channels.get("latched")));
    }

    /**
     * Verifies that closed loop roll damps initial spin.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void closedLoopRollDampsInitialSpin() throws Exception {
        var b = changed(block("PID", "TABS", 0, "flight"), "kp", .025);
        var r = rocket(false);
        var controlled = flight(r, design(b), List.of(binding(b, target(r, "Aft fins"), 1)), 2, 3);
        var neutral = flight(rocket(false), FlightComputerDesign.zephyrus(), List.of(), 2, 3);
        double on = Math.abs(controlled.data.getLast(FlightDataType.TYPE_ROLL_RATE)),
                off = Math.abs(neutral.data.getLast(FlightDataType.TYPE_ROLL_RATE));
        System.out.println("CONTROLLER_ROLL controlled=" + on + " baseline=" + off + " rad/s");
        assertTrue(
                on < off * .8, "Feedback must reduce spin: controlled=" + on + " baseline=" + off);
    }

    /**
     * Verifies that airbrake blocks lower altitude.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void airbrakeBlocksLowerAltitude() throws Exception {
        var b = block("CONSTANT", "AIRBRAKES", 1, "preflight", "flight");
        var r = rocket(false);
        var brake =
                r.getSelectedConfiguration().getActiveComponents().stream()
                        .filter(x -> x instanceof AirbrakeSet)
                        .findFirst()
                        .orElseThrow();
        var open = flight(r, design(b), List.of(binding(b, brake, 1)), 8, 0);
        var closed = flight(rocket(false), design(changed(b, "constantValue", 0)), List.of(), 8, 0);
        assertTrue(
                open.sim.getSimulatedData().getMaxAltitude()
                        < closed.sim.getSimulatedData().getMaxAltitude() - 1);
    }

    /** Verifies that ownership and mappings are validated and survive renames. */
    @Test
    void ownershipAndMappingsAreValidatedAndSurviveRenames() {
        var r = rocket(false);
        var a = block("PID", "TABS", 0, "flight");
        var b = block("PID", "TABS", 0, "flight");
        var d = design(a, b);
        var settings =
                new FlightComputerControllerSettings(
                        d.id(),
                        List.of(
                                binding(a, target(r, "Aft fins"), 1),
                                binding(b, target(r, "Aft fins"), -1)));
        assertTrue(
                assertThrows(
                                IllegalArgumentException.class,
                                () ->
                                        settings.validate(
                                                d,
                                                r.getSelectedConfiguration().getActiveComponents()))
                        .getMessage()
                        .contains("ownership"));
        var one = new FlightComputerControllerSettings(d.id(), List.of(settings.bindings().get(0)));
        target(r, "Aft fins").setName("Renamed");
        assertDoesNotThrow(
                () -> one.validate(d, r.getSelectedConfiguration().getActiveComponents()));
        var config = new info.openrocket.core.util.Config();
        one.write(config);
        assertEquals(one, FlightComputerControllerSettings.read(config));
        var state =
                new SimulationStatus(
                        r.getSelectedConfiguration(),
                        RTFCVerificationRocket.simulation(r).getOptions().toSimulationConditions());
        var runtime = new FlightComputerControllerRuntime(d, new Trace(s -> {}));
        var actuators =
                new FlightComputerControllerActuators(state, d, one, runtime, new Trace(s -> {}));
        assertThrows(
                IllegalArgumentException.class, () -> actuators.checkJavaWrite("roll", "Renamed"));
        assertDoesNotThrow(() -> actuators.checkJavaWrite("roll", null));
    }
}
