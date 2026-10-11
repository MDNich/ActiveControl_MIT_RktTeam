package info.openrocket.core.simulation.flightcomputer;

import edu.mit.rocket_team.FC.RTFC;
import edu.mit.rocket_team.RTFCVerificationRocket;
import edu.mit.rocket_team.util.RTSimulationCommunicator;
import edu.mit.rocket_team.util.RTUtilLibrary.Trace;
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

class FlightComputerActuationTest extends BaseTestCase {
    @TempDir Path temp;
    private String telemetry;

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
     * Builds the rocket fixture used by this regression case.
     *
     * @param twoCans two cans
     * @return the resulting Rocket
     */
    private Rocket rocket(boolean twoCans) {
        var rocket = RTFCVerificationRocket.rocket();
        var original =
                rocket.getSelectedConfiguration().getActiveComponents().stream()
                        .filter(c -> c instanceof TrapezoidFinSet)
                        .map(c -> (TrapezoidFinSet) c)
                        .findFirst()
                        .orElseThrow();
        var fins = new TabControlledTrapezoidFinSet(original);
        fins.setName("Aft fins");
        fins.setTabSpan(.08);
        fins.setTabChord(.04);
        fins.setTabOffset(.04);
        fins.setCNALPHA(2);
        if (twoCans) {
            var forward = new TabControlledTrapezoidFinSet(4, .3, .15, .1, .18, .08, .04, .04, 0);
            forward.setName("Forward fins");
            forward.setThickness(.005);
            forward.setCNALPHA(2);
            fins.getParent().addChild(forward);
            forward.setAxialOffset(.8);
        }
        return rocket;
    }

    /**
     * Builds or reads the design fixture for the regression assertions.
     *
     * @param source source text, model or measured-channel selection
     * @param airbrakes airbrakes
     * @return the resulting FlightComputerDesign
     */
    private FlightComputerDesign design(String source, boolean airbrakes) {
        var design = FlightComputerDesign.zephyrus();
        var nodes = Json.createArrayBuilder();
        var links = Json.createArrayBuilder();
        for (var node : design.nodes().getValuesAs(JsonObject.class))
            if (airbrakes || !node.getString("type").equals("airbrakes")) nodes.add(node);
        for (var link : design.connections().getValuesAs(JsonObject.class))
            if (airbrakes || !link.getString("port").equals("airbrakes")) links.add(link);
        nodes.add(
                Json.createObjectBuilder(FlightComputerModels.newNode("java_board", "", 0, 0))
                        .add("program", source)
                        .add(
                                "properties",
                                Json.createObjectBuilder()
                                        .add("periodUs", 1000)
                                        .add("executionUs", 0)));
        return design.with("nodes", nodes.build()).with("connections", links.build());
    }

    private record Result(Simulation simulation, FlightDataBranch branch, Path log) {}

    /**
     * Runs the configured test simulation and returns its flight results.
     *
     * @param rocket rocket model for this operation
     * @param commands commands
     * @param airbrakes airbrakes
     * @param duration duration
     * @return the resulting Result
     * @throws Exception if the requested operation cannot be completed
     */
    private Result run(Rocket rocket, String commands, boolean airbrakes, double duration)
            throws Exception {
        String source =
                "import info.openrocket.core.simulation.flightcomputer.JavaBoardProgram; public class BoardProgram extends JavaBoardProgram { public void step(Context io) { "
                        + commands
                        + " } }";
        var design = design(source, airbrakes);
        JavaBoardProgram.approve(source);
        try {
            var fc =
                    new FlightControllerSimulatorListener(s -> {}, .0025, false)
                            .withDesign(
                                    new FlightComputerLibrary.Resolved(
                                            design, temp.resolve("test.fc"), "test"));
            var sim = RTFCVerificationRocket.simulation(rocket);
            sim.getOptions().setMaxSimulationTime(duration);
            sim.simulate(fc);
            return new Result(
                    sim,
                    sim.getSimulatedData().getBranch(0),
                    fc.getFlightComputer().telemetry.getLogPath());
        } finally {
            JavaBoardProgram.revoke(source);
        }
    }

    /**
     * Builds or reads the roll fixture for the regression assertions.
     *
     * @param result result
     * @return computed numeric value
     */
    private double roll(Result result) {
        return result.branch.getLast(FlightDataType.TYPE_ROLL_RATE);
    }

    /**
     * Builds or reads the fin fixture for the regression assertions.
     *
     * @param rocket rocket model for this operation
     * @param name display name or lookup key
     * @return the resulting TabControlledTrapezoidFinSet
     */
    private TabControlledTrapezoidFinSet fin(Rocket rocket, String name) {
        return (TabControlledTrapezoidFinSet)
                rocket.getSelectedConfiguration().getActiveComponents().stream()
                        .filter(c -> c.getName().equals(name))
                        .findFirst()
                        .orElseThrow();
    }

    /**
     * Verifies that constant45 degrees produces roll with no airbrake output and does not change
     * design.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void constant45DegreesProducesRollWithNoAirbrakeOutputAndDoesNotChangeDesign()
            throws Exception {
        var positiveRocket = rocket(false);
        var positive = run(positiveRocket, "io.setRollDegrees(45);", false, 1.5);
        var negative = run(rocket(false), "io.setRollDegrees(-45);", false, 1.5);
        var neutral = run(rocket(false), "io.setRollDegrees(0);", false, 1.5);
        assertTrue(Math.abs(roll(positive)) > .1, "positive roll=" + roll(positive));
        assertTrue(
                roll(positive) * roll(negative) < 0, "commands must give opposite roll directions");
        assertTrue(Math.abs(roll(neutral)) < .01, "neutral roll=" + roll(neutral));
        assertEquals(Math.PI / 4, positive.branch.getLast(FlightComputerData.ROLL_TAB_ANGLE), 1e-9);
        assertEquals(
                0,
                fin(positiveRocket, "Aft fins").getTabAngle(),
                "Only the per-flight copy may change");
        System.out.println(
                "FC_ACTUATION roll +45="
                        + roll(positive)
                        + " -45="
                        + roll(negative)
                        + " neutral="
                        + roll(neutral)
                        + " rad/s at 1.5 s");
    }

    /**
     * Verifies that one millisecond program waits for twenty millisecond pwm before both physical
     * outputs move.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void oneMillisecondProgramWaitsForTwentyMillisecondPwmBeforeBothPhysicalOutputsMove()
            throws Exception {
        var result =
                run(
                        rocket(false),
                        "io.setRollDegrees(io.timeUs() >= 1005000 ? 45 : 0); io.setAirbrakes(io.timeUs() >= 1005000 ? 1 : 0);",
                        true,
                        .045);
        var time = result.branch.get(FlightDataType.TYPE_TIME);
        for (int i = 0; i < time.size(); i++) {
            double expected = time.get(i) < .02 - 1e-8 ? 0 : 1;
            assertEquals(
                    expected * Math.PI / 4,
                    result.branch.get(FlightComputerData.ROLL_TAB_ANGLE).get(i),
                    1e-8,
                    "roll t=" + time.get(i));
            assertEquals(
                    expected,
                    result.branch.get(FlightComputerData.AIRBRAKE_PHYSICAL).get(i),
                    1e-8,
                    "airbrakes t=" + time.get(i));
        }
    }

    /**
     * Verifies that named fin cans move independently and survive log import.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void namedFinCansMoveIndependentlyAndSurviveLogImport() throws Exception {
        var rocket = rocket(true);
        var forward =
                FlightComputerData.actuatorChannels(
                        "roll", fin(rocket, "Forward fins").getID().toString(), "Forward fins");
        var aft =
                FlightComputerData.actuatorChannels(
                        "roll", fin(rocket, "Aft fins").getID().toString(), "Aft fins");
        var result =
                run(
                        rocket,
                        "io.setRollDegrees(0); io.setRollDegrees(\"Forward fins\",45); io.setRollDegrees(\"Aft fins\",-15);",
                        false,
                        .1);
        assertEquals(Math.PI / 4, result.branch.getLast(forward.physical()), 1e-9);
        assertEquals(-Math.PI / 12, result.branch.getLast(aft.physical()), 1e-9);
        assertTrue(
                Double.isNaN(result.branch.getLast(FlightComputerData.ROLL_TAB_ANGLE)),
                "A mixed angle must not be reported as one angle");
        var imported = FlightComputerLogReader.read(result.log).get(0).data().getBranch(0);
        assertEquals(Math.PI / 4, imported.getLast(forward.physical()), 1e-9);
        assertEquals(-Math.PI / 12, imported.getLast(aft.command()), 1e-9);
        var empty = info.openrocket.core.document.OpenRocketDocumentFactory.createNewRocket();
        empty.addSimulation(
                new Simulation(
                        empty,
                        empty.getRocket(),
                        Simulation.Status.EXTERNAL,
                        "Actuator log",
                        new SimulationOptions(),
                        List.of(),
                        new FlightData(imported)));
        var storage = new info.openrocket.core.document.StorageOptions();
        storage.setSaveSimulationData(true);
        var saved = new java.io.ByteArrayOutputStream();
        new info.openrocket.core.file.openrocket.OpenRocketSaver()
                .save(
                        saved,
                        empty,
                        storage,
                        new info.openrocket.core.logging.WarningSet(),
                        new info.openrocket.core.logging.ErrorSet());
        assertTrue(saved.toString(java.nio.charset.StandardCharsets.UTF_8).contains("fctypes="));
        var context = new info.openrocket.core.file.DocumentLoadingContext();
        context.setOpenRocketDocument(
                info.openrocket.core.document.OpenRocketDocumentFactory.createEmptyRocket());
        context.setAttachmentFactory(
                new info.openrocket.core.file.FileSystemAttachmentFactory(temp.toFile()));
        new info.openrocket.core.file.openrocket.importt.OpenRocketLoader()
                .loadFromStream(
                        context,
                        new java.io.ByteArrayInputStream(saved.toByteArray()),
                        "named.ork");
        var reopened =
                context.getOpenRocketDocument().getSimulation(0).getSimulatedData().getBranch(0);
        assertEquals(imported.get(forward.physical()), reopened.get(forward.physical()));
        var restored =
                Arrays.stream(reopened.getTypes())
                        .filter(t -> t.getSymbol().equals(aft.command().getSymbol()))
                        .findFirst()
                        .orElseThrow();
        assertEquals(info.openrocket.core.unit.UnitGroup.UNITS_ANGLE, restored.getUnitGroup());
    }

    /**
     * Verifies that custom board airbrakes increase drag and lower the trajectory.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void customBoardAirbrakesIncreaseDragAndLowerTheTrajectory() throws Exception {
        var closed = run(RTFCVerificationRocket.rocket(), "io.setAirbrakes(0);", true, 8);
        var open = run(RTFCVerificationRocket.rocket(), "io.setAirbrakes(1);", true, 8);
        assertTrue(
                open.simulation.getSimulatedData().getMaxAltitude()
                        < closed.simulation.getSimulatedData().getMaxAltitude() - 1);
        assertEquals(1, open.branch.getLast(FlightComputerData.AIRBRAKE_PHYSICAL), 1e-9);
        System.out.println(
                "FC_ACTUATION airbrakes max altitude within 8 s: closed="
                        + closed.simulation.getSimulatedData().getMaxAltitude()
                        + " open="
                        + open.simulation.getSimulatedData().getMaxAltitude());
    }

    /**
     * Verifies that named airbrakes support several sets and broadcast resets overrides.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void namedAirbrakesSupportSeveralSetsAndBroadcastResetsOverrides() throws Exception {
        var rocket = RTFCVerificationRocket.rocket();
        var body =
                (BodyTube)
                        rocket.getSelectedConfiguration().getActiveComponents().stream()
                                .filter(c -> c instanceof BodyTube)
                                .findFirst()
                                .orElseThrow();
        var extra = new AirbrakeSet(.08, .04, .005, .04, .1, 4);
        extra.setName("Forward brakes");
        body.addChild(extra);
        extra.setFracExposed(0);
        var channel =
                FlightComputerData.actuatorChannels(
                        "airbrakes", extra.getID().toString(), extra.getName());
        var result =
                run(
                        rocket,
                        "io.setAirbrakes(0); if(io.timeUs()<1030000) io.setAirbrakes(\"Forward brakes\",0.5);",
                        true,
                        .06);
        assertTrue(
                result.branch.get(channel.physical()).stream().anyMatch(v -> v > .49 && v < .51));
        assertEquals(0, result.branch.getLast(channel.physical()), 1e-9);
        assertEquals(0, extra.getFracExposed(), 1e-9);
    }

    /**
     * Verifies that named target errors are actionable and disabled pwm holds last angle.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void namedTargetErrorsAreActionableAndDisabledPwmHoldsLastAngle() throws Exception {
        var rocket = rocket(true);
        var sim = RTFCVerificationRocket.simulation(rocket);
        var status =
                new SimulationStatus(
                        rocket.getSelectedConfiguration(),
                        sim.getOptions().toSimulationConditions());
        var bridge = new RTSimulationCommunicator(new Trace(s -> {}));
        bridge.bind(status, false, true);
        var targeted =
                new RTFC.Output(
                        0,
                        0,
                        0,
                        0,
                        0,
                        1500,
                        1500,
                        Map.of("Forward fins", 45f, "Aft fins", -15f),
                        Map.of());
        bridge.apply(targeted, false);
        bridge.apply(new RTFC.Output(0, 0, 0, 0, 0, 0, 0), false);
        assertEquals(Math.PI / 4, fin(rocket, "Forward fins").getTabAngle(), 1e-9);
        var bad = new RTFC.Output(0, 0, 0, 0, 0, 1500, 1500, Map.of("Typo", 45f), Map.of());
        assertTrue(
                assertThrows(IllegalArgumentException.class, () -> bridge.apply(bad, false))
                        .getMessage()
                        .contains("matches 0"));
        fin(rocket, "Aft fins").setName("Forward fins");
        var duplicate =
                new RTFC.Output(0, 0, 0, 0, 0, 1500, 1500, Map.of("Forward fins", 45f), Map.of());
        assertTrue(
                assertThrows(IllegalArgumentException.class, () -> bridge.apply(duplicate, false))
                        .getMessage()
                        .contains("matches 2"));
    }

    /** Verifies that separate java boards and servo nodes can drive separate named targets. */
    @Test
    void separateJavaBoardsAndServoNodesCanDriveSeparateNamedTargets() {
        String a =
                "import info.openrocket.core.simulation.flightcomputer.JavaBoardProgram; public class BoardProgram extends JavaBoardProgram { public void step(Context io) { io.setRollDegrees(\"Forward fins\",12); } }";
        String b = a.replace("Forward fins", "Aft fins").replace(",12", ",-7");
        var base = design(a, false);
        var board =
                Json.createObjectBuilder(FlightComputerModels.newNode("java_board", "", 0, 0))
                        .add("id", "second-board")
                        .add("program", b)
                        .build();
        var servo =
                Json.createObjectBuilder(FlightComputerModels.newNode("roll", "", 0, 0))
                        .add("id", "second-servo")
                        .build();
        var link =
                Json.createObjectBuilder()
                        .add("id", "second-roll-link")
                        .add("from", "second-servo")
                        .add("to", "cpu")
                        .add("port", "roll")
                        .add("bus", "PWM")
                        .add("resource", "TIM5")
                        .add("address", "pin-1");
        var design =
                base.with(
                                "nodes",
                                Json.createArrayBuilder(base.nodes()).add(board).add(servo).build())
                        .with(
                                "connections",
                                Json.createArrayBuilder(base.connections()).add(link).build());
        design.requireRunnable();
        JavaBoardProgram.approve(a);
        JavaBoardProgram.approve(b);
        try {
            var fc = new RTFC(new Trace(s -> {}));
            fc.configure(design);
            fc.init();
            fc.dispatchBoards(0);
            assertTrue(fc.getOutput().rollAngles().isEmpty(), "No named change before PWM");
            fc.latchPwm();
            assertEquals(Map.of("Forward fins", 12f, "Aft fins", -7f), fc.getOutput().rollAngles());
        } finally {
            JavaBoardProgram.revoke(a);
            JavaBoardProgram.revoke(b);
        }
    }

    /** Verifies that airbrake copies preserve names position and independent drag scaling. */
    @Test
    void airbrakeCopiesPreserveNamesPositionAndIndependentDragScaling() {
        var rocket = RTFCVerificationRocket.rocket();
        var brake =
                (AirbrakeSet)
                        rocket.getSelectedConfiguration().getActiveComponents().stream()
                                .filter(c -> c instanceof AirbrakeSet)
                                .findFirst()
                                .orElseThrow();
        var beforeRename = rocket.getSelectedConfiguration().getModID();
        brake.setName("My brakes");
        assertNotSame(beforeRename, rocket.getSelectedConfiguration().getModID());
        brake.setAxialOffset(.7);
        brake.setLocalOverrideCD(2);
        brake.setFracExposed(.5);
        var copy = (AirbrakeSet) rocket.copyWithOriginalID().findComponent(brake.getID());
        assertEquals(brake.getName(), copy.getName());
        assertEquals(brake.getPosition(), copy.getPosition());
        assertEquals(2, copy.getLocalOverrideCD());
        var conditions =
                new info.openrocket.core.aerodynamics.FlightConditions(
                        rocket.getSelectedConfiguration());
        conditions.setMach(.2);
        conditions.setAOA(0);
        var calc = new info.openrocket.core.aerodynamics.barrowman.AirbrakeSetCalc(brake);
        var warnings = new info.openrocket.core.logging.WarningSet();
        double drag = calc.calculatePressureCD(conditions, 0, 0, warnings);
        copy.setLocalOverrideCD(9);
        assertEquals(
                drag,
                calc.calculatePressureCD(conditions, 0, 0, warnings),
                1e-12,
                "Another set/run must not change this set's drag");
    }

    /**
     * Verifies that cached fin calculator reads updated tab angles and reports the forcing moment.
     */
    @Test
    void cachedFinCalculatorReadsUpdatedTabAnglesAndReportsTheForcingMoment() {
        var rocket = rocket(false);
        var fin = fin(rocket, "Aft fins");
        var calc = new info.openrocket.core.aerodynamics.barrowman.FinSetCalc(fin);
        var conditions =
                new info.openrocket.core.aerodynamics.FlightConditions(
                        rocket.getSelectedConfiguration());
        conditions.setMach(.2);
        conditions.setAOA(0);
        var forces = new info.openrocket.core.aerodynamics.AerodynamicForces();
        for (double angle : new double[] {45, 0, -45}) {
            fin.setTabAngle(Math.toRadians(angle));
            forces.zero();
            calc.calculateNonaxialForces(
                    conditions,
                    info.openrocket.core.util.Transformation.IDENTITY,
                    forces,
                    new info.openrocket.core.logging.WarningSet());
            assertEquals(Math.signum(angle), Math.signum(forces.getCroll()), "angle=" + angle);
            assertEquals(forces.getCroll(), forces.getCrollForce() - forces.getCrollDamp(), 1e-10);
        }
    }
}
