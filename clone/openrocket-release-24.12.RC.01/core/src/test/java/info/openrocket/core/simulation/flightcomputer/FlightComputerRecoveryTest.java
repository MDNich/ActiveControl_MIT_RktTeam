package info.openrocket.core.simulation.flightcomputer;

import edu.mit.rocket_team.RTFCVerificationRocket;
import edu.mit.rocket_team.FC.FlightComputerTimingSettings;
import edu.mit.rocket_team.telemetry.*;
import edu.mit.rocket_team.control.RTPyroController;
import edu.mit.rocket_team.util.RTUtilLibrary.Trace;
import info.openrocket.core.document.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.simulation.listeners.*;
import info.openrocket.core.util.BaseTestCase;
import jakarta.json.*;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import java.nio.file.*;
import java.util.*;
import static org.junit.jupiter.api.Assertions.*;

class FlightComputerRecoveryTest extends BaseTestCase {
    @TempDir Path temp;
    private String telemetry;
    private boolean recoveryHardware = true;

    /** Initializes application services and isolated state for this regression fixture. */
    @BeforeEach
    void setup() {
        telemetry = System.getProperty("openrocket.fc.telemetryDir");
        System.setProperty("openrocket.fc.telemetryDir", temp.toString());
    }

    /** Releases test resources and restores state changed by the fixture. */
    @AfterEach
    void cleanup() {
        FlightComputerDesignTest.set("openrocket.fc.telemetryDir", telemetry);
    }

    /**
     * Builds or reads the chute fixture for the regression assertions.
     *
     * @param rocket rocket model for this operation
     * @return the resulting Parachute
     */
    private Parachute chute(Rocket rocket) {
        return (Parachute)
                rocket.getSelectedConfiguration().getActiveComponents().stream()
                        .filter(c -> c instanceof Parachute)
                        .findFirst()
                        .orElseThrow();
    }

    /**
     * Builds or reads the settings fixture for the regression assertions.
     *
     * @param device device
     * @param delay delay
     * @param channels channels
     * @return the resulting FlightComputerRecoverySettings
     */
    private FlightComputerRecoverySettings settings(
            RecoveryDevice device, long delay, Integer... channels) {
        return new FlightComputerRecoverySettings(
                List.of(
                        new FlightComputerRecoverySettings.Binding(
                                device.getID(), Set.of(channels), delay)));
    }

    /**
     * Builds or reads the design fixture for the regression assertions.
     *
     * @param source source text, model or measured-channel selection
     * @param cost cost
     * @return the resulting FlightComputerDesign
     */
    private FlightComputerDesign design(String source, long cost) {
        var d = FlightComputerDesign.zephyrus();
        var node =
                Json.createObjectBuilder(FlightComputerModels.newNode("java_board", "", 0, 0))
                        .add("program", source)
                        .add(
                                "properties",
                                Json.createObjectBuilder()
                                        .add("periodUs", 1000)
                                        .add("executionUs", cost))
                        .build();
        var nodes = Json.createArrayBuilder();
        var links = Json.createArrayBuilder();
        for (var n : d.nodes().getValuesAs(JsonObject.class))
            if (recoveryHardware || !n.getString("type").equals("recovery")) nodes.add(n);
        for (var link : d.connections().getValuesAs(JsonObject.class))
            if (recoveryHardware || !link.getString("port").equals("recovery")) links.add(link);
        return d.with("nodes", nodes.add(node).build()).with("connections", links.build());
    }

    private record Result(Simulation sim, FlightDataBranch branch, Path log) {}

    /**
     * Runs the configured test simulation and returns its flight results.
     *
     * @param rocket rocket model for this operation
     * @param settings validated configuration for this operation
     * @param code code
     * @param cost cost
     * @param duration duration
     * @param phase phase
     * @param loss loss
     * @param basic basic
     * @param extra extra
     * @return the resulting Result
     * @throws Exception if the requested operation cannot be completed
     */
    private Result run(
            Rocket rocket,
            FlightComputerRecoverySettings settings,
            String code,
            long cost,
            double duration,
            int phase,
            int loss,
            boolean basic,
            SimulationListener... extra)
            throws Exception {
        String source =
                "import info.openrocket.core.simulation.flightcomputer.JavaBoardProgram; public class BoardProgram extends JavaBoardProgram { boolean first,second; public void step(Context io) { "
                        + code
                        + " } }";
        var design = design(source, cost);
        JavaBoardProgram.approve(source);
        try {
            var fc =
                    new FlightControllerSimulatorListener(
                                    s -> {},
                                    .0025,
                                    false,
                                    new TelemetryLinkSettings(loss, 0, 1),
                                    FlightComputerOutputSettings.DEFAULT,
                                    new FlightComputerTimingSettings(3000, 0, 0, phase, 1))
                            .withDesign(
                                    new FlightComputerLibrary.Resolved(
                                            design, temp.resolve("recovery.fc"), "test"))
                            .withRecovery(settings);
            var sim = RTFCVerificationRocket.simulation(rocket);
            sim.getOptions().setMaxSimulationTime(duration);
            if (basic) {
                var conditions = sim.getOptions().toSimulationConditions();
                conditions.setSimulation(sim);
                conditions.getSimulationListenerList().add(fc);
                conditions.getSimulationListenerList().addAll(List.of(extra));
                var engine = new BasicEventSimulationEngine();
                engine.simulate(conditions);
                return new Result(
                        sim,
                        engine.getFlightData().getBranch(0),
                        fc.getFlightComputer().telemetry.getLogPath());
            }
            var listeners = new ArrayList<SimulationListener>();
            listeners.add(fc);
            listeners.addAll(List.of(extra));
            sim.simulate(listeners.toArray(SimulationListener[]::new));
            return new Result(
                    sim,
                    sim.getSimulatedData().getBranch(0),
                    fc.getFlightComputer().telemetry.getLogPath());
        } finally {
            JavaBoardProgram.revoke(source);
        }
    }

    /**
     * Builds or reads the deployments fixture for the regression assertions.
     *
     * @param b b
     * @return the resulting List&lt;FlightEvent&gt;
     */
    private List<FlightEvent> deployments(FlightDataBranch b) {
        return b.getEvents().stream()
                .filter(e -> e.getType() == FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT)
                .toList();
    }

    /**
     * Verifies that commands use completion time and sub millisecond delay independent of pwm and
     * downlink.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void commandsUseCompletionTimeAndSubMillisecondDelayIndependentOfPwmAndDownlink()
            throws Exception {
        for (int phase : new int[] {0, 6371}) {
            var rocket = RTFCVerificationRocket.rocket();
            var chute = chute(rocket);
            var result =
                    run(
                            rocket,
                            settings(chute, 250, 2),
                            "if(!first && io.timeUs()>=6000000){io.fireRecovery(2);first=true;}",
                            250,
                            5.5,
                            phase,
                            1,
                            false);
            var events = deployments(result.branch);
            assertEquals(1, events.size());
            assertEquals(5.0005, events.get(0).getTime(), 1e-9);
            var detail = (RecoveryDeployment) events.get(0).getData();
            assertEquals(5_000_250, detail.commandUs());
            assertEquals(5_000_500, detail.dueUs());
            assertTrue(detail.origin().startsWith("java_board:"));
            assertEquals(
                    1,
                    result.branch.getLast(
                            FlightComputerData.recoveryDevice(
                                    chute.getID().toString(), chute.getName())));
            assertEquals(
                    DeploymentConfiguration.DeployEvent.APOGEE,
                    chute.getDeploymentConfigurations().getDefault().getDeployEvent());
            assertTrue(Files.readString(result.log).contains("action=recovery.deploy"));
        }
    }

    /**
     * Verifies that zero delay acts without waiting for the next tick in both engines.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void zeroDelayActsWithoutWaitingForTheNextTickInBothEngines() throws Exception {
        for (boolean basic : new boolean[] {false, true}) {
            var rocket = RTFCVerificationRocket.rocket();
            var result =
                    run(
                            rocket,
                            settings(chute(rocket), 0, 2),
                            "if(!first && io.timeUs()>=6000000){io.fireRecovery(2);first=true;}",
                            345,
                            5.02,
                            0,
                            0,
                            basic);
            assertEquals(5.000345, deployments(result.branch).get(0).getTime(), 1e-9);
        }
    }

    /**
     * Verifies that automatic rules are overridden but disabling fc restores the saved rule.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void automaticRulesAreOverriddenButDisablingFcRestoresTheSavedRule() throws Exception {
        var rocket = RTFCVerificationRocket.rocket();
        var chute = chute(rocket);
        chute.getDeploymentConfigurations()
                .getDefault()
                .setDeployEvent(DeploymentConfiguration.DeployEvent.LAUNCH);
        chute.getDeploymentConfigurations().getDefault().setDeployDelay(5);
        var fc = run(rocket, FlightComputerRecoverySettings.EMPTY, "", 0, 5.1, 0, 0, false);
        assertTrue(deployments(fc.branch).isEmpty());
        var normal = RTFCVerificationRocket.simulation(rocket);
        normal.getOptions().setMaxSimulationTime(5.1);
        normal.simulate();
        assertEquals(5, deployments(normal.getSimulatedData().getBranch(0)).get(0).getTime(), 1e-9);
    }

    /**
     * Verifies that drogue then main continue the fc clock and change descent.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void drogueThenMainContinueTheFcClockAndChangeDescent() throws Exception {
        var rocket = RTFCVerificationRocket.rocket();
        var drogue = chute(rocket);
        drogue.setName("Drogue");
        drogue.setDiameter(.4);
        var main = new Parachute();
        main.setName("Main");
        main.setDiameter(1.2);
        drogue.getParent().addChild(main);
        var settings =
                new FlightComputerRecoverySettings(
                        List.of(
                                new FlightComputerRecoverySettings.Binding(
                                        drogue.getID(), Set.of(0, 1), 0),
                                new FlightComputerRecoverySettings.Binding(
                                        main.getID(), Set.of(2), 250)));
        String code =
                "if(!first && io.timeUs()>=9000000){io.fireRecovery(0);io.fireRecovery(1);first=true;} if(!second && io.timeUs()>=19000000){io.fireRecovery(2);second=true;}";
        var result = run(rocket, settings, code, 0, 20, 0, 0, false);
        var events = deployments(result.branch);
        assertEquals(2, events.size());
        assertEquals(8, events.get(0).getTime(), 1e-9);
        assertEquals(18.00025, events.get(1).getTime(), 1e-9);
        int mainIndex =
                java.util.stream.IntStream.range(0, result.branch.getLength())
                        .filter(i -> result.branch.get(FlightDataType.TYPE_TIME).get(i) >= 18)
                        .findFirst()
                        .orElseThrow();
        assertTrue(
                result.branch.get(FlightDataType.TYPE_VELOCITY_Z).get(mainIndex) < 0,
                "Main must be tested during actual descent");
        var ballistic = run(rocket, FlightComputerRecoverySettings.EMPTY, code, 0, 20, 0, 0, false);
        assertTrue(
                Math.abs(
                                result.branch.getLast(FlightDataType.TYPE_VELOCITY_Z)
                                        - ballistic.branch.getLast(FlightDataType.TYPE_VELOCITY_Z))
                        > 1);
        var imported = FlightComputerLogReader.read(result.log).get(0).data().getBranch(0);
        assertEquals(2, deployments(imported).size());
        assertEquals(
                "Main", ((RecoveryDeployment) deployments(imported).get(1).getData()).deviceName());
        var channel = FlightComputerData.recoveryDevice(main.getID().toString(), main.getName());
        var times = imported.get(FlightDataType.TYPE_TIME);
        int index = times.indexOf(18.00025);
        assertTrue(index >= 0);
        assertEquals(1, imported.get(channel).get(index));
        assertEquals(0, imported.get(channel).get(index - 1));
        System.out.println(
                "FC_RECOVERY drogue=8.0 main=18.00025 imported_main="
                        + times.get(index)
                        + " velocity="
                        + result.branch.getLast(FlightDataType.TYPE_VELOCITY_Z));
        roundTrip(result.sim, imported, settings, channel);
    }

    /**
     * Builds or reads the round trip fixture for the regression assertions.
     *
     * @param original original
     * @param imported imported
     * @param settings validated configuration for this operation
     * @param channel logical recovery channel, numbered 0 through 5
     * @throws Exception if the requested operation cannot be completed
     */
    private void roundTrip(
            Simulation original,
            FlightDataBranch imported,
            FlightComputerRecoverySettings settings,
            FlightDataType channel)
            throws Exception {
        var doc = OpenRocketDocumentFactory.createDocumentFromRocket(original.getRocket());
        var sim =
                new Simulation(
                        doc,
                        doc.getRocket(),
                        Simulation.Status.EXTERNAL,
                        "Imported recovery",
                        original.getOptions().clone(),
                        List.of(),
                        new FlightData(imported));
        doc.addSimulation(sim);
        ZephyrusFlightComputer.applyRecovery(sim, settings);
        var storage = new StorageOptions();
        storage.setSaveSimulationData(true);
        var bytes = new java.io.ByteArrayOutputStream();
        new info.openrocket.core.file.openrocket.OpenRocketSaver()
                .save(
                        bytes,
                        doc,
                        storage,
                        new info.openrocket.core.logging.WarningSet(),
                        new info.openrocket.core.logging.ErrorSet());
        var context = new info.openrocket.core.file.DocumentLoadingContext();
        context.setOpenRocketDocument(OpenRocketDocumentFactory.createEmptyRocket());
        context.setMotorFinder(
                (type, manufacturer, designation, diameter, length, digest, warnings) -> null);
        context.setAttachmentFactory(
                new info.openrocket.core.file.FileSystemAttachmentFactory(temp.toFile()));
        new info.openrocket.core.file.openrocket.importt.OpenRocketLoader()
                .loadFromStream(
                        context,
                        new java.io.ByteArrayInputStream(bytes.toByteArray()),
                        "recovery.ork");
        var reopened = context.getOpenRocketDocument().getSimulation(0);
        assertEquals(settings, ZephyrusFlightComputer.read(reopened).getRecoverySettings());
        var branch = reopened.getSimulatedData().getBranch(0);
        assertEquals(imported.get(channel), branch.get(channel));
        assertEquals(deployments(imported).get(1).getData(), deployments(branch).get(1).getData());
    }

    /**
     * Verifies that bridge rejects unarmed warmup unknown and after landing and deduplicates.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void bridgeRejectsUnarmedWarmupUnknownAndAfterLandingAndDeduplicates() throws Exception {
        var rocket = RTFCVerificationRocket.rocket();
        var chute = chute(rocket);
        var sim = RTFCVerificationRocket.simulation(rocket);
        var status =
                new SimulationStatus(
                        rocket.getSelectedConfiguration(),
                        sim.getOptions().toSimulationConditions());
        var trace = new Trace(s -> {});
        var pyro = new RTPyroController(trace);
        try (var bridge =
                new FlightComputerRecoveryBridge(
                        status, settings(chute, 0, 0, 1), pyro, trace, 1_000_000)) {
            trace.time(500_000);
            pyro.armPyro(0);
            pyro.firePyro(0);
            bridge.drain(status);
            assertTrue(status.getEventQueue().isEmpty());
            trace.time(2_000_000);
            pyro.firePyro(1);
            bridge.drain(status);
            assertTrue(status.getEventQueue().isEmpty());
            pyro.armPyro(0);
            pyro.firePyro(0);
            pyro.armPyro(1);
            pyro.firePyro(1);
            bridge.drain(status);
            assertEquals(1, status.getEventQueue().size());
            var authorized = status.getEventQueue().poll();
            assertTrue(bridge.allow(status, authorized));
            var forged =
                    new FlightEvent(
                            FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT,
                            1,
                            chute,
                            authorized.getData());
            assertFalse(bridge.allow(status, forged));
            status.addEvent(new FlightEvent(FlightEvent.Type.GROUND_HIT, 1));
            assertFalse(bridge.allow(status, authorized));
            status.setLanded(true);
            pyro.armPyro(0);
            pyro.firePyro(0);
            bridge.drain(status);
            assertEquals(1, status.getEventQueue().size());
        }
    }

    /**
     * Verifies that unowned events and listener veto never show physical deployment.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void unownedEventsAndListenerVetoNeverShowPhysicalDeployment() throws Exception {
        var rocket = RTFCVerificationRocket.rocket();
        var chute = chute(rocket);
        var inject =
                new AbstractSimulationListener() {
                    /**
                     * Builds or reads the start simulation fixture for the regression assertions.
                     *
                     * @param status stat in microseconds
                     * @throws info.openrocket.core.simulation.exception.SimulationException if the
                     *     requested operation cannot be completed
                     */
                    public void startSimulation(SimulationStatus status)
                            throws info.openrocket.core.simulation.exception.SimulationException {
                        status.addEvent(
                                new FlightEvent(
                                        FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT,
                                        .05,
                                        status.getConfiguration()
                                                .getRocket()
                                                .findComponent(chute.getID())));
                    }
                };
        assertTrue(
                deployments(
                                run(
                                                rocket,
                                                FlightComputerRecoverySettings.EMPTY,
                                                "",
                                                0,
                                                .1,
                                                0,
                                                0,
                                                false,
                                                inject)
                                        .branch)
                        .isEmpty());
        var veto =
                new AbstractSimulationListener() {
                    /**
                     * Builds or reads the recovery device deployment fixture for the regression
                     * assertions.
                     *
                     * @param status stat in microseconds
                     * @param device device
                     * @return true when the condition described above holds
                     */
                    public boolean recoveryDeviceDeployment(
                            SimulationStatus status, RecoveryDevice device) {
                        return false;
                    }
                };
        var result =
                run(
                        rocket,
                        settings(chute, 0, 2),
                        "if(!first && io.timeUs()>=6000000){io.fireRecovery(2);first=true;}",
                        0,
                        5.1,
                        0,
                        0,
                        false,
                        veto);
        assertTrue(deployments(result.branch).isEmpty());
        assertFalse(Files.readString(result.log).contains("action=recovery.deploy"));
    }

    /** Verifies that mappings are validated and renames keep their identity. */
    @Test
    void mappingsAreValidatedAndRenamesKeepTheirIdentity() {
        var rocket = RTFCVerificationRocket.rocket();
        var chute = chute(rocket);
        var map = settings(chute, 250, 0, 1);
        var config = new info.openrocket.core.util.Config();
        map.write(config);
        assertEquals(map, FlightComputerRecoverySettings.read(config));
        assertEquals(251, FlightComputerRecoverySettings.delayFromMilliseconds(.2506));
        assertThrows(
                IllegalArgumentException.class,
                () -> FlightComputerRecoverySettings.delayFromMilliseconds(Double.NaN));
        chute.setName("Renamed");
        var sim = RTFCVerificationRocket.simulation(rocket);
        var status =
                new SimulationStatus(
                        rocket.getSelectedConfiguration(),
                        sim.getOptions().toSimulationConditions());
        try (var bridge =
                new FlightComputerRecoveryBridge(
                        status, map, new RTPyroController(), new Trace(s -> {}), 0)) {
            assertEquals("Renamed", bridge.metadata().getJsonObject(0).getString("name"));
        }
        chute.getParent().removeChild(chute);
        assertThrows(
                IllegalArgumentException.class,
                () ->
                        new FlightComputerRecoveryBridge(
                                status, map, new RTPyroController(), new Trace(s -> {}), 0));
    }

    /**
     * Verifies that no recovery hardware discards commands even with assignments.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void noRecoveryHardwareDiscardsCommandsEvenWithAssignments() throws Exception {
        recoveryHardware = false;
        var rocket = RTFCVerificationRocket.rocket();
        var result =
                run(
                        rocket,
                        settings(chute(rocket), 0, 2),
                        "io.fireRecovery(2);",
                        0,
                        .03,
                        0,
                        0,
                        false);
        assertTrue(deployments(result.branch).isEmpty());
        assertTrue(Files.readString(result.log).contains("reason=no_component"));
    }

    /**
     * Verifies that one channel can deploy several devices and keeps separate delays.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void oneChannelCanDeploySeveralDevicesAndKeepsSeparateDelays() throws Exception {
        var rocket = RTFCVerificationRocket.rocket();
        var first = chute(rocket);
        var second = new Streamer();
        second.setName("Streamer");
        first.getParent().addChild(second);
        var map =
                new FlightComputerRecoverySettings(
                        List.of(
                                new FlightComputerRecoverySettings.Binding(
                                        first.getID(), Set.of(2), 0),
                                new FlightComputerRecoverySettings.Binding(
                                        second.getID(), Set.of(2), 375)));
        var result =
                run(
                        rocket,
                        map,
                        "if(!first && io.timeUs()>=6000000){io.fireRecovery(2);first=true;}",
                        0,
                        5.1,
                        0,
                        0,
                        false);
        var events = deployments(result.branch);
        assertEquals(2, events.size());
        assertEquals(5, events.get(0).getTime(), 1e-9);
        assertEquals(5.000375, events.get(1).getTime(), 1e-9);
        assertEquals(second.getID(), events.get(1).getSource().getID());
    }

    /**
     * Verifies that premature recovery keeps the existing under thrust abort.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void prematureRecoveryKeepsTheExistingUnderThrustAbort() throws Exception {
        var rocket = RTFCVerificationRocket.rocket();
        var result =
                run(
                        rocket,
                        settings(chute(rocket), 0, 2),
                        "if(!first && io.timeUs()>=1100000){io.fireRecovery(2);first=true;}",
                        0,
                        .2,
                        0,
                        0,
                        false);
        var abort = result.branch.getFirstEvent(FlightEvent.Type.SIM_ABORT);
        assertNotNull(abort);
        assertNotNull(
                FlightComputerLogReader.read(result.log)
                        .get(0)
                        .data()
                        .getBranch(0)
                        .getFirstEvent(FlightEvent.Type.SIM_ABORT));
        assertEquals(
                info.openrocket.core.logging.SimulationAbort.Cause.DEPLOY_UNDER_THRUST,
                ((info.openrocket.core.logging.SimulationAbort) abort.getData()).getCause());
    }
}
