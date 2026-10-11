import com.google.inject.*;
import info.openrocket.core.document.*;
import info.openrocket.core.plugin.PluginModule;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.simulation.flightcomputer.*;
import info.openrocket.core.startup.Application;
import info.openrocket.core.util.TestRockets;
import info.openrocket.swing.startup.GuiModule;
import info.openrocket.swing.gui.flightcomputer.*;
import info.openrocket.swing.gui.simulation.SimulationConfigDialog;
import jakarta.json.*;
import javax.swing.*;
import java.awt.*;
import java.nio.file.*;
import java.util.*;

/** Native UI, repeated flights and controller-result import using only the packaged JAR. */
public class ControllerDesktopVerification {
    /**
     * Runs the controller desktop verification entry point, reporting verification failures to the
     * invoking process.
     *
     * @param args command-line arguments
     */
    public static void main(String[] args) {
        try {
            verify(Path.of(args[0]).toAbsolutePath());
            System.exit(0);
        } catch (Throwable e) {
            e.printStackTrace();
            System.exit(1);
        }
    }

    /**
     * Runs the packaged-application integration assertions and writes verification artifacts.
     *
     * @param output output snapshot or destination
     * @throws Exception if the requested operation cannot be completed
     * @throws AssertionError when rejected: Missing controller plot channels
     */
    static void verify(Path output) throws Exception {
        Files.createDirectories(output);
        System.setProperty("openrocket.bypass.presets", "true");
        System.setProperty("openrocket.bypass.motors", "true");
        System.setProperty("openrocket.fc.library", output.resolve("library").toString());
        System.setProperty("openrocket.fc.telemetryDir", output.resolve("telemetry").toString());
        var module = new GuiModule();
        Application.setInjector(Guice.createInjector(module, new PluginModule()));
        module.startLoader();
        SwingUtilities.invokeAndWait(info.openrocket.swing.gui.util.GUIUtil::applyLAF);
        var base = FlightComputerDesign.zephyrus();
        var nodes = Json.createArrayBuilder();
        var links = Json.createArrayBuilder();
        for (var node : base.nodes().getValuesAs(JsonObject.class))
            if (!node.getString("type").equals("airbrakes")) nodes.add(node);
        for (var link : base.connections().getValuesAs(JsonObject.class))
            if (!link.getString("port").equals("airbrakes")) links.add(link);
        base = base.with("nodes", nodes.build()).with("connections", links.build());
        var b = FlightComputerController.create(base);
        b =
                b.with(
                        Json.createObjectBuilder(b.json())
                                .add("enabled", true)
                                .add("name", "Roll damping")
                                .add("kp", .025)
                                .add("periodUs", 1000)
                                .add("executionUs", 1500)
                                .build());
        var design = FlightComputerController.replace(base, java.util.List.of(b));
        var file = output.resolve("controller-demo.fc");
        design.write(file);
        var rocket = TestRockets.makeEstesAlphaIII();
        var original =
                (TrapezoidFinSet)
                        rocket.getSelectedConfiguration().getActiveComponents().stream()
                                .filter(x -> x instanceof TrapezoidFinSet)
                                .findFirst()
                                .orElseThrow();
        var fin = new TabControlledTrapezoidFinSet(original);
        fin.setName("Roll control fins");
        fin.setTabSpan(.02);
        fin.setTabChord(.01);
        fin.setTabOffset(.01);
        fin.setCNALPHA(2);
        var doc = OpenRocketDocumentFactory.createDocumentFromRocket(rocket);
        var sim = new Simulation(doc, rocket);
        doc.addSimulation(sim);
        sim.setFlightConfigurationId(TestRockets.TEST_FCID_1);
        sim.setName("Controller blocks — packaged verification");
        sim.getOptions().setMaxSimulationTime(1.4);
        sim.getOptions().setInitialRollRate(3);
        sim.getOptions().setWindSpeedAverage(0);
        sim.getOptions().setWindSpeedDeviation(0);
        ZephyrusFlightComputer.selectFile(sim, file);
        ZephyrusFlightComputer.applyControllers(
                sim,
                new FlightComputerControllerSettings(
                        design.id(),
                        java.util.List.of(
                                new FlightComputerControllerSettings.Binding(
                                        b.id(), fin.getID(), 1, 0, -15, 15, false))));
        ZephyrusFlightComputer.apply(
                sim, true, edu.mit.rocket_team.telemetry.TelemetryLinkSettings.DEFAULT);
        SwingUtilities.invokeAndWait(
                () -> {
                    try {
                        var editor = new FlightComputerDesigner(file, sim, p -> {});
                        var dialog =
                                new JDialog(
                                        (Frame) null, "Controller blocks — verification", false);
                        dialog.setContentPane(editor);
                        dialog.setSize(1280, 850);
                        dialog.setVisible(true);
                        try {
                            var tabs = find(editor, JTabbedPane.class, null);
                            tabs.setSelectedIndex(1);
                            var controller =
                                    find(
                                            editor,
                                            FlightComputerControllerPanel.class,
                                            "fc.controllers");
                            var properties = find(controller, JTabbedPane.class, null);
                            for (int i = 0; i < 3; i++) {
                                properties.setSelectedIndex(i);
                                dialog.validate();
                                render(
                                        dialog.getRootPane(),
                                        output.resolve("packaged-controller-" + i + ".png"));
                            }
                        } finally {
                            dialog.dispose();
                        }
                        var options = new SimulationConfigDialog(null, doc, false, sim);
                        try {
                            options.setModalityType(Dialog.ModalityType.MODELESS);
                            options.setSize(1100, 860);
                            options.setVisible(true);
                            var tabs = find(options, JTabbedPane.class, null);
                            tabs.setSelectedIndex(tabs.indexOfTab("Flight computer"));
                            options.validate();
                            render(
                                    options.getRootPane(),
                                    output.resolve("packaged-simulation.png"));
                        } finally {
                            options.dispose();
                        }
                    } catch (Exception e) {
                        throw new RuntimeException(e);
                    }
                });
        var paths = new HashSet<Path>();
        var rolls = new ArrayList<Double>();
        var channels = FlightComputerControllerPlots.channels(design, b);
        for (int i = 0; i < 2; i++) {
            sim.simulate();
            var branch = sim.getSimulatedData().getBranch(0);
            if (branch.get(channels.get("latched")) == null)
                throw new AssertionError("Missing controller plot channels");
            rolls.add(branch.getLast(FlightDataType.TYPE_ROLL_RATE));
            paths.add(sim.getFlightComputerLogPath());
            var imported =
                    FlightComputerLogReader.read(sim.getFlightComputerLogPath())
                            .get(0)
                            .data()
                            .getBranch(0);
            if (imported.get(channels.get("latched")) == null)
                throw new AssertionError("Controller log import lost channels");
        }
        if (paths.size() != 2 || Math.abs(rolls.get(0) - rolls.get(1)) > 1e-10)
            throw new AssertionError("Run isolation/reproducibility failed");
        if (fin.getTabAngle() != 0) throw new AssertionError("Original fin mutated");
        var storage = new StorageOptions();
        storage.setSaveSimulationData(true);
        try (var stream = Files.newOutputStream(output.resolve("controller-demo.ork"))) {
            new info.openrocket.core.file.openrocket.OpenRocketSaver()
                    .save(
                            stream,
                            doc,
                            storage,
                            new info.openrocket.core.logging.WarningSet(),
                            new info.openrocket.core.logging.ErrorSet());
        }
        Files.writeString(
                output.resolve("summary.txt"),
                "Packaged designer and Flight computer tab: passed\nIndependent repeat runs and log import: passed\nPeriod: 1000 us; execution cost: 1500 us; PWM: 20000 us\nFinal roll rates (rad/s): "
                        + rolls
                        + "\nOriginal fin unchanged: passed\n");
    }

    /**
     * Renders the verification view to an image artifact.
     *
     * @param c selected channel or controller
     * @param path source or destination filesystem path
     * @throws Exception if the requested operation cannot be completed
     */
    static void render(JComponent c, Path path) throws Exception {
        var image =
                new java.awt.image.BufferedImage(
                        c.getWidth(), c.getHeight(), java.awt.image.BufferedImage.TYPE_INT_RGB);
        var g = image.createGraphics();
        c.printAll(g);
        g.dispose();
        javax.imageio.ImageIO.write(image, "png", path.toFile());
    }

    /**
     * Finds the Swing component required by the verification scenario.
     *
     * @param <T> t type used by this operation
     * @param c selected channel or controller
     * @param type type
     * @param name display name or lookup key
     * @return the resulting T
     */
    static <T extends Component> T find(Container c, Class<T> type, String name) {
        for (var child : c.getComponents()) {
            if (type.isInstance(child) && (name == null || name.equals(child.getName())))
                return type.cast(child);
            if (child instanceof Container n) {
                T found = find(n, type, name);
                if (found != null) return found;
            }
        }
        return null;
    }
}
