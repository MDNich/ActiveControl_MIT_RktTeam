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
import info.openrocket.swing.gui.simulation.SimulationConfigDialog;
import jakarta.json.*;
import javax.swing.*;
import java.awt.*;
import java.nio.file.*;
import java.util.*;

/** Native dialog, repeated flights and log import using only the packaged application JAR. */
public class RecoveryDesktopVerification {
    /**
     * Runs the recovery desktop verification entry point, reporting verification failures to the
     * invoking process.
     *
     * @param args command-line arguments
     */
    public static void main(String[] args) {
        try {
            verify(Path.of(args[0]).toAbsolutePath());
            System.exit(0);
        } catch (Throwable error) {
            error.printStackTrace();
            System.exit(1);
        }
    }

    /**
     * Runs the packaged-application integration assertions and writes verification artifacts.
     *
     * @param output output snapshot or destination
     * @throws Exception if the requested operation cannot be completed
     * @throws AssertionError when rejected: Expected one recovery device
     */
    private static void verify(Path output) throws Exception {
        Files.createDirectories(output);
        System.setProperty("openrocket.bypass.presets", "true");
        System.setProperty("openrocket.bypass.motors", "true");
        System.setProperty("openrocket.fc.library", output.resolve("library").toString());
        System.setProperty("openrocket.fc.telemetryDir", output.resolve("telemetry").toString());
        var module = new GuiModule();
        Application.setInjector(Guice.createInjector(module, new PluginModule()));
        module.startLoader();
        String code =
                "import info.openrocket.core.simulation.flightcomputer.JavaBoardProgram; public class BoardProgram extends JavaBoardProgram { boolean fired; public void step(Context io) { if(!fired && io.timeUs()>=5000000){io.fireRecovery(2);fired=true;} } }";
        var design = FlightComputerDesign.zephyrus();
        var board =
                Json.createObjectBuilder(FlightComputerModels.newNode("java_board", "", 0, 0))
                        .add("program", code)
                        .add(
                                "properties",
                                Json.createObjectBuilder()
                                        .add("periodUs", 1000)
                                        .add("executionUs", 0));
        var nodes = Json.createArrayBuilder();
        var links = Json.createArrayBuilder();
        for (var n : design.nodes().getValuesAs(JsonObject.class))
            if (!Set.of("airbrakes", "roll").contains(n.getString("type"))) nodes.add(n);
        for (var link : design.connections().getValuesAs(JsonObject.class))
            if (!Set.of("airbrakes", "roll").contains(link.getString("port"))) links.add(link);
        design = design.with("nodes", nodes.add(board).build()).with("connections", links.build());
        Path file = output.resolve("verification.fc");
        design.write(file);
        JavaBoardProgram.approve(code);
        try {
            var rocket = TestRockets.makeEstesAlphaIII();
            var doc = OpenRocketDocumentFactory.createDocumentFromRocket(rocket);
            var sim = new Simulation(doc, rocket);
            sim.setFlightConfigurationId(TestRockets.TEST_FCID_1);
            sim.setName("Packaged recovery verification");
            doc.addSimulation(sim);
            sim.getOptions().setWindSpeedAverage(0);
            sim.getOptions().setWindSpeedDeviation(0);
            sim.getOptions().setMaxSimulationTime(5.1);
            ZephyrusFlightComputer.selectFile(sim, file);
            SwingUtilities.invokeAndWait(
                    () -> {
                        var dialog = new SimulationConfigDialog(null, doc, false, sim);
                        try {
                            dialog.setModalityType(Dialog.ModalityType.MODELESS);
                            dialog.setSize(1080, 870);
                            dialog.setVisible(true);
                            var tabs = find(dialog, JTabbedPane.class, null);
                            tabs.setSelectedIndex(tabs.indexOfTab("Flight computer"));
                            find(dialog, JCheckBox.class, "fc.enabled").doClick();
                            var table = find(dialog, JTable.class, "fc.recovery.mappings");
                            if (table.getRowCount() != 1)
                                throw new AssertionError("Expected one recovery device");
                            table.getModel().setValueAt(true, 0, 3);
                            table.getModel().setValueAt(.375, 0, 7);
                            table.scrollRectToVisible(table.getCellRect(0, 0, true));
                            dialog.validate();
                            var image =
                                    new java.awt.image.BufferedImage(
                                            dialog.getWidth(),
                                            dialog.getHeight(),
                                            java.awt.image.BufferedImage.TYPE_INT_RGB);
                            var g = image.createGraphics();
                            dialog.getRootPane().printAll(g);
                            g.dispose();
                            try {
                                javax.imageio.ImageIO.write(
                                        image,
                                        "png",
                                        output.resolve("packaged-dialog.png").toFile());
                            } catch (Exception e) {
                                throw new RuntimeException(e);
                            }
                        } finally {
                            dialog.dispose();
                        }
                    });
            var times = new ArrayList<Double>();
            var paths = new HashSet<Path>();
            for (int i = 0; i < 2; i++) {
                sim.simulate();
                var branch = sim.getSimulatedData().getBranch(0);
                var events =
                        branch.getEvents().stream()
                                .filter(
                                        e ->
                                                e.getType()
                                                        == FlightEvent.Type
                                                                .RECOVERY_DEVICE_DEPLOYMENT)
                                .toList();
                if (events.size() != 1)
                    throw new AssertionError("Expected one physical deployment on run " + i);
                double time = events.get(0).getTime();
                if (Math.abs(time - 4.000375) > 1e-9)
                    throw new AssertionError("Wrong deployment time " + time);
                times.add(time);
                var imported =
                        FlightComputerLogReader.read(sim.getFlightComputerLogPath())
                                .get(0)
                                .data()
                                .getBranch(0);
                var event = imported.getFirstEvent(FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT);
                if (event == null || !events.get(0).getData().equals(event.getData()))
                    throw new AssertionError("Imported recovery metadata");
                paths.add(sim.getFlightComputerLogPath());
            }
            if (paths.size() != 2) throw new AssertionError("Runs must have separate output files");
            var storage = new StorageOptions();
            storage.setSaveSimulationData(true);
            try (var stream = Files.newOutputStream(output.resolve("recovery.ork"))) {
                new info.openrocket.core.file.openrocket.OpenRocketSaver()
                        .save(
                                stream,
                                doc,
                                storage,
                                new info.openrocket.core.logging.WarningSet(),
                                new info.openrocket.core.logging.ErrorSet());
            }
            String summary =
                    "Packaged native dialog: passed\nChannel: 2\nDevice delay: 0.375 ms\nTwo independent deployments (s): "
                            + times
                            + "\nExact event/device metadata on log import: passed\nSaved synthetic recovery.ork\n";
            Files.writeString(output.resolve("summary.txt"), summary);
            System.out.println(summary);
        } finally {
            JavaBoardProgram.revoke(code);
        }
    }

    /**
     * Finds the Swing component required by the verification scenario.
     *
     * @param <T> t type used by this operation
     * @param parent parent window, model or output directory
     * @param type type
     * @param name display name or lookup key
     * @return the resulting T
     */
    private static <T extends Component> T find(Container parent, Class<T> type, String name) {
        for (var child : parent.getComponents()) {
            if (type.isInstance(child) && (name == null || name.equals(child.getName())))
                return type.cast(child);
            if (child instanceof Container nested) {
                T result = find(nested, type, name);
                if (result != null) return result;
            }
        }
        return null;
    }
}
