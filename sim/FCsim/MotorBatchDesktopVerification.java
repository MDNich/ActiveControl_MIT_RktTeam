import com.google.inject.*;
import com.google.inject.util.Modules;
import info.openrocket.core.database.motor.*;
import info.openrocket.core.document.*;
import info.openrocket.core.motor.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.motorbatch.*;
import info.openrocket.core.startup.Application;
import info.openrocket.core.plugin.PluginModule;
import info.openrocket.core.util.TestRockets;
import info.openrocket.swing.startup.GuiModule;
import info.openrocket.swing.gui.simulation.MotorBatchDialog;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.atomic.AtomicReference;
import javax.swing.*;

/** Smoke test of the packaged JAR and native dialog, runnable on macOS and Windows. */
public class MotorBatchDesktopVerification {
    /**
     * Runs the motor batch desktop verification entry point, reporting verification failures to the
     * invoking process.
     *
     * @param args command-line arguments
     * @throws Exception if the requested operation cannot be completed
     */
    public static void main(String[] args) throws Exception {
        try {
            verify(Path.of(args[0]));
            System.exit(0);
        } catch (Throwable t) {
            t.printStackTrace();
            System.exit(1);
        }
    }

    /**
     * Runs the packaged-application integration assertions and writes verification artifacts.
     *
     * @param output output snapshot or destination
     * @throws Exception if the requested operation cannot be completed
     * @throws AssertionError when rejected: Default worker count
     */
    private static void verify(Path output) throws Exception {
        Files.createDirectories(output);
        System.setProperty("openrocket.bypass.presets", "true");
        System.setProperty("openrocket.bypass.motors", "true");
        var db = new ThrustCurveMotorSetDatabase();
        var module = new GuiModule();
        Application.setInjector(
                Guice.createInjector(
                        Modules.override(module)
                                .with(
                                        new AbstractModule() {
                                            /**
                                             * Configures application services for the standalone
                                             * verification run.
                                             */
                                            @Override
                                            protected void configure() {
                                                bind(ThrustCurveMotorSetDatabase.class)
                                                        .toInstance(db);
                                                bind(MotorDatabase.class).toInstance(db);
                                            }
                                        }),
                        new PluginModule()));
        module.startLoader();
        var rocket = TestRockets.makeEstesAlphaIII();
        for (var c : rocket)
            if (c instanceof InnerTube t && t.isMotorMount()) {
                t.setOuterRadius(.0094);
                t.setThickness(.0003);
                for (var child : t.getChildren())
                    if (child instanceof EngineBlock block)
                        block.setAxialOffset(-block.getLength());
                for (var it = t.getMotorIterator(); it.hasNext(); )
                    if (it.next().getMotor() instanceof ThrustCurveMotor motor) db.addMotor(motor);
            }
        var doc = OpenRocketDocumentFactory.createDocumentFromRocket(rocket);
        var source = new Simulation(doc, rocket);
        source.setFlightConfigurationId(TestRockets.TEST_FCID_1);
        source.setName("Packaged motor batch verification");
        source.getOptions().setRandomSeed(17);
        source.getOptions().setMaxSimulationTime(80);
        source.getOptions().setWindSpeedAverage(0);
        source.getOptions().setWindSpeedDeviation(0);
        doc.addSimulation(source);
        var dialog = new AtomicReference<MotorBatchDialog>();
        SwingUtilities.invokeAndWait(
                () -> {
                    var d = new MotorBatchDialog(null, doc, source);
                    dialog.set(d);
                    d.setModalityType(Dialog.ModalityType.MODELESS);
                    d.setVisible(true);
                    d.validate();
                    if (!Integer.valueOf(4).equals(find(d, JSpinner.class, null).getValue()))
                        throw new AssertionError("Default worker count");
                    if (find(d, JTable.class, null).getRowCount() != 3)
                        throw new AssertionError("Fixture catalogue count");
                    var image =
                            new BufferedImage(
                                    d.getWidth(), d.getHeight(), BufferedImage.TYPE_INT_RGB);
                    var g = image.createGraphics();
                    d.getRootPane().printAll(g);
                    g.dispose();
                    try {
                        javax.imageio.ImageIO.write(
                                image, "png", output.resolve("dialog.png").toFile());
                    } catch (Exception e) {
                        throw new RuntimeException(e);
                    }
                    find(d, JButton.class, "Run batch").doClick();
                });
        var report = new AtomicReference<MotorBatchReport>();
        long deadline = System.nanoTime() + 120_000_000_000L;
        while (report.get() == null && System.nanoTime() < deadline) {
            SwingUtilities.invokeAndWait(
                    () -> {
                        if (!doc.getMotorBatches().isEmpty())
                            report.set(doc.getMotorBatches().get(0));
                    });
            Thread.sleep(50);
        }
        SwingUtilities.invokeAndWait(() -> dialog.get().dispose());
        var result = Objects.requireNonNull(report.get(), "Dialog batch timed out");
        if (result.peakThreads() < 1 || result.peakThreads() > 4)
            throw new AssertionError("Worker bound");
        if (result.rows().stream()
                .anyMatch(r -> r.selected().status() == MotorBatchReport.Status.ERROR))
            throw new AssertionError("Trial error");
        if (source.getSimulatedData() != null) throw new AssertionError("Baseline mutated");
        MotorBatchCSV.write(
                output.resolve("results.csv"),
                result,
                MotorBatchCSV.Scope.ALL_CANDIDATES,
                Set.of());
        Files.writeString(output.resolve("report.json"), MotorBatchIO.write(result));
        var completed =
                result.rows().stream().filter(r -> !Double.isNaN(r.selected().delay())).toList();
        var shortlist =
                MotorBatchSnapshot.createSimulations(
                        MotorBatchIO.read(MotorBatchIO.write(result)), completed);
        if (shortlist.getSimulationCount() != completed.size())
            throw new AssertionError("Shortlist count");
        String summary =
                "OS: "
                        + System.getProperty("os.name")
                        + "\nJava: "
                        + System.getProperty("java.version")
                        + "\nMotors: "
                        + result.rows().size()
                        + "\nPeak workers: "
                        + result.peakThreads()
                        + "\nShortlist: "
                        + shortlist.getSimulationCount()
                        + "\nPassed dialog, execution, CSV, report reload, shortlist, and baseline isolation.\n";
        Files.writeString(output.resolve("summary.txt"), summary);
        System.out.println(summary);
    }

    /**
     * Finds the Swing component required by the verification scenario.
     *
     * @param <T> t type used by this operation
     * @param parent parent window, model or output directory
     * @param type type
     * @param text text
     * @return the resulting T
     */
    private static <T extends Component> T find(Container parent, Class<T> type, String text) {
        for (var c : parent.getComponents()) {
            if (type.isInstance(c)
                    && (text == null || c instanceof AbstractButton b && text.equals(b.getText())))
                return type.cast(c);
            if (c instanceof Container child) {
                T found = find(child, type, text);
                if (found != null) return found;
            }
        }
        return null;
    }
}
