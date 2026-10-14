package info.openrocket.swing.gui.simulation;

import info.openrocket.swing.util.EnsembleSwingTestCase;
import info.openrocket.core.document.*;
import info.openrocket.core.util.TestRockets;
import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.nio.file.*;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class EnsembleOptionsPanelTest extends EnsembleSwingTestCase {
    /**
     * Verifies that controls bind and fit within existing options panel.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void controlsBindAndFitWithinExistingOptionsPanel() throws Exception {
        SwingUtilities.invokeAndWait(
                () -> {
                    try {
                        var rocket = TestRockets.makeEstesAlphaIII();
                        var doc = OpenRocketDocumentFactory.createDocumentFromRocket(rocket);
                        var simulation = new Simulation(doc, rocket);
                        var panel = new SimulationOptionsPanel(doc, simulation);
                        var checkbox = (JCheckBox) find(panel, "ensemble.enabled");
                        assertNotNull(checkbox);
                        checkbox.doClick();
                        var thrust = (JSpinner) find(panel, "ensemble.Thrust noise σ (N)");
                        thrust.setValue(12.0);
                        var editor = ((JSpinner.NumberEditor) thrust.getEditor()).getTextField();
                        assertThrows(
                                java.text.ParseException.class,
                                () -> editor.getFormatter().stringToValue("-1"));
                        var runCount = (JSpinner) find(panel, "ensemble.Number of runs");
                        assertThrows(
                                java.text.ParseException.class,
                                () ->
                                        ((JSpinner.NumberEditor) runCount.getEditor())
                                                .getTextField()
                                                .getFormatter()
                                                .stringToValue("1"));
                        assertTrue(simulation.getOptions().getEnsembleSettings().enabled());
                        assertEquals(
                                12, simulation.getOptions().getEnsembleSettings().motorSigma());
                        var mass = (JSpinner) find(panel, "ensemble.Total mass σ (%)");
                        var drag = (JSpinner) find(panel, "ensemble.Overall Cd σ (%)");
                        var force = (JSpinner) find(panel, "ensemble.Roll forcing σ (%)");
                        var damping = (JSpinner) find(panel, "ensemble.Roll damping σ (%)");
                        var threads = (JSpinner) find(panel, "ensemble.Threads");
                        assertEquals(4, ((Number) threads.getValue()).intValue());
                        threads.setValue(2);
                        var direction = (JSpinner) find(panel, "ensemble.Wind direction σ (°)");
                        direction.setValue(15.0);
                        assertEquals(2, simulation.getOptions().getEnsembleSettings().threads());
                        assertEquals(Math.toRadians(15), simulation.getOptions().getEnsembleSettings().windDirectionSigma());
                        mass.setValue(10.0);
                        drag.setValue(15.0);
                        force.setValue(20.0);
                        damping.setValue(25.0);
                        assertEquals(.1, simulation.getOptions().getEnsembleSettings().massSigma());
                        assertEquals(.15, simulation.getOptions().getEnsembleSettings().dragSigma());
                        assertEquals(.2, simulation.getOptions().getEnsembleSettings().rollForceSigma());
                        assertEquals(.25, simulation.getOptions().getEnsembleSettings().rollDampingSigma());
                        checkbox.doClick();
                        assertFalse(mass.isEnabled());
                        assertFalse(drag.isEnabled());
                        assertFalse(force.isEnabled());
                        assertFalse(damping.isEnabled());
                        assertFalse(thrust.isEnabled());
                        assertEquals(
                                12, simulation.getOptions().getEnsembleSettings().motorSigma());
                        checkbox.doClick();
                        panel.setSize(1120, 700);
                        layout(panel);
                        for (var c : panel.getComponents())
                            if (c instanceof JScrollPane scroll) {
                                scroll.getViewport()
                                        .setViewPosition(
                                                new Point(
                                                        0,
                                                        Math.max(
                                                                0,
                                                                scroll.getViewport()
                                                                                .getView()
                                                                                .getHeight()
                                                                        - scroll.getViewport()
                                                                                .getHeight())));
                            }
                        var image = new BufferedImage(1120, 700, BufferedImage.TYPE_INT_RGB);
                        var g = image.createGraphics();
                        panel.printAll(g);
                        g.dispose();
                        Path dir = Path.of("build/ensemble-verification");
                        Files.createDirectories(dir);
                        javax.imageio.ImageIO.write(
                                image, "png", dir.resolve("ensemble-options.png").toFile());
                    } catch (Exception e) {
                        throw new RuntimeException(e);
                    }
                });
    }

    /**
     * Verifies that progress dialog runs ensemble without reading locked options on edt.
     *
     * @throws Exception if the requested operation cannot be completed
     * @throws AssertionError when rejected: Progress UI must use its options snapshot while the
     *     simulation runs
     */
    @Test
    void progressDialogRunsEnsembleWithoutReadingLockedOptionsOnEdt() throws Exception {
        var r = TestRockets.makeEstesAlphaIII();
        var doc = OpenRocketDocumentFactory.createDocumentFromRocket(r);
        var simulation =
                new Simulation(doc, r) {
                    /**
                     * Returns the simulation options supplied by the regression fixture.
                     *
                     * @return the simulation options supplied by the regression fixture
                     * @throws AssertionError when rejected: Progress UI must use its options
                     *     snapshot while the simulation runs
                     */
                    @Override
                    public info.openrocket.core.simulation.SimulationOptions getOptions() {
                        if (SwingUtilities.isEventDispatchThread() && getEnsembleRunNumber() > 0)
                            throw new AssertionError(
                                    "Progress UI must use its options snapshot while the simulation runs");
                        return super.getOptions();
                    }
                };
        simulation.setFlightConfigurationId(TestRockets.TEST_FCID_1);
        simulation.getOptions().setMaxSimulationTime(2);
        simulation.getOptions().setTimeStep(.02);
        simulation.getOptions().setISAAtmosphere(true);
        simulation.getOptions().setLaunchRodLength(1);
        simulation
                .getOptions()
                .setEnsembleSettings(
                        new info.openrocket.core.simulation.ensemble.EnsembleSettings(
                                true, 2, .1, .05, 0, 0, 0, 4));
        java.util.concurrent.atomic.AtomicReference<Throwable> error =
                new java.util.concurrent.atomic.AtomicReference<>();
        Thread.UncaughtExceptionHandler previous = Thread.getDefaultUncaughtExceptionHandler();
        Thread.setDefaultUncaughtExceptionHandler(
                (thread, failure) -> error.compareAndSet(null, failure));
        var dialog = new SimulationRunDialog[1];
        try {
            SwingUtilities.invokeAndWait(
                    () -> dialog[0] = new SimulationRunDialog(null, doc, simulation));
            var field = SimulationRunDialog.class.getDeclaredField("simulationWorkers");
            field.setAccessible(true);
            for (var worker : (SimulationWorker[]) field.get(dialog[0]))
                worker.get(30, java.util.concurrent.TimeUnit.SECONDS);
            SwingUtilities.invokeAndWait(() -> {});
            assertNull(error.get());
            assertNotNull(simulation.getSimulatedData().getEnsembleResult());
        } finally {
            SwingUtilities.invokeAndWait(
                    () -> {
                        if (dialog[0] != null) dialog[0].dispose();
                    });
            Thread.setDefaultUncaughtExceptionHandler(previous);
        }
    }

    /**
     * Finds a descendant Swing component matching the requested test selector.
     *
     * @param parent parent window, model or output directory
     * @param name display name or lookup key
     * @return the resulting Component
     */
    private static Component find(Container parent, String name) {
        for (var child : parent.getComponents()) {
            if (name.equals(child.getName())) return child;
            if (child instanceof Container c) {
                var found = find(c, name);
                if (found != null) return found;
            }
        }
        return null;
    }

    /**
     * Lays out nested Swing components so size and clipping assertions use realized bounds.
     *
     * @param c selected channel or controller
     */
    private static void layout(Container c) {
        c.doLayout();
        for (var child : c.getComponents()) if (child instanceof Container nested) layout(nested);
    }
}
