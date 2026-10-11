package info.openrocket.swing.gui.simulation;

import info.openrocket.core.document.*;
import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.swing.gui.components.ConfigurationComboBox;
import info.openrocket.swing.gui.adaptors.BooleanModel;
import info.openrocket.swing.gui.util.GUIUtil;
import info.openrocket.core.simulation.SimulationOptions;
import info.openrocket.swing.util.EnsembleSwingTestCase;
import org.junit.jupiter.api.Test;

import javax.swing.*;
import java.awt.GraphicsEnvironment;
import java.util.Collection;
import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assumptions.assumeFalse;

class SimulationConfigDialogLifecycleTest extends EnsembleSwingTestCase {
    /**
     * Builds or reads the listener count fixture for the regression assertions.
     *
     * @param source source text, model or measured-channel selection
     * @param field field
     * @return computed integer value
     */
    private static int listenerCount(Object source, String field) {
        try {
            var member = source.getClass().getDeclaredField(field);
            member.setAccessible(true);
            return ((Collection<?>) member.get(source)).size();
        } catch (ReflectiveOperationException e) {
            throw new AssertionError(e);
        }
    }

    /**
     * Verifies that button cleanup unsubscribes its action before it is detached.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void buttonCleanupUnsubscribesItsActionBeforeItIsDetached() throws Exception {
        SwingUtilities.invokeAndWait(
                () -> {
                    var options = new SimulationOptions();
                    int baseline = listenerCount(options, "listeners");
                    var model = new BooleanModel(options, "ISAAtmosphere");
                    var button = new JCheckBox(model);
                    assertEquals(baseline + 1, listenerCount(options, "listeners"));
                    GUIUtil.setNullModels(button);
                    assertEquals(baseline, listenerCount(options, "listeners"));
                });
    }

    /**
     * Verifies that configuration selector releases and restores its rocket subscription.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void configurationSelectorReleasesAndRestoresItsRocketSubscription() throws Exception {
        SwingUtilities.invokeAndWait(
                () -> {
                    var rocket = new Rocket();
                    int baseline = listenerCount(rocket, "listenerList");
                    var selector = new ConfigurationComboBox(rocket, false);
                    assertEquals(baseline, listenerCount(rocket, "listenerList"));
                    for (int i = 0; i < 3; i++) {
                        selector.addNotify();
                        assertEquals(baseline + 1, listenerCount(rocket, "listenerList"));
                        selector.removeNotify();
                        assertEquals(baseline, listenerCount(rocket, "listenerList"));
                    }
                });
    }

    /**
     * Verifies that disposing edited and unedited dialogs releases external listeners.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void disposingEditedAndUneditedDialogsReleasesExternalListeners() throws Exception {
        assumeFalse(GraphicsEnvironment.isHeadless(), "Requires native Swing window lifecycle");
        var document = OpenRocketDocumentFactory.createNewRocket();
        var simulation = new Simulation(document, document.getRocket());
        document.addSimulation(simulation);
        int simulationListeners = listenerCount(simulation, "listeners");
        int rocketListeners = listenerCount(document.getRocket(), "listenerList");
        int optionListeners = listenerCount(simulation.getOptions(), "listeners");
        for (int i = 0; i < 4; i++) {
            final int iteration = i;
            SwingUtilities.invokeAndWait(
                    () -> {
                        var dialog = new SimulationConfigDialog(null, document, false, simulation);
                        if (iteration % 2 == 1) simulation.setName("Edited " + iteration);
                        dialog.dispose();
                    });
            // windowClosed performs the existing model cleanup on the event queue.
            SwingUtilities.invokeAndWait(
                    () -> {
                        assertEquals(simulationListeners, listenerCount(simulation, "listeners"));
                        assertEquals(
                                rocketListeners,
                                listenerCount(document.getRocket(), "listenerList"));
                        assertEquals(
                                optionListeners,
                                listenerCount(simulation.getOptions(), "listeners"));
                    });
        }
    }

    /**
     * Verifies that repeated runs and closed editors do not retain previous flight results.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void repeatedRunsAndClosedEditorsDoNotRetainPreviousFlightResults() throws Exception {
        assumeFalse(GraphicsEnvironment.isHeadless(), "Requires native Swing window lifecycle");
        var rocket = info.openrocket.core.util.TestRockets.makeEstesAlphaIII();
        var document = OpenRocketDocumentFactory.createDocumentFromRocket(rocket);
        var simulation = new Simulation(document, rocket);
        document.addSimulation(simulation);
        simulation.setFlightConfigurationId(info.openrocket.core.util.TestRockets.TEST_FCID_1);
        simulation.getOptions().setMaxSimulationTime(2);
        var retired = new java.util.ArrayList<java.lang.ref.WeakReference<?>>();
        for (int i = 0; i < 4; i++) {
            if (simulation.getSimulatedData() != null)
                retired.add(new java.lang.ref.WeakReference<>(simulation.getSimulatedData()));
            retired.add(runAndClose(document, simulation));
            SwingUtilities.invokeAndWait(
                    () -> {
                        var editor = new SimulationConfigDialog(null, document, false, simulation);
                        retired.add(new java.lang.ref.WeakReference<>(editor));
                        editor.dispose();
                    });
            SwingUtilities.invokeAndWait(() -> {});
        }
        for (int i = 0; i < 30 && retired.stream().anyMatch(r -> r.get() != null); i++) {
            System.gc();
            Thread.sleep(50);
            SwingUtilities.invokeAndWait(() -> {});
        }
        assertEquals(
                0,
                retired.stream().filter(r -> r.get() != null).count(),
                "Closed dialogs and superseded results must be collectible");
        assertNotNull(
                simulation.getSimulatedData(), "Keep the current flight for plotting and saving");
    }

    /**
     * Runs a simulation, closes its editors and releases references for retention checks.
     *
     * @param document document
     * @param simulation simulation being configured or executed
     * @return the resulting WeakReference&lt;?&gt;
     * @throws Exception if the requested operation cannot be completed
     */
    private static java.lang.ref.WeakReference<?> runAndClose(
            OpenRocketDocument document, Simulation simulation) throws Exception {
        var holder = new SimulationRunDialog[1];
        SwingUtilities.invokeAndWait(
                () -> holder[0] = new SimulationRunDialog(null, document, simulation));
        var field = SimulationRunDialog.class.getDeclaredField("simulationWorkers");
        field.setAccessible(true);
        for (var worker : (SimulationWorker[]) field.get(holder[0]))
            worker.get(30, java.util.concurrent.TimeUnit.SECONDS);
        SwingUtilities.invokeAndWait(() -> holder[0].dispose());
        SwingUtilities.invokeAndWait(() -> {});
        return new java.lang.ref.WeakReference<>(holder[0]);
    }
}
