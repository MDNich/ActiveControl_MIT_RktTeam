package info.openrocket.swing.gui.main;

import info.openrocket.core.document.OpenRocketDocumentFactory;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.simulation.SimulationOptions;
import info.openrocket.swing.util.EnsembleSwingTestCase;
import org.junit.jupiter.api.Test;

import javax.swing.JButton;
import javax.swing.SwingUtilities;
import java.awt.Component;
import java.awt.GraphicsEnvironment;
import java.awt.event.ActionEvent;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assumptions.assumeFalse;

class SimulationPanelTest extends EnsembleSwingTestCase {
    /**
     * Verifies that new rocket panel starts with batch action disabled.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void newRocketPanelStartsWithBatchActionDisabled() throws Exception {
        assumeFalse(GraphicsEnvironment.isHeadless());
        SwingUtilities.invokeAndWait(
                () -> {
                    var document = OpenRocketDocumentFactory.createNewRocket();
                    var panel = new SimulationPanel(null, document);
                    var button = batchButton(panel);
                    assertFalse(button.isEnabled());
                    // A stale queued action must also tolerate an empty selection.
                    button.getAction()
                            .actionPerformed(
                                    new ActionEvent(button, ActionEvent.ACTION_PERFORMED, "batch"));
                });
    }

    /**
     * Verifies that existing simulations start unselected and batch tracks selection.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void existingSimulationsStartUnselectedAndBatchTracksSelection() throws Exception {
        assumeFalse(GraphicsEnvironment.isHeadless());
        SwingUtilities.invokeAndWait(
                () -> {
                    var document = OpenRocketDocumentFactory.createNewRocket();
                    document.addSimulation(new Simulation(document, document.getRocket()));
                    document.addSimulation(new Simulation(document, document.getRocket()));
                    document.addSimulation(
                            new Simulation(
                                    document,
                                    document.getRocket(),
                                    Simulation.Status.EXTERNAL,
                                    "Imported flight",
                                    new SimulationOptions(),
                                    List.of(),
                                    null));
                    var panel = new SimulationPanel(null, document);
                    var button = batchButton(panel);
                    var selection = panel.getSimulationListSelectionModel();
                    assertFalse(
                            button.isEnabled(),
                            "Opening a populated document must tolerate no selected simulation");
                    selection.setSelectionInterval(0, 0);
                    assertTrue(button.isEnabled());
                    selection.setSelectionInterval(0, 1);
                    assertFalse(button.isEnabled(), "Batch simulation needs exactly one baseline");
                    selection.setSelectionInterval(2, 2);
                    assertFalse(
                            button.isEnabled(),
                            "Imported flight data is not a simulatable baseline");
                    button.getAction()
                            .actionPerformed(
                                    new ActionEvent(button, ActionEvent.ACTION_PERFORMED, "batch"));
                    selection.clearSelection();
                    assertFalse(button.isEnabled());
                });
    }

    /**
     * Builds or reads the batch button fixture for the regression assertions.
     *
     * @param panel panel
     * @return the resulting JButton
     * @throws AssertionError when rejected: Missing motor batch button
     */
    private static JButton batchButton(SimulationPanel panel) {
        for (Component component : panel.getComponents()) {
            if (component instanceof JButton button
                    && "Batch simulate motors…".equals(button.getText())) {
                return button;
            }
        }
        throw new AssertionError("Missing motor batch button");
    }
}
