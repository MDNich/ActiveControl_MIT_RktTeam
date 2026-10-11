package info.openrocket.swing.gui.simulation;

import info.openrocket.core.document.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.simulation.flightcomputer.FlightComputerRecoverySettings;
import info.openrocket.core.util.TestRockets;
import info.openrocket.swing.util.EnsembleSwingTestCase;
import org.junit.jupiter.api.Test;
import javax.swing.*;
import java.awt.*;
import java.nio.file.*;
import static org.junit.jupiter.api.Assertions.*;

class FlightComputerRecoveryPanelTest extends EnsembleSwingTestCase {
    /**
     * Verifies that assignments are editable persist when disabled and deleted devices can be
     * cleared.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void assignmentsAreEditablePersistWhenDisabledAndDeletedDevicesCanBeCleared() throws Exception {
        SwingUtilities.invokeAndWait(
                () -> {
                    var rocket = TestRockets.makeEstesAlphaIII();
                    var sim = new Simulation(rocket);
                    var chute =
                            (RecoveryDevice)
                                    rocket.getSelectedConfiguration().getActiveComponents().stream()
                                            .filter(c -> c instanceof RecoveryDevice)
                                            .findFirst()
                                            .orElseThrow();
                    var panel = new FlightComputerPanel(sim, () -> {});
                    var enabled = (JCheckBox) find(panel, "fc.enabled");
                    var table = (JTable) find(panel, "fc.recovery.mappings");
                    assertFalse(table.isEnabled());
                    enabled.doClick();
                    assertTrue(table.isEnabled());
                    table.getModel().setValueAt(true, 0, 1);
                    table.getModel().setValueAt(true, 0, 2);
                    table.getModel().setValueAt(.250, 0, 7);
                    var expected =
                            new FlightComputerRecoverySettings(
                                    java.util.List.of(
                                            new FlightComputerRecoverySettings.Binding(
                                                    chute.getID(), java.util.Set.of(0, 1), 250)));
                    assertEquals(expected, ZephyrusFlightComputer.read(sim).getRecoverySettings());
                    enabled.doClick();
                    assertEquals(expected, ZephyrusFlightComputer.read(sim).getRecoverySettings());
                    assertFalse(table.isEnabled());
                    assertTrue(
                            ((JLabel) find(panel, "fc.recovery.authority"))
                                    .getText()
                                    .contains("saved OpenRocket"));
                    enabled.doClick();
                    chute.setName("Recovery device renamed");
                    panel.refresh();
                    assertTrue(table.getValueAt(0, 0).toString().contains("renamed"));
                    render(panel);
                    chute.getParent().removeChild(chute);
                    panel.refresh();
                    assertTrue(table.getValueAt(0, 8).toString().contains("Missing"));
                    table.setRowSelectionInterval(0, 0);
                    ((JButton) find(panel, "fc.recovery.clear")).doClick();
                    assertEquals(
                            FlightComputerRecoverySettings.EMPTY,
                            ZephyrusFlightComputer.read(sim).getRecoverySettings());
                });
    }

    /**
     * Renders the test view so its controls and layout can be inspected.
     *
     * @param panel panel
     */
    private static void render(JComponent panel) {
        try {
            panel.setSize(850, panel.getPreferredSize().height);
            layout(panel);
            var image =
                    new java.awt.image.BufferedImage(
                            panel.getWidth(),
                            panel.getHeight(),
                            java.awt.image.BufferedImage.TYPE_INT_RGB);
            var g = image.createGraphics();
            panel.printAll(g);
            g.dispose();
            Path dir = Path.of("build/fc-recovery-verification");
            Files.createDirectories(dir);
            javax.imageio.ImageIO.write(
                    image, "png", dir.resolve("recovery-settings.png").toFile());
        } catch (Exception e) {
            throw new RuntimeException(e);
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
            if (child instanceof Container nested) {
                var found = find(nested, name);
                if (found != null) return found;
            }
        }
        return null;
    }

    /**
     * Lays out nested Swing components so size and clipping assertions use realized bounds.
     *
     * @param parent parent window, model or output directory
     */
    private static void layout(Container parent) {
        parent.doLayout();
        for (var child : parent.getComponents())
            if (child instanceof Container nested) layout(nested);
    }
}
