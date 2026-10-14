package info.openrocket.swing.gui.flightcomputer;

import info.openrocket.core.document.Simulation;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.flightcomputer.*;
import info.openrocket.core.util.TestRockets;
import info.openrocket.swing.util.EnsembleSwingTestCase;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.nio.file.*;
import static org.junit.jupiter.api.Assertions.*;

class FlightComputerControllerPanelTest extends EnsembleSwingTestCase {
    @TempDir Path directory;

    /**
     * Verifies that signal flow editor applies and saves controller and uuid mapping together.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void signalFlowEditorAppliesAndSavesControllerAndUuidMappingTogether() throws Exception {
        var file = directory.resolve("blocks.fc");
        FlightComputerDesign.zephyrus().write(file);
        SwingUtilities.invokeAndWait(
                () -> {
                    try {
                        var rocket = TestRockets.makeEstesAlphaIII();
                        var old =
                                (TrapezoidFinSet)
                                        rocket
                                                .getSelectedConfiguration()
                                                .getActiveComponents()
                                                .stream()
                                                .filter(c -> c instanceof TrapezoidFinSet)
                                                .findFirst()
                                                .orElseThrow();
                        var fin = new TabControlledTrapezoidFinSet(old);
                        fin.setName("Aft roll fins");
                        fin.setTabSpan(.02);
                        fin.setTabChord(.01);
                        fin.setCNALPHA(2);
                        var sim = new Simulation(rocket);
                        var editor = new FlightComputerDesigner(file, sim, p -> {});
                        editor.setSize(1280, 850);
                        layout(editor);
                        editor.addNode("controller", 0, 0);
                        layout(editor);
                        var panel = (FlightComputerControllerPanel) find(editor, "fc.controllers");
                        assertNotNull(panel);
                        assertEquals(1, editor.getDesign().controllers().size());
                        var name = (JTextField) find(panel, "fc.controller.name");
                        name.setText("Roll damping");
                        ((JCheckBox) find(panel, "fc.controller.enabled")).doClick();
                        ((JSpinner) find(panel, "fc.controller.kp")).setValue(.025);
                        ((JComboBox<?>) find(panel, "fc.controller.target")).setSelectedIndex(1);
                        assertTrue(panel.hasPending());
                        panel.applyPending();
                        assertFalse(panel.hasPending());
                        assertEquals(
                                "Roll damping", editor.getDesign().controllers().get(0).name());
                        assertEquals(.025, editor.getDesign().controllers().get(0).n("kp"));
                        var action = editor.getActionMap().get("save-control");
                        action.actionPerformed(new java.awt.event.ActionEvent(editor, 0, "save"));
                        assertEquals(
                                editor.getDesign().json(), FlightComputerDesign.read(file).json());
                        var bindings =
                                info.openrocket.core.simulation.extension.impl
                                        .ZephyrusFlightComputer.read(sim)
                                        .getControllerSettings()
                                        .bindings();
                        assertEquals(1, bindings.size());
                        assertEquals(fin.getID(), bindings.get(0).deviceId());
                        Path images = Path.of("build/fc-controller-verification");
                        Files.createDirectories(images);
                        var tabs = findTabs(panel);
                        assertNotNull(tabs);
                        for (int i = 0; i < 3; i++) {
                            tabs.setSelectedIndex(i);
                            layout(editor);
                            layout(editor);
                            render(editor, images.resolve("controllers-" + i + ".png"));
                        }
                        panel.addController();
                        assertEquals(2, editor.getDesign().controllers().size());
                        editor.getActionMap()
                                .get("undo")
                                .actionPerformed(new java.awt.event.ActionEvent(editor, 0, "undo"));
                        assertEquals(1, editor.getDesign().controllers().size());
                    } catch (Exception ex) {
                        throw new RuntimeException(ex);
                    }
                });
    }

    @Test
    void ordinaryFinsCanBecomeUsableControllerTargetsWithoutLosingProperties() throws Exception {
        SwingUtilities.invokeAndWait(() -> {
            var rocket = TestRockets.makeEstesAlphaIII();
            var doc = info.openrocket.core.document.OpenRocketDocumentFactory.createDocumentFromRocket(rocket);
            var sim = new Simulation(doc, rocket);
            var original = (TrapezoidFinSet) rocket.getSelectedConfiguration().getActiveComponents().stream()
                    .filter(c -> c instanceof TrapezoidFinSet).findFirst().orElseThrow();
            original.setFilletRadius(.004);
            original.setMountingTabOffset(.009);
            var id = original.getID();
            var design = FlightComputerDesign.zephyrus();
            var controller = FlightComputerController.create(design);
            assertEquals(FlightComputerController.Mode.TABS, controller.mode());
            var panel = new FlightComputerActuatorMappingPanel(sim, controller, FlightComputerControllerSettings.EMPTY, () -> {});
            var target = (JComboBox<?>) find(panel, "fc.controller.target");
            assertEquals(1, target.getItemCount());
            ((JButton) find(panel, "fc.controller.createTabs")).doClick();
            var fin = assertInstanceOf(TabControlledTrapezoidFinSet.class, rocket.findComponent(id));
            assertEquals(.004, fin.getFilletRadius());
            assertEquals(.009, fin.getMountingTabOffset(), 1e-12);
            assertTrue(fin.getTabChord() * fin.getTabSpan() * fin.getCNALPHA() > 0);
            assertEquals(2, target.getItemCount());
            assertEquals(1, target.getSelectedIndex());
            var settings = panel.settings(design.id());
            assertEquals(id, settings.bindings().get(0).deviceId());
            var withController = FlightComputerController.replace(design, java.util.List.of(controller));
            settings.validate(withController, sim.getActiveConfiguration().getActiveComponents());
            panel.setSize(550, 500);
            layout(panel);
            try {
                Path images = Path.of("build/fc-controller-verification");
                Files.createDirectories(images);
                render(panel, images.resolve("tab-assignment.png"));
            } catch (Exception e) { throw new RuntimeException(e); }
        });
    }

    /**
     * Renders the test view so its controls and layout can be inspected.
     *
     * @param c selected channel or controller
     * @param p p
     * @throws Exception if the requested operation cannot be completed
     */
    private static void render(JComponent c, Path p) throws Exception {
        var image = new BufferedImage(c.getWidth(), c.getHeight(), BufferedImage.TYPE_INT_RGB);
        var g = image.createGraphics();
        c.printAll(g);
        g.dispose();
        javax.imageio.ImageIO.write(image, "png", p.toFile());
    }

    /**
     * Finds a descendant Swing component matching the requested test selector.
     *
     * @param c selected channel or controller
     * @param name display name or lookup key
     * @return the resulting Component
     */
    private static Component find(Container c, String name) {
        for (var child : c.getComponents()) {
            if (name.equals(child.getName())) return child;
            if (child instanceof Container nested) {
                var found = find(nested, name);
                if (found != null) return found;
            }
        }
        return null;
    }

    /**
     * Finds the nested tabbed pane used by the UI assertions.
     *
     * @param c selected channel or controller
     * @return the resulting JTabbedPane
     */
    private static JTabbedPane findTabs(Container c) {
        for (var child : c.getComponents()) {
            if (child instanceof JTabbedPane t) return t;
            if (child instanceof Container nested) {
                var f = findTabs(nested);
                if (f != null) return f;
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
