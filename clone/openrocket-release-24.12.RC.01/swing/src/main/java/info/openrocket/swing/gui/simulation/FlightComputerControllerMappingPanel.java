package info.openrocket.swing.gui.simulation;

import info.openrocket.core.document.Simulation;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.simulation.flightcomputer.*;
import info.openrocket.swing.gui.flightcomputer.FlightComputerActuatorMappingPanel;
import javax.swing.*;
import java.awt.*;

/** Rocket assignments without requiring the portable controller definition to be edited. */
final class FlightComputerControllerMappingPanel extends JPanel {
    private final Simulation simulation;
    private final Runnable changed;
    private final JPanel holder = new JPanel(new BorderLayout());
    private final JComboBox<Item> blocks = new JComboBox<>();
    private final JLabel status = new JLabel();
    private final JButton apply = new JButton("Apply actuator assignments");
    private FlightComputerControllerSettings draft = FlightComputerControllerSettings.EMPTY;
    private boolean pending;
    private FlightComputerDesign design;
    private FlightComputerActuatorMappingPanel editor;
    private boolean refreshing;

    private record Item(FlightComputerController controller) {
        /**
         * Returns the display representation used by this value in logs or UI controls.
         *
         * @return the display representation used by this value in logs or UI controls
         */
        public String toString() {
            return controller.name();
        }
    }

    /**
     * Builds the flight computer controller mapping panel and connects its controls to the supplied
     * model.
     *
     * @param simulation simulation being configured or executed
     * @param changed initial changed
     */
    FlightComputerControllerMappingPanel(Simulation simulation, Runnable changed) {
        super(new BorderLayout(5, 5));
        this.simulation = simulation;
        this.changed = changed;
        setBorder(BorderFactory.createTitledBorder("Controller actuators"));
        add(blocks, BorderLayout.NORTH);
        add(holder);
        var bottom = new JPanel(new BorderLayout(4, 4));
        bottom.add(status);
        var actions = new JPanel(new GridLayout(0, 1, 4, 4));
        actions.add(apply);
        var clear = new JButton("Clear all actuator assignments");
        clear.setName("fc.controllerAssignments.clear");
        clear.addActionListener(
                e -> {
                    ZephyrusFlightComputer.applyControllers(
                            simulation, FlightComputerControllerSettings.EMPTY);
                    changed.run();
                    refresh();
                });
        actions.add(clear);
        bottom.add(actions, BorderLayout.SOUTH);
        add(bottom, BorderLayout.SOUTH);
        blocks.setName("fc.controllerAssignments.block");
        blocks.addActionListener(
                e -> {
                    if (!refreshing) {
                        try {
                            if (editor != null) draft = editor.settings(design.id());
                            showSelected();
                        } catch (RuntimeException error) {
                            status.setText(error.getMessage());
                        }
                    }
                });
        apply.setName("fc.controllerAssignments.apply");
        apply.addActionListener(
                e -> {
                    try {
                        if (editor == null) return;
                        var settings = editor.settings(design.id());
                        settings.validate(
                                design, simulation.getActiveConfiguration().getActiveComponents());
                        ZephyrusFlightComputer.applyControllers(simulation, settings);
                        pending = false;
                        changed.run();
                        refresh();
                    } catch (RuntimeException error) {
                        JOptionPane.showMessageDialog(
                                this,
                                error.getMessage(),
                                "Controller assignments",
                                JOptionPane.ERROR_MESSAGE);
                    }
                });
        refresh();
    }

    /** Reloads controller blocks and assignments from the selected FC design. */
    void refresh() {
        refreshing = true;
        try {
            blocks.removeAllItems();
            editor = null;
            design = null;
            var fc = ZephyrusFlightComputer.read(simulation);
            draft = fc.getControllerSettings();
            pending = false;
            if (fc.hasDesignFile()) {
                try {
                    design =
                            FlightComputerDesign.read(
                                    FlightComputerLibrary.resolve(fc.getDesignReference()));
                    design.controllers().forEach(c -> blocks.addItem(new Item(c)));
                } catch (Exception e) {
                    status.setText("Open the designer to repair the flight computer file");
                }
            }
        } finally {
            refreshing = false;
        }
        showSelected();
    }

    /** Displays physical actuator assignments for the selected controller. */
    private void showSelected() {
        holder.removeAll();
        editor = null;
        apply.setEnabled(false);
        if (design != null && blocks.getSelectedItem() instanceof Item item) {
            editor =
                    new FlightComputerActuatorMappingPanel(
                            simulation,
                            item.controller(),
                            draft,
                            () -> {
                                pending = true;
                                status.setText("Unapplied assignment changes");
                            });
            holder.add(editor);
            apply.setEnabled(true);
            status.setText(
                    pending
                            ? "Unapplied assignment changes"
                            : "Saved in this simulation; definitions stay in .fc");
        } else status.setText("Add controller blocks in the flight computer designer.");
        revalidate();
        repaint();
    }
}
