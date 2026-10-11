package info.openrocket.swing.gui.flightcomputer;

import info.openrocket.core.document.Simulation;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.flightcomputer.*;
import javax.swing.*;
import java.awt.*;
import java.util.*;
import java.util.List;

/** Shared assignment editor; changes stay local until its enclosing editor applies them. */
public final class FlightComputerActuatorMappingPanel extends JPanel {
    private record Device(UUID id, String label) {
        /**
         * Returns the display representation used by this value in logs or UI controls.
         *
         * @return the display representation used by this value in logs or UI controls
         */
        public String toString() {
            return label;
        }
    }

    private static final class Row {
        JComboBox<Device> device;
        JSpinner trim, min, max;
        JCheckBox reverse, offset;
        JPanel panel;
    }

    private final Simulation simulation;
    private final FlightComputerController controller;
    private final FlightComputerControllerSettings original;
    private final Runnable changed;
    private final List<Row> rows = new ArrayList<>();
    private final JPanel contents = new JPanel();

    /**
     * Builds the flight computer actuator mapping panel and connects its controls to the supplied
     * model.
     *
     * @param simulation simulation being configured or executed
     * @param controller initial controller
     * @param settings validated configuration for this operation
     * @param changed initial changed
     */
    public FlightComputerActuatorMappingPanel(
            Simulation simulation,
            FlightComputerController controller,
            FlightComputerControllerSettings settings,
            Runnable changed) {
        super(new BorderLayout(4, 4));
        this.simulation = simulation;
        this.controller = controller;
        original = settings;
        this.changed = changed;
        setBorder(BorderFactory.createTitledBorder("This simulation — physical assignments"));
        contents.setLayout(new BoxLayout(contents, BoxLayout.Y_AXIS));
        add(contents);
        if (simulation == null) {
            add(new JLabel("Assign actuators when opened from a simulation."), BorderLayout.NORTH);
            return;
        }
        var add = new JButton("Add another target");
        add.setName("fc.controller.addTarget");
        add.addActionListener(
                e -> {
                    addRow(null);
                    changed.run();
                });
        add(add, BorderLayout.SOUTH);
        settings.forController(controller.id()).forEach(this::addRow);
        if (rows.isEmpty()) addRow(null);
    }

    /**
     * Creates a numeric physical-binding editor with bounded values.
     *
     * @param v v
     * @return the resulting JSpinner
     */
    private JSpinner number(double v) {
        var s = new JSpinner(new SpinnerNumberModel(v, -100000.0, 100000.0, .1));
        s.setEditor(new JSpinner.NumberEditor(s, "0.###"));
        s.addChangeListener(e -> changed.run());
        return s;
    }

    /**
     * Adds a target row with sign, trim and output-limit controls.
     *
     * @param binding binding
     */
    private void addRow(FlightComputerControllerSettings.Binding binding) {
        var row = new Row();
        row.panel =
                new JPanel(
                        new net.miginfocom.swing.MigLayout(
                                "fillx,insets 5,wrap 2", "[grow,fill][grow,fill]"));
        row.device = new JComboBox<>();
        row.device.setName("fc.controller.target");
        row.device.addItem(new Device(null, "Recorded only / no physical target"));
        var active = simulation.getActiveConfiguration();
        boolean found = binding == null;
        for (var component : simulation.getRocket())
            if (FlightComputerControllerSettings.eligible(controller.mode(), component)) {
                boolean available = active.isComponentActive(component);
                String label =
                        component.getName()
                                + " — "
                                + (component.getParent() == null
                                        ? "Rocket"
                                        : component.getParent().getName())
                                + " · "
                                + component.getID().toString().substring(0, 8)
                                + (available ? "" : " (inactive)");
                if (component instanceof TabControlledTrapezoidFinSet fin
                        && (fin.getTabChord() * fin.getTabSpan() == 0 || fin.getCNALPHA() == 0))
                    label += " (zero tab authority)";
                var item = new Device(component.getID(), label);
                row.device.addItem(item);
                if (binding != null && binding.deviceId().equals(item.id)) {
                    row.device.setSelectedItem(item);
                    found = true;
                }
            }
        if (!found) {
            var missing = new Device(binding.deviceId(), "Missing: " + binding.deviceId());
            row.device.addItem(missing);
            row.device.setSelectedItem(missing);
        }
        row.panel.add(row.device, "span,growx,wmin 0");
        row.device.addActionListener(e -> changed.run());
        row.reverse =
                new JCheckBox("Reverse direction", binding != null && binding.direction() < 0);
        row.reverse.addActionListener(e -> changed.run());
        row.offset =
                new JCheckBox("Offset from saved cant", binding == null || binding.cantOffset());
        row.offset.setVisible(controller.mode() == FlightComputerController.Mode.CANT);
        row.offset.addActionListener(e -> changed.run());
        row.panel.add(row.reverse);
        row.panel.add(row.offset);
        row.trim = number(binding == null ? 0 : binding.trim());
        row.min = number(binding == null ? controller.n("minimum") : binding.minimum());
        row.max = number(binding == null ? controller.n("maximum") : binding.maximum());
        row.panel.add(new JLabel("Target trim / neutral offset"));
        row.panel.add(row.trim);
        row.panel.add(new JLabel("Physical minimum"));
        row.panel.add(row.min);
        row.panel.add(new JLabel("Physical maximum"));
        row.panel.add(row.max);
        var remove = new JButton("Remove target");
        remove.addActionListener(
                e -> {
                    rows.remove(row);
                    contents.remove(row.panel);
                    contents.revalidate();
                    contents.repaint();
                    changed.run();
                });
        row.panel.add(remove, "span,align right");
        rows.add(row);
        contents.add(row.panel);
        revalidate();
        repaint();
    }

    /**
     * Reads and commits a numeric target-binding editor.
     *
     * @param s s
     * @return computed numeric value
     * @throws IllegalArgumentException when rejected: Invalid actuator number
     */
    private static double value(JSpinner s) {
        try {
            s.commitEdit();
        } catch (java.text.ParseException e) {
            throw new IllegalArgumentException("Invalid actuator number", e);
        }
        return ((Number) s.getValue()).doubleValue();
    }

    /**
     * Validates and returns the edited stable-UUID actuator assignments.
     *
     * @param designId design id
     * @return the resulting FlightComputerControllerSettings
     */
    public FlightComputerControllerSettings settings(String designId) {
        if (simulation == null) return original;
        var bindings =
                new ArrayList<>(
                        original.bindings().stream()
                                .filter(b -> !b.controllerId().equals(controller.id()))
                                .toList());
        for (var row : rows) {
            var device = (Device) row.device.getSelectedItem();
            if (device.id() != null)
                bindings.add(
                        new FlightComputerControllerSettings.Binding(
                                controller.id(),
                                device.id(),
                                row.reverse.isSelected() ? -1 : 1,
                                value(row.trim),
                                value(row.min),
                                value(row.max),
                                row.offset.isSelected()));
        }
        return new FlightComputerControllerSettings(designId, bindings);
    }
}
