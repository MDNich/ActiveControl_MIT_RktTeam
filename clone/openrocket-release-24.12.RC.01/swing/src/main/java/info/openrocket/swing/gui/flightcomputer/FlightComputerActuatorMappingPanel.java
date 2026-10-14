package info.openrocket.swing.gui.flightcomputer;

import info.openrocket.core.document.Simulation;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.flightcomputer.*;
import javax.swing.*;
import java.awt.*;
import java.util.*;
import java.util.List;

/** Assignments stay local until applied; the explicit Add tabs action edits the rocket with Undo. */
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
    private boolean refreshingDevices;

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
        if (controller.mode() == FlightComputerController.Mode.TABS) addTabCreationControls();
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
        fillDevices(row, binding == null ? null : binding.deviceId());
        row.panel.add(row.device, "span,growx,wmin 0");
        row.device.addActionListener(e -> { if (!refreshingDevices) changed.run(); });
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

    private void fillDevices(Row row, UUID selected) {
        refreshingDevices = true;
        try {
            row.device.removeAllItems();
            row.device.addItem(new Device(null, "Recorded only / no physical target"));
            var active = simulation.getActiveConfiguration();
            boolean found = selected == null;
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
                    if (selected != null && selected.equals(item.id)) {
                        row.device.setSelectedItem(item);
                        found = true;
                    }
                }
            if (!found) {
                var missing = new Device(selected, "Missing: " + selected);
                row.device.addItem(missing);
                row.device.setSelectedItem(missing);
            }
        } finally { refreshingDevices = false; }
    }

    private void addTabCreationControls() {
        var tools = new JPanel(new net.miginfocom.swing.MigLayout("fillx, insets 5, wrap 2", "[grow,fill][]"));
        var note = new JTextArea("Fin tabs need a roll-control fin set, not through-the-wall mounting tabs. "
                + "Select an existing target below, or add control tabs to trapezoidal fins here.");
        note.setLineWrap(true); note.setWrapStyleWord(true); note.setEditable(false); note.setOpaque(false);
        note.setColumns(30); note.setRows(3);
        tools.add(note, "span,growx");
        var candidates = new JComboBox<Device>();
        candidates.setName("fc.controller.tabCandidate");
        for (var c : simulation.getRocket())
            if (c instanceof TrapezoidFinSet && !(c instanceof TabControlledTrapezoidFinSet)
                    && simulation.getActiveConfiguration().isComponentActive(c))
                candidates.addItem(new Device(c.getID(), c.getName()));
        tools.add(candidates, "wmin 0");
        var convert = new JButton("Add roll-control tabs");
        convert.setName("fc.controller.createTabs");
        convert.setEnabled(candidates.getItemCount() > 0);
        convert.setToolTipText("Changes the rocket, with Undo. Review tab geometry and lift slope in the fin editor.");
        tools.add(convert);
        var explanation = new JTextArea("New tabs: 25% span, 20% chord, centered on the trailing edge; "
                + "initial lift slope 6.283 /rad (editable approximation). This changes the rocket; Undo is available.");
        explanation.setLineWrap(true); explanation.setWrapStyleWord(true); explanation.setEditable(false);
        explanation.setOpaque(false); explanation.setColumns(30); explanation.setRows(3);
        tools.add(explanation, "span,growx");
        convert.addActionListener(event -> {
            var selected = (Device) candidates.getSelectedItem();
            if (selected == null) return;
            var component = simulation.getRocket().findComponent(selected.id());
            if (!(component instanceof TrapezoidFinSet plain) || component instanceof TabControlledTrapezoidFinSet) return;
            if (simulation.getDocument() != null) simulation.getDocument().addUndoPosition("Add roll-control tabs");
            double chord = .2 * Math.max(1e-6, Math.min(plain.getRootChord(),
                    plain.getTipChord() > 0 ? plain.getTipChord() : plain.getRootChord() / 2));
            var fin = new TabControlledTrapezoidFinSet(plain);
            double edge = fin.getFinPoints()[2].sub(fin.getFinPoints()[3]).length();
            fin.setTabShape(edge * .25, chord, edge * .375, 0);
            fin.setCNALPHA(2 * Math.PI);
            for (var row : rows) {
                var current = (Device) row.device.getSelectedItem();
                fillDevices(row, current == null ? null : current.id());
            }
            var target = rows.stream()
                    .filter(row -> ((Device) row.device.getSelectedItem()).id() == null)
                    .findFirst().orElseGet(() -> {
                        addRow(null);
                        return rows.get(rows.size() - 1);
                    });
            for (int i = 0; i < target.device.getItemCount(); i++)
                if (fin.getID().equals(target.device.getItemAt(i).id())) target.device.setSelectedIndex(i);
            candidates.removeItem(selected);
            convert.setEnabled(candidates.getItemCount() > 0);
            System.out.println("FC_CONTROLLER added roll-control tabs component=" + fin.getID() + " name=" + fin.getName());
            changed.run();
        });
        add(tools, BorderLayout.NORTH);
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
            if (device != null && device.id() != null)
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
