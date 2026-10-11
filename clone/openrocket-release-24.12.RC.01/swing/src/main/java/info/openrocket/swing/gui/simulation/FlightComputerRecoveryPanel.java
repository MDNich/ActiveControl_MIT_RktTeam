package info.openrocket.swing.gui.simulation;

import info.openrocket.core.document.Simulation;
import info.openrocket.core.rocketcomponent.RecoveryDevice;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.simulation.flightcomputer.FlightComputerRecoverySettings;
import net.miginfocom.swing.MigLayout;
import javax.swing.*;
import javax.swing.table.AbstractTableModel;
import java.util.*;

/** Per-simulation wiring to rocket hardware, deliberately kept out of portable .fc designs. */
final class FlightComputerRecoveryPanel extends JPanel {
    private final Simulation simulation;
    private final Runnable changed;
    private final List<Row> rows = new ArrayList<>();
    private final Model model = new Model();
    private final JTable table =
            new JTable(model) {
                /**
                 * Returns the contextual tooltip for the pointed-at table cell.
                 *
                 * @param event event to process
                 * @return the contextual tooltip for the pointed-at table cell
                 */
                @Override
                public String getToolTipText(java.awt.event.MouseEvent event) {
                    int row = rowAtPoint(event.getPoint());
                    if (row < 0) return null;
                    var item = rows.get(convertRowIndexToModel(row));
                    return item.name + " (" + item.id + ")";
                }
            };
    private final JLabel authority = new JLabel();
    private final JButton clear = new JButton("Clear selected assignment");

    private record Row(
            UUID id, String name, RecoveryDevice device, Set<Integer> channels, long delayUs) {}

    /**
     * Builds the flight computer recovery panel and connects its controls to the supplied model.
     *
     * @param simulation simulation being configured or executed
     * @param changed initial changed
     */
    FlightComputerRecoveryPanel(Simulation simulation, Runnable changed) {
        super(new MigLayout("fillx, insets 8", "[grow]"));
        this.simulation = simulation;
        this.changed = changed;
        setBorder(BorderFactory.createTitledBorder("Recovery"));
        authority.setName("fc.recovery.authority");
        add(authority, "growx, wrap");
        var help =
                new JTextArea(
                        "Assign channels to parachutes or streamers. Unassigned devices remain stowed when FC is enabled. All Java boards share channels 0–5. Delay is measured from the FC firing command, rounded to the nearest microsecond.");
        help.setEditable(false);
        help.setLineWrap(true);
        help.setWrapStyleWord(true);
        help.setOpaque(false);
        help.setFont(UIManager.getFont("Label.font"));
        add(help, "growx, wmin 0, wrap");
        table.setName("fc.recovery.mappings");
        table.setRowHeight(Math.max(25, table.getRowHeight()));
        table.putClientProperty("terminateEditOnFocusLost", true);
        table.setAutoResizeMode(JTable.AUTO_RESIZE_OFF);
        table.getColumnModel().getColumn(0).setPreferredWidth(230);
        for (int c = 1; c <= 6; c++) table.getColumnModel().getColumn(c).setPreferredWidth(32);
        table.getColumnModel().getColumn(7).setPreferredWidth(100);
        table.getColumnModel().getColumn(8).setPreferredWidth(160);
        table.getColumnModel().getColumn(7).setCellEditor(new DelayEditor());
        var scroll = new JScrollPane(table);
        scroll.setColumnHeaderView(table.getTableHeader());
        add(scroll, "growx, wmin 0, h 130lp:170lp:240lp, wrap");
        clear.setName("fc.recovery.clear");
        clear.addActionListener(
                e -> {
                    if (table.isEditing() && !table.getCellEditor().stopCellEditing()) return;
                    int index = table.getSelectedRow();
                    if (index < 0) return;
                    var row = rows.get(table.convertRowIndexToModel(index));
                    rows.set(
                            table.convertRowIndexToModel(index),
                            new Row(row.id, row.name, row.device, Set.of(), 0));
                    save();
                    refresh();
                });
        add(clear, "wrap");
        refresh();
    }

    /** Reloads recovery-device choices and the saved channel assignments. */
    void refresh() {
        rows.clear();
        var fc = ZephyrusFlightComputer.read(simulation);
        var bindings = new LinkedHashMap<UUID, FlightComputerRecoverySettings.Binding>();
        for (var b : fc.getRecoverySettings().bindings()) bindings.put(b.deviceId(), b);
        for (var c : simulation.getRocket())
            if (c instanceof RecoveryDevice device) {
                var b = bindings.remove(c.getID());
                String name =
                        c.getName()
                                + " — "
                                + (c.getParent() == null ? "" : c.getParent().getName());
                rows.add(
                        new Row(
                                c.getID(),
                                name,
                                device,
                                b == null ? Set.of() : b.channels(),
                                b == null ? 0 : b.delayUs()));
            }
        for (var b : bindings.values())
            rows.add(
                    new Row(
                            b.deviceId(),
                            "Missing: " + b.deviceId(),
                            null,
                            b.channels(),
                            b.delayUs()));
        model.fireTableDataChanged();
        setActive(fc.isEnabled());
    }

    /**
     * Updates whether FC recovery assignment controls are enabled.
     *
     * @param enabled whether this capability is enabled
     */
    void setActive(boolean enabled) {
        authority.setText(
                enabled
                        ? "FC controls all recovery — OpenRocket deployment rules are ignored."
                        : "FC disabled — saved OpenRocket deployment rules apply.");
        table.setEnabled(enabled);
        clear.setEnabled(enabled);
    }

    /** Commits the edited recovery mappings to the simulation extension. */
    private void save() {
        var bindings = new ArrayList<FlightComputerRecoverySettings.Binding>();
        for (var row : rows)
            if (!row.channels.isEmpty() || row.delayUs != 0)
                bindings.add(
                        new FlightComputerRecoverySettings.Binding(
                                row.id, row.channels, row.delayUs));
        var settings = new FlightComputerRecoverySettings(bindings);
        if (!settings.equals(ZephyrusFlightComputer.read(simulation).getRecoverySettings())) {
            ZephyrusFlightComputer.applyRecovery(simulation, settings);
            changed.run();
        }
    }

    private final class Model extends AbstractTableModel {
        /**
         * Returns the number of rows in the current table model.
         *
         * @return the number of rows in the current table model
         */
        public int getRowCount() {
            return rows.size();
        }

        /**
         * Returns the number of columns in this table model.
         *
         * @return the number of columns in this table model
         */
        public int getColumnCount() {
            return 9;
        }

        /**
         * Returns the display heading for a table column.
         *
         * @param c selected channel or controller
         * @return the display heading for a table column
         */
        public String getColumnName(int c) {
            return c == 0
                    ? "Recovery device"
                    : c <= 6 ? Integer.toString(c - 1) : c == 7 ? "Delay (ms)" : "Status";
        }

        /**
         * Returns the value type used to choose a column renderer and editor.
         *
         * @param c selected channel or controller
         * @return the value type used to choose a column renderer and editor
         */
        public Class<?> getColumnClass(int c) {
            return c > 0 && c <= 6 ? Boolean.class : c == 7 ? Double.class : String.class;
        }

        /**
         * Tests whether the selected table cell can be edited.
         *
         * @param r r
         * @param c selected channel or controller
         * @return true when the condition described above holds
         */
        public boolean isCellEditable(int r, int c) {
            return table.isEnabled() && c > 0 && c < 8 && rows.get(r).device != null;
        }

        /**
         * Returns the model value at the requested table row and column.
         *
         * @param r r
         * @param c selected channel or controller
         * @return the model value at the requested table row and column
         */
        public Object getValueAt(int r, int c) {
            var row = rows.get(r);
            if (c == 0) return row.name;
            if (c <= 6) return row.channels.contains(c - 1);
            if (c == 7) return row.delayUs / 1000.0;
            if (row.device == null) return "Missing — clear assignment";
            if (!simulation.getActiveConfiguration().isComponentActive(row.device))
                return "Inactive configuration";
            return row.channels.isEmpty() ? "Unassigned — stowed" : "Assigned";
        }

        /**
         * Applies an edited table value to the corresponding model entry.
         *
         * @param value replacement or input value
         * @param r r
         * @param c selected channel or controller
         */
        public void setValueAt(Object value, int r, int c) {
            var row = rows.get(r);
            var channels = new TreeSet<>(row.channels);
            long delay = row.delayUs;
            if (c <= 6) {
                if (Boolean.TRUE.equals(value)) channels.add(c - 1);
                else channels.remove(c - 1);
            } else
                delay =
                        FlightComputerRecoverySettings.delayFromMilliseconds(
                                ((Number) value).doubleValue());
            rows.set(r, new Row(row.id, row.name, row.device, channels, delay));
            save();
            fireTableRowsUpdated(r, r);
        }
    }

    private static final class DelayEditor extends AbstractCellEditor
            implements javax.swing.table.TableCellEditor {
        private final JSpinner spinner =
                new JSpinner(new SpinnerNumberModel(0.0, 0.0, 86_400_000.0, 0.1));

        /** Creates a DelayEditor value from the supplied . */
        DelayEditor() {
            spinner.setEditor(new JSpinner.NumberEditor(spinner, "0.###"));
            ((JSpinner.NumberEditor) spinner.getEditor())
                    .getTextField()
                    .addActionListener(e -> stopCellEditing());
        }

        /**
         * Loads the selected table value into its cell editor.
         *
         * @param table table being edited or rendered
         * @param value replacement or input value
         * @param selected whether this item is selected
         * @param row zero-based row index
         * @param col col
         * @return the resulting Component
         */
        public java.awt.Component getTableCellEditorComponent(
                JTable table, Object value, boolean selected, int row, int col) {
            spinner.setValue(value);
            return spinner;
        }

        /**
         * Returns the current cell editor value.
         *
         * @return the current cell editor value
         */
        public Object getCellEditorValue() {
            return spinner.getValue();
        }

        /**
         * Validates and commits the current cell editor value before editing ends.
         *
         * @return true when the condition described above holds
         */
        public boolean stopCellEditing() {
            try {
                spinner.commitEdit();
                return super.stopCellEditing();
            } catch (java.text.ParseException e) {
                java.awt.Toolkit.getDefaultToolkit().beep();
                return false;
            }
        }
    }
}
