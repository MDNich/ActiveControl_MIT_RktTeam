package info.openrocket.swing.gui.simulation;

import edu.mit.rocket_team.zephyrus.telemetry.TelemetryLinkSettings;
import edu.mit.rocket_team.zephyrus.telemetry.FlightComputerOutputSettings;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.startup.Application;
import info.openrocket.swing.gui.SpinnerEditor;
import net.miginfocom.swing.MigLayout;
import javax.swing.*;

/** Dedicated editor for the standard, persisted FC extension. */
final class FlightComputerPanel extends JPanel {
    private final Simulation simulation;
    private final Runnable changed;
    private final JCheckBox enabled = new JCheckBox(text("enabled"));
    private final JSpinner loss = new JSpinner(new SpinnerNumberModel(0.0, 0.0, 100.0, 1.0));
    private final JSpinner delay = new JSpinner(new SpinnerNumberModel(0, 0, 10_000, 10));
    private final JSpinner seed = new JSpinner(new SpinnerNumberModel(1, 0, Integer.MAX_VALUE, 1));
    private final JTextField output = new JTextField();
    private final JTextField csvFile = new JTextField(16), logFile = new JTextField(16);
    private final java.util.List<JButton> fileButtons = new java.util.ArrayList<>();
    private boolean updating;

    private static String text(String key) { return Application.getTranslator().get("FCSettings." + key); }

    FlightComputerPanel(Simulation simulation, Runnable changed) {
        super(new MigLayout("fillx, insets 8", "[grow][85lp!][]"));
        this.simulation = simulation;
        this.changed = changed;
        setBorder(BorderFactory.createTitledBorder(text("title")));
        enabled.setName("fc.enabled");
        add(enabled, "span, wrap");
        row("loss", loss, "%"); row("delay", delay, "ms"); row("seed", seed, "");
        add(new JLabel(text("help")), "span, wrap para");
        fileRow("csvFile", csvFile, "csv");
        fileRow("logFile", logFile, "log");
        add(new JLabel(text("automaticFiles")), "span, wrap");
        output.setEditable(false);
        output.setToolTipText(text("output"));
        add(output, "span, growx, wrap");
        refresh();
        enabled.addActionListener(e -> apply());
        for (JSpinner spinner : new JSpinner[]{loss, delay, seed}) spinner.addChangeListener(e -> apply());
    }

    private void fileRow(String key, JTextField field, String extension) {
        JLabel label = new JLabel(text(key)); label.setLabelFor(field);
        add(label, "span, wrap");
        field.setName("fc." + key); field.setToolTipText(text("path.tip"));
        add(field, "span 2, growx");
        JButton browse = new JButton(text("browse")); fileButtons.add(browse);
        browse.addActionListener(e -> {
            JFileChooser chooser = new JFileChooser(); chooser.setDialogTitle(text(key));
            chooser.setFileFilter(new javax.swing.filechooser.FileNameExtensionFilter(extension.toUpperCase(java.util.Locale.ROOT), extension));
            if (!field.getText().isBlank()) chooser.setSelectedFile(new java.io.File(field.getText()));
            else chooser.setSelectedFile(new java.io.File(extension.equals("csv") ? "telemetry.csv" : "OR.log"));
            if (chooser.showSaveDialog(this) != JFileChooser.APPROVE_OPTION) return;
            String path = chooser.getSelectedFile().getAbsolutePath();
            if (!path.toLowerCase(java.util.Locale.ROOT).endsWith("." + extension)) path += "." + extension;
            field.setText(path); apply();
        });
        add(browse, "wrap");
        field.addActionListener(e -> apply());
        field.addFocusListener(new java.awt.event.FocusAdapter() {
            @Override public void focusLost(java.awt.event.FocusEvent e) { if (!e.isTemporary()) apply(); }
        });
    }

    private void row(String key, JSpinner spinner, String unit) {
        JLabel label = new JLabel(text(key)); label.setLabelFor(spinner);
        spinner.setName("fc." + key); spinner.setEditor(new SpinnerEditor(spinner));
        spinner.setToolTipText(text(key + ".tip"));
        spinner.getAccessibleContext().setAccessibleName(text(key));
        add(label); add(spinner, "growx"); add(new JLabel(unit), "wrap");
    }

    void refresh() {
        updating = true;
        try {
            var fc = ZephyrusFlightComputer.read(simulation);
            var link = fc.getLinkSettings();
            enabled.setSelected(fc.isEnabled());
            csvFile.setText(fc.getOutputSettings().csvFile()); logFile.setText(fc.getOutputSettings().logFile());
            loss.setValue(link.packetLossFraction() * 100); delay.setValue(link.delayMs()); seed.setValue(link.randomSeed());
            setControlsEnabled();
            var path = simulation.getFlightComputerTelemetryPath();
            output.setText(path == null ? "" : path.toString()); output.setVisible(path != null);
        } finally { updating = false; }
    }

    private void setControlsEnabled() {
        for (JSpinner spinner : new JSpinner[]{loss, delay, seed}) spinner.setEnabled(enabled.isSelected());
        csvFile.setEnabled(enabled.isSelected()); logFile.setEnabled(enabled.isSelected());
        for (JButton button : fileButtons) button.setEnabled(enabled.isSelected());
    }

    private void apply() {
        if (updating) return;
        try {
            var link = new TelemetryLinkSettings(((Number) loss.getValue()).doubleValue() / 100,
                    ((Number) delay.getValue()).intValue(), ((Number) seed.getValue()).intValue());
            var files = new FlightComputerOutputSettings(csvFile.getText(), logFile.getText());
            var old = ZephyrusFlightComputer.read(simulation);
            if (old.isEnabled() != enabled.isSelected() || !old.getLinkSettings().equals(link) || !old.getOutputSettings().equals(files)) {
                ZephyrusFlightComputer.apply(simulation, enabled.isSelected(), link, files);
                changed.run();
            }
            setControlsEnabled();
        } catch (IllegalArgumentException error) {
            JOptionPane.showMessageDialog(this, error.getMessage(), text("title"), JOptionPane.ERROR_MESSAGE);
            refresh();
        }
    }
}
