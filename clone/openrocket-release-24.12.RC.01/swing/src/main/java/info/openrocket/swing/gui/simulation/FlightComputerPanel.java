package info.openrocket.swing.gui.simulation;

import edu.mit.rocket_team.zephyrus.telemetry.TelemetryLinkSettings;
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
        add(new JLabel(text("help")), "span, wrap");
        output.setEditable(false);
        output.setToolTipText(text("output"));
        add(output, "span, growx, wrap");
        refresh();
        enabled.addActionListener(e -> apply());
        for (JSpinner spinner : new JSpinner[]{loss, delay, seed}) spinner.addChangeListener(e -> apply());
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
            loss.setValue(link.packetLossFraction() * 100); delay.setValue(link.delayMs()); seed.setValue(link.randomSeed());
            setControlsEnabled();
            var path = simulation.getFlightComputerTelemetryPath();
            output.setText(path == null ? "" : path.toString()); output.setVisible(path != null);
        } finally { updating = false; }
    }

    private void setControlsEnabled() {
        for (JSpinner spinner : new JSpinner[]{loss, delay, seed}) spinner.setEnabled(enabled.isSelected());
    }

    private void apply() {
        if (updating) return;
        ZephyrusFlightComputer.apply(simulation, enabled.isSelected(), new TelemetryLinkSettings(
                ((Number) loss.getValue()).doubleValue() / 100,
                ((Number) delay.getValue()).intValue(), ((Number) seed.getValue()).intValue()));
        setControlsEnabled();
        changed.run();
    }
}
