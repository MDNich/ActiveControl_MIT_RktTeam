package info.openrocket.swing.gui.simulation;

import info.openrocket.core.simulation.SimulationOptions;
import info.openrocket.core.simulation.ensemble.EnsembleSettings;
import net.miginfocom.swing.MigLayout;
import javax.swing.*;
import java.util.ArrayList;
import java.util.List;

/** Controls for uncertainty runs in the existing simulation editor. */
final class EnsembleOptionsPanel extends JPanel {
    EnsembleOptionsPanel(SimulationOptions options) {
        super(new MigLayout("fillx, insets 8", "[grow][130lp!]", ""));
        setBorder(BorderFactory.createTitledBorder("Ensemble simulation"));
        var s=options.getEnsembleSettings();
        JCheckBox enabled=new JCheckBox("Run multiple flights with variation",s.enabled());
        enabled.setName("ensemble.enabled"); add(enabled,"span, wrap");
        List<JSpinner> fields=new ArrayList<>();
        JSpinner runs=field("Number of runs",s.runs(),2,10000,1,"Each run is a complete flight. Failed runs stop the batch.",fields);
        JSpinner motor=field("Thrust noise σ (N)",s.motorSigma(),0,1e6,1,"Additive Gaussian noise: T(t) + e(t), independently for each motor. Negative thrust is clipped to zero.",fields);
        JSpinner interval=field("Noise sample interval (s)",s.noiseInterval(),.005,10,.005,"Independent Gaussian knots with variance-normalized interpolation. Shorter intervals produce faster fluctuations.",fields);
        JSpinner temperature=field("Temperature σ (K)",s.temperatureSigma(),0,50,.5,"One Gaussian launch-temperature offset per run; 1 K of variation equals 1 °C.",fields);
        JSpinner pressure=field("Pressure σ (%)",s.pressureSigma()*100,0,30,.5,"One Gaussian launch-pressure offset per run, as a percentage of the configured pressure.",fields);
        JSpinner wind=field("Wind component σ (m/s)",s.windSigma(),0,100,.1,"Independent East and North offsets per run, applied at all altitudes. Baseline turbulence stays fixed.",fields);
        JSpinner seed=field("Random seed",s.seed(),Integer.MIN_VALUE,Integer.MAX_VALUE,1,"Repeat this seed and configuration to repeat the ensemble.",fields);
        Runnable update=()-> {
            options.setEnsembleSettings(new EnsembleSettings(enabled.isSelected(),((Number)runs.getValue()).intValue(),number(motor),number(interval),number(temperature),number(pressure)/100,number(wind),((Number)seed.getValue()).intValue()));
            fields.forEach(f->f.setEnabled(enabled.isSelected()));
        };
        for (var f:fields) { f.addChangeListener(e->update.run()); f.setEnabled(s.enabled()); }
        enabled.addActionListener(e->update.run());
        add(new JLabel("<html>Set a σ to 0 to disable that source. Both sources → combined band.<br>2D: mean ±1σ; 3D: mean; distributions: individual flight summaries.<br>Saving simulation data includes the full results of every run.</html>"),"span, wrap");
    }
    private JSpinner field(String label, Number value, Comparable<?> min, Comparable<?> max, Number step, String tip,List<JSpinner> fields) {
        SpinnerNumberModel model = value instanceof Integer
                ? new SpinnerNumberModel(value.intValue(), ((Number)min).intValue(), ((Number)max).intValue(), step.intValue())
                : new SpinnerNumberModel(value.doubleValue(), ((Number)min).doubleValue(), ((Number)max).doubleValue(), step.doubleValue());
        var spinner=new JSpinner(model);
        spinner.setName("ensemble."+label);spinner.setToolTipText(tip);
        // NumberEditor carries the model bounds into the text formatter: invalid typed
        // values revert on focus loss instead of reaching the settings constructor.
        spinner.setEditor(new JSpinner.NumberEditor(spinner, value instanceof Integer ? "0" : "0.########"));
        var text=new JLabel(label);text.setToolTipText(tip);text.setLabelFor(spinner);
        // Override each editor's minimum width as well as the column width; the
        // full integer range otherwise makes the seed editor overflow its cell.
        add(text);add(spinner,"width 130lp!, growx, wrap");fields.add(spinner);return spinner;
    }
    private static double number(JSpinner s) { return ((Number)s.getValue()).doubleValue(); }
}
