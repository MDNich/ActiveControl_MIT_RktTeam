package info.openrocket.swing.gui.simulation;

import edu.mit.rocket_team.zephyrus.telemetry.TelemetryLinkSettings;
import edu.mit.rocket_team.zephyrus.FC.FlightComputerTimingSettings;
import edu.mit.rocket_team.zephyrus.telemetry.FlightComputerOutputSettings;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.startup.Application;
import net.miginfocom.swing.MigLayout;
import info.openrocket.core.simulation.flightcomputer.*;
import info.openrocket.swing.gui.flightcomputer.FlightComputerDesigner;
import javax.swing.*;

/** Dedicated editor for the standard, persisted FC extension. */
final class FlightComputerPanel extends JPanel {
    private final Simulation simulation;
    private final info.openrocket.core.document.OpenRocketDocument document;
    private final Runnable changed;
    private final JCheckBox enabled = new JCheckBox(text("enabled"));
    private final JSpinner loss = new JSpinner(new SpinnerNumberModel(0.0, 0.0, 100.0, 1.0));
    private final JSpinner delay = new JSpinner(new SpinnerNumberModel(0, 0, 10_000, 10));
    private final JSpinner seed = new JSpinner(new SpinnerNumberModel(1, 0, Integer.MAX_VALUE, 1));
    private final JSpinner sensorRead = new JSpinner(new SpinnerNumberModel(3.0,0.0,1000.0,0.1));
    private final JSpinner work = new JSpinner(new SpinnerNumberModel(0.0,0.0,1000.0,0.1));
    private final JSpinner jitter = new JSpinner(new SpinnerNumberModel(0.0,0.0,1000.0,0.1));
    private final JSpinner phase = new JSpinner(new SpinnerNumberModel(0.0,0.0,19.999,0.1));
    private final JSpinner timingSeed = new JSpinner(new SpinnerNumberModel(1,0,Integer.MAX_VALUE,1));
    private final JTextField output = new JTextField();
    private final JTextField csvFile = new JTextField(16), logFile = new JTextField(16);
    private final java.util.List<JButton> fileButtons = new java.util.ArrayList<>();
    private boolean updating;
    private final JComboBox<String> computer=new JComboBox<>(new String[]{"Built-in controller","Iris — unavailable","Balius — unavailable"});
    private final JComboBox<LibraryItem> designs=new JComboBox<>();
    private final JLabel designStatus=new JLabel();
    private record LibraryItem(java.nio.file.Path path,String name){public String toString(){return name;}}
    private final java.util.List<JComponent> designControls=new java.util.ArrayList<>();

    private static String text(String key) { return Application.getTranslator().get("FCSettings." + key); }

    FlightComputerPanel(Simulation simulation,Runnable changed){this(null,simulation,changed);}
    FlightComputerPanel(info.openrocket.core.document.OpenRocketDocument document,Simulation simulation, Runnable changed) {
        super(new MigLayout("fillx, insets 8", "[grow][130lp!][]"));
        this.document=document;
        this.simulation = simulation;
        this.changed = changed;
        setBorder(BorderFactory.createTitledBorder(text("title")));
        enabled.setName("fc.enabled");
        add(enabled, "span, wrap");
        add(new JLabel("Computer"));add(computer,"span 2, growx, wrap");computer.setName("fc.computer");
        computer.addActionListener(e->{if(!updating&&computer.getSelectedIndex()!=0){JOptionPane.showMessageDialog(this,"This computer's firmware model is not available yet.");computer.setSelectedIndex(0);}});
        add(new JLabel("Design"));add(designs,"span 2, growx, wrap");designs.setName("fc.design");
        var buttons=new JPanel(new java.awt.FlowLayout(java.awt.FlowLayout.LEFT,4,0));
        for(String label:new String[]{"Load .fc…","Edit…","New…","Library…"}){var button=new JButton(label);buttons.add(button);designControls.add(button);button.addActionListener(e->designAction(label));}
        add(buttons,"span, wrap");add(designStatus,"span, growx, wrap para");
        designs.addActionListener(e->{if(!updating&&designs.getSelectedItem() instanceof LibraryItem item&&item.path()!=null)selectDesign(item.path());});
        row("loss", loss, "%"); row("delay", delay, "ms"); row("seed", seed, "");
        add(new JLabel(text("help")), "span, wrap para");
        add(new JLabel(text("timing")), "span, wrap");
        row("sensorRead", sensorRead, "ms"); row("work", work, "ms"); row("jitter", jitter, "ms");
        row("phase", phase, "ms"); row("timingSeed", timingSeed, "");
        add(new JLabel(text("timingHelp")), "span, wrap para");
        fileRow("csvFile", csvFile, "csv");
        fileRow("logFile", logFile, "log");
        add(new JLabel(text("automaticFiles")), "span, wrap");
        output.setEditable(false);
        output.setToolTipText(text("output"));
        add(output, "span, growx, wrap");
        var importLog=new JButton("Import FC log as result…");importLog.setName("fc.importLog");importLog.setEnabled(document!=null);importLog.addActionListener(e->importLog());add(importLog,"span, wrap");
        refresh();
        enabled.addActionListener(e -> apply());
        for (JSpinner spinner : new JSpinner[]{loss, delay, seed, sensorRead, work, jitter, phase, timingSeed}) spinner.addChangeListener(e -> apply());
    }

    private void refreshLibrary(ZephyrusFlightComputer fc) {
        designs.removeAllItems();
        if(!fc.hasDesignFile())designs.addItem(new LibraryItem(null,"Built-in settings (no .fc selected)"));
        try {
            java.nio.file.Path selected=fc.hasDesignFile()?FlightComputerLibrary.resolve(fc.getDesignReference()):null;
            for(var path:FlightComputerLibrary.list()){
                try{var d=FlightComputerDesign.read(path);var item=new LibraryItem(path,d.name()+" — "+path.getFileName());designs.addItem(item);if(path.equals(selected))designs.setSelectedItem(item);}catch(java.io.IOException ignored){}
            }
            if(selected!=null){var d=FlightComputerDesign.read(selected);computer.setSelectedIndex(d.computer().equals("Iris")?1:d.computer().equals("Balius")?2:0);long errors=d.diagnostics().stream().filter(FlightComputerDesign.Diagnostic::error).count();designStatus.setText(errors>0?errors+" design errors — open Edit":"External .fc · edit timing and hardware in designer");}
            else designStatus.setText("Load or create a .fc design to use the graphical editor");
        }catch(Exception e){designStatus.setText("Missing / invalid .fc — use Load to locate it");}
        designStatus.setToolTipText(FlightComputerLibrary.directory().toString());
    }
    private void selectDesign(java.nio.file.Path path){
        try{ZephyrusFlightComputer.selectFile(simulation,path);changed.run();refresh();}
        catch(Exception e){JOptionPane.showMessageDialog(this,e.getMessage(),"Flight computer",JOptionPane.ERROR_MESSAGE);}
    }
    private void designAction(String action){
        try{
            if(action.equals("Load .fc…")){
                var chooser=new JFileChooser();chooser.setDialogTitle("Import flight computer into the library");chooser.setFileFilter(new javax.swing.filechooser.FileNameExtensionFilter("Flight computer (*.fc)","fc"));
                if(chooser.showOpenDialog(this)==JFileChooser.APPROVE_OPTION)selectDesign(chooser.getSelectedFile().toPath());
            }else if(action.equals("Library…")){
                FlightComputerLibrary.template();java.awt.Desktop.getDesktop().open(FlightComputerLibrary.directory().toFile());
            }else{
                var fc=ZephyrusFlightComputer.read(simulation);java.nio.file.Path path;
                if(action.equals("New…")){
                    String name=JOptionPane.showInputDialog(this,"Design name","Flight computer design");if(name==null||name.isBlank())return;
                    var d=FlightComputerDesign.zephyrus().withTiming(fc.getTimingSettings()).copy(name);
                    path=FlightComputerLibrary.directory().resolve(name.replaceAll("[^A-Za-z0-9._-]","-")+"-"+d.id().substring(0,8)+".fc");d.write(path);selectDesign(path);
                }else if(fc.hasDesignFile())path=FlightComputerLibrary.resolve(fc.getDesignReference());
                else{
                    path=FlightComputerLibrary.template();
                    // Opening an old simulation remains read-only. Selecting a file is explicit.
                }
                FlightComputerDesigner.open(this,path,simulation,this::selectDesign);
            }
        }catch(Exception e){JOptionPane.showMessageDialog(this,e.getMessage(),"Flight computer",JOptionPane.ERROR_MESSAGE);}
    }

    private void importLog(){
        var chooser=new JFileChooser();chooser.setDialogTitle("Import an OpenRocket FC action log");chooser.setFileFilter(new javax.swing.filechooser.FileNameExtensionFilter("OpenRocket log (*.log, *.txt)","log","txt"));
        if(chooser.showOpenDialog(this)!=JFileChooser.APPROVE_OPTION)return;var path=chooser.getSelectedFile().toPath();
        new SwingWorker<java.util.List<FlightComputerLogReader.Result>,Void>(){
            protected java.util.List<FlightComputerLogReader.Result> doInBackground()throws Exception{return FlightComputerLogReader.read(path);}
            protected void done(){try{
                var results=get();Simulation first=null;for(var result:results){var imported=new Simulation(document,simulation.getRocket(),Simulation.Status.EXTERNAL,result.name(),simulation.getOptions().clone(),java.util.List.of(),result.data());document.addSimulation(imported);if(first==null)first=imported;}
                JOptionPane.showMessageDialog(FlightComputerPanel.this,results.size()+" imported result(s) added to the simulation list.\n"+results.get(0).note());
                var dialog=new SimulationConfigDialog(SwingUtilities.getWindowAncestor(FlightComputerPanel.this),document,false,first);dialog.switchToPlotTab();dialog.setVisible(true);
            }catch(Exception ex){JOptionPane.showMessageDialog(FlightComputerPanel.this,ex.getCause()==null?ex.getMessage():ex.getCause().getMessage(),"FC log import",JOptionPane.ERROR_MESSAGE);}}
        }.execute();
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
        spinner.setName("fc." + key); spinner.setEditor(new JSpinner.NumberEditor(spinner, spinner==seed || spinner==timingSeed || spinner==delay ? "0" : "0.###"));
        ((JSpinner.NumberEditor)spinner.getEditor()).getTextField().setColumns(8);
        spinner.setToolTipText(text(key + ".tip"));
        spinner.getAccessibleContext().setAccessibleName(text(key));
        add(label); add(spinner, "width 130lp!, growx"); add(new JLabel(unit), "wrap");
    }

    void refresh() {
        updating = true;
        try {
            var fc = ZephyrusFlightComputer.read(simulation);
            var link = fc.getLinkSettings();
            enabled.setSelected(fc.isEnabled());
            refreshLibrary(fc);
            csvFile.setText(fc.getOutputSettings().csvFile()); logFile.setText(fc.getOutputSettings().logFile());
            loss.setValue(link.packetLossFraction() * 100); delay.setValue(link.delayMs()); seed.setValue(link.randomSeed());
            var timing=fc.getTimingSettings();
            sensorRead.setValue(timing.sensorReadUs()/1000.0); work.setValue(timing.extraWorkUs()/1000.0);
            jitter.setValue(timing.workJitterUs()/1000.0); phase.setValue(timing.pwmPhaseUs()/1000.0);
            timingSeed.setValue(timing.randomSeed());
            setControlsEnabled();
            var path = simulation.getFlightComputerTelemetryPath();
            output.setText(path == null ? "" : path.toString()); output.setVisible(path != null);
        } finally { updating = false; }
    }

    private void setControlsEnabled() {
        for (JSpinner spinner : new JSpinner[]{loss, delay, seed, sensorRead, work, jitter, phase, timingSeed}) spinner.setEnabled(enabled.isSelected());
        if(ZephyrusFlightComputer.read(simulation).hasDesignFile())for(JSpinner spinner:new JSpinner[]{sensorRead,work,jitter,phase,timingSeed})spinner.setEnabled(false);
        computer.setEnabled(enabled.isSelected());designs.setEnabled(enabled.isSelected());
        csvFile.setEnabled(enabled.isSelected()); logFile.setEnabled(enabled.isSelected());
        for (JButton button : fileButtons) button.setEnabled(enabled.isSelected());
    }

    private static int microseconds(JSpinner spinner) { return (int)Math.round(((Number)spinner.getValue()).doubleValue()*1000); }

    private void apply() {
        if (updating) return;
        try {
            var link = new TelemetryLinkSettings(((Number) loss.getValue()).doubleValue() / 100,
                    ((Number) delay.getValue()).intValue(), ((Number) seed.getValue()).intValue());
            var files = new FlightComputerOutputSettings(csvFile.getText(), logFile.getText());
            var timing=new FlightComputerTimingSettings(microseconds(sensorRead),microseconds(work),microseconds(jitter),
                    microseconds(phase),((Number)timingSeed.getValue()).intValue());
            var old = ZephyrusFlightComputer.read(simulation);
            if (old.isEnabled() != enabled.isSelected() || !old.getLinkSettings().equals(link) || !old.getOutputSettings().equals(files) || !old.getTimingSettings().equals(timing)) {
                ZephyrusFlightComputer.apply(simulation, enabled.isSelected(), link, files, timing);
                changed.run();
            }
            setControlsEnabled();
        } catch (IllegalArgumentException error) {
            JOptionPane.showMessageDialog(this, error.getMessage(), text("title"), JOptionPane.ERROR_MESSAGE);
            refresh();
        }
    }
}
