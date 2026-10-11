package info.openrocket.swing.gui.simulation;

import java.awt.*;
import java.awt.event.*;
import java.nio.file.Path;
import java.util.*;
import java.util.List;
import javax.swing.*;
import javax.swing.table.*;
import net.miginfocom.swing.MigLayout;
import info.openrocket.core.document.*;
import info.openrocket.core.motor.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.motorbatch.*;
import info.openrocket.core.startup.Application;
import info.openrocket.swing.gui.dialogs.motor.thrustcurve.PreferredMotorCurve;
import info.openrocket.swing.gui.main.BasicFrame;
import static info.openrocket.core.simulation.motorbatch.MotorBatchReport.*;

/** Desktop motor search. Background workers publish immutable rows, never live simulation objects. */
public final class MotorBatchDialog extends JDialog {
    private final OpenRocketDocument document;
    private final Simulation source;
    private final List<ThrustCurveMotor> catalogue;
    private final JComboBox<MountChoice> mount=new JComboBox<>();
    private final JList<String> manufacturers;
    private final JTextField diameter=new JTextField(8),length=new JTextField(8),search=new JTextField(12);
    private final JComboBox<String> motorType=new JComboBox<>(new String[]{"ALL","SINGLE","RELOAD","HYBRID","UNKNOWN"});
    private final JComboBox<String> certification=new JComboBox<>(new String[]{"ALL","BELOW_L1","L1","L2","L3","REVIEW"});
    private final JCheckBox unavailable=new JCheckBox("Hide motors marked unavailable");
    private final JCheckBox ignoreIntersections=new JCheckBox("Ignore component intersections");
    private final JSpinner threads=new JSpinner(new SpinnerNumberModel(4,1,256,1));
    private final JComboBox<String> delay=new JComboBox<>(new String[]{"Keep baseline delay","Choose best listed delay"});
    private final JTextField[] impulse=range(),thrust=range(),burn=range();
    private final Map<MotorBatchMetrics.Metric,JTextField[]> criteria=new EnumMap<>(MotorBatchMetrics.Metric.class);
    private final JButton run=new JButton("Run batch"),cancel=new JButton("Cancel"),export=new JButton("Export CSV…"),shortlist=new JButton("Create simulations from selected rows");
    private final JLabel count=new JLabel("Choose a motor mount"),status=new JLabel("Single flight per motor/delay; ensemble variation is not applied.");
    private final JProgressBar progress=new JProgressBar();
    private final Model model=new Model();
    private final JTable table=new JTable(model);
    private final JTextArea details=new JTextArea(5,30);
    private final JComboBox<String> view=new JComboBox<>(new String[]{"All","Meets criteria","Does not meet criteria","Needs review"});
    private final JComboBox<MotorBatchReport> history=new JComboBox<>();
    private final JPanel settingsPanel=new SettingsPanel();
    private MotorBatchRunner runner;
    private SwingWorker<MotorBatchReport,MotorBatchRunner.Progress> worker;
    private MotorBatchReport report;
    private boolean closeAfterCancel,loadingHistory;
    private record MountChoice(UUID id,String label,double diameter,double length) { @Override public String toString(){return label;} }

    public MotorBatchDialog(Window parent,OpenRocketDocument document,Simulation source) {
        super(parent,"Batch simulate motors",ModalityType.APPLICATION_MODAL);
        this.document=document;this.source=source;
        catalogue=Application.getThrustCurveMotorSetDatabase().getMotorSets().stream().filter(s->s.getMotorCount()>0)
                .map(PreferredMotorCurve::select).sorted(Comparator.comparing((ThrustCurveMotor m)->m.getManufacturer().getDisplayName())
                        .thenComparing((ThrustCurveMotor m)->m.getDesignation()).thenComparing(ThrustCurveMotor::getDigest)).toList();
        manufacturers=new JList<>(catalogue.stream().map(m->m.getManufacturer().getDisplayName()).distinct().sorted().toArray(String[]::new));
        manufacturers.setVisibleRowCount(5);manufacturers.setSelectionInterval(0,Math.max(0,manufacturers.getModel().getSize()-1));
        for(var c:source.getActiveConfiguration().getActiveComponents())if(c instanceof MotorMount m && m.isMotorMount()) {
            var motor=m.getMotorConfig(source.getFlightConfigurationId()).getMotor();
            mount.addItem(new MountChoice(c.getID(),c.getName()+" ("+m.getMotorCountIncludingAssemblyCopies()+" motor(s))",
                    motor==null?Math.rint(m.getMotorMountDiameter()*1000)/1000:motor.getDiameter(),m.getLength()+m.getMotorOverhang()));
        }
        build();mount.addActionListener(e->mountChanged());mountChanged();
        setDefaultCloseOperation(DO_NOTHING_ON_CLOSE);
        addWindowListener(new WindowAdapter(){@Override public void windowClosing(WindowEvent e){close();}});
        setMinimumSize(new Dimension(1000,600));setSize(1250,800);setLocationRelativeTo(parent);
        refreshHistory();refreshPreview();
    }
    private static JTextField[] range(){return new JTextField[]{new JTextField(7),new JTextField(7)};}
    private void build() {
        setLayout(new BorderLayout(8,8));
        var heading=new JPanel(new MigLayout("fillx","[][grow,fill][]"));
        heading.add(new JLabel("Baseline: "+source.getName()),"span,wrap");
        heading.add(new JLabel("Saved batches:"));heading.add(history);var remove=new JButton("Remove saved batch");heading.add(remove);
        history.addActionListener(e->{if(!loadingHistory&&worker==null&&history.getSelectedItem() instanceof MotorBatchReport r){report=r;showReport(r);status.setText("Viewing saved results from "+r.created()+". Run batch uses the current controls and baseline.");}});
        remove.addActionListener(e->{if(worker==null&&history.getSelectedItem() instanceof MotorBatchReport r){document.removeMotorBatch(r.id());refreshHistory();report=null;refreshPreview();}});
        add(heading,BorderLayout.NORTH);

        var launch=section("Rocket and workers");
        launch.add(new JLabel("Motor mount"),"span,wrap");launch.add(mount,"span,growx,wmin 0,wrap");
        launch.add(new JLabel("Nominal diameter (mm)"));launch.add(diameter,"span 2,growx,wmin 0,wrap");
        launch.add(label("Length warning above (mm)"));launch.add(length,"span 2,growx,wmin 0,wrap");
        length.setToolTipText("Motors longer than this value still run. Defaults to mount length + overhang.");
        launch.add(ignoreIntersections,"span,wrap");
        ignoreIntersections.setToolTipText("Run motors despite modeled component intersections. Ignored intersections remain in results and CSV. The motor diameter must still fit the mount.");
        ignoreIntersections.addActionListener(e->refreshPreview());
        launch.add(new JLabel("Worker threads"));launch.add(threads,"span 2,growx,wmin 0,wrap");
        launch.add(new JLabel("Ejection delay"),"span,wrap");launch.add(delay,"span,growx,wmin 0,wrap");
        var hint=new JTextArea("Longer motors still run; length warnings remain in results. The intersection override allows overlapping motors to run and records the overlaps. Motor diameter must fit the mount.");
        hint.setLineWrap(true);hint.setWrapStyleWord(true);hint.setEditable(false);hint.setOpaque(false);hint.setRows(4);launch.add(hint,"span,growx,wmin 0,h 72!");settingsPanel.add(launch);

        var filters=section("Motor filters");
        filters.add(new JLabel("Manufacturers"),"top");filters.add(new JScrollPane(manufacturers),"span 2,growx,wmin 0,wrap");
        JButton all=new JButton("All"),none=new JButton("None");filters.add(all,"skip 1");filters.add(none,"wrap");
        all.addActionListener(e->manufacturers.setSelectionInterval(0,Math.max(0,manufacturers.getModel().getSize()-1)));none.addActionListener(e->manufacturers.clearSelection());
        filters.add(new JLabel("Search"));filters.add(search,"span 2,growx,wmin 0,wrap");
        filters.add(new JLabel("Type"));filters.add(motorType,"span 2,growx,wmin 0,wrap");filters.add(unavailable,"span,wrap");
        filters.add(new JLabel("NAR/Tripoli motor level"));filters.add(certification,"span 2,growx,wmin 0,wrap");
        certification.setToolTipText("L1: H–I; L2: J–L; L3: M–O. Sub-H high-power exceptions use available metadata. This does not verify current motor approval or full-rocket requirements.");
        certification.setRenderer(new DefaultListCellRenderer(){@Override public Component getListCellRendererComponent(JList<?> list,Object value,int index,boolean selected,boolean focus){return super.getListCellRendererComponent(list,levelLabel(String.valueOf(value)),index,selected,focus);}});
        filters.add(new JLabel("Range"));filters.add(new JLabel("Minimum"));filters.add(new JLabel("Maximum"),"wrap");
        addRange(filters,"Total impulse (N·s)",impulse);addRange(filters,"Average thrust (N)",thrust);addRange(filters,"Burn time (s)",burn);
        var classes=new JComboBox<String>(new String[]{"Any impulse class","1/8A","1/4A","1/2A","A","B","C","D","E","F","G","H","I","J","K","L","M","N","O"});
        filters.add(new JLabel("Impulse class preset"));filters.add(classes,"span 2,growx,wmin 0,wrap");
        classes.addActionListener(e->{int i=classes.getSelectedIndex();if(i==0){impulse[0].setText("");impulse[1].setText("");}else{double max=1.25*Math.pow(2,i-3);impulse[0].setText(Double.toString(Math.nextUp(max/2)));impulse[1].setText(Double.toString(max));}});
        settingsPanel.add(filters);
        var limits=section("Flight criteria — blank means disabled");
        limits.add(new JLabel("Metric"));limits.add(new JLabel("Minimum"));limits.add(new JLabel("Maximum"),"wrap");
        for(var metric:List.of(MotorBatchMetrics.Metric.RAIL_SPEED,MotorBatchMetrics.Metric.MIN_STABILITY,MotorBatchMetrics.Metric.APOGEE,
                MotorBatchMetrics.Metric.MAX_SPEED,MotorBatchMetrics.Metric.MAX_MACH,MotorBatchMetrics.Metric.MAX_ACCELERATION,
                MotorBatchMetrics.Metric.DEPLOY_SPEED,MotorBatchMetrics.Metric.LANDING_VERTICAL,MotorBatchMetrics.Metric.THRUST_WEIGHT)) {
            var r=range();criteria.put(metric,r);addRange(limits,metric.label,r);
        }
        criteria.get(MotorBatchMetrics.Metric.RAIL_SPEED)[0].setText("15");criteria.get(MotorBatchMetrics.Metric.MIN_STABILITY)[0].setText("1");
        settingsPanel.add(limits);var preview=new JButton("Update candidates");preview.addActionListener(e->refreshPreview());settingsPanel.add(preview);

        table.setAutoCreateRowSorter(true);table.setAutoResizeMode(JTable.AUTO_RESIZE_OFF);table.setSelectionMode(ListSelectionModel.MULTIPLE_INTERVAL_SELECTION);table.moveColumn(13,2);table.moveColumn(14,3);
        table.setDefaultRenderer(Double.class,new DefaultTableCellRenderer(){@Override protected void setValue(Object value){setHorizontalAlignment(SwingConstants.RIGHT);super.setValue(value instanceof Double d&&d==Double.POSITIVE_INFINITY?"Plugged":value);}});
        for(int i=0;i<table.getColumnCount();i++)table.getColumnModel().getColumn(i).setPreferredWidth(i<2?145:155);
        table.getSelectionModel().addListSelectionListener(e->{showDetails();updateButtons();});
        view.addActionListener(e->filterView());
        var results=new JPanel(new BorderLayout(5,5));var toolbar=new JPanel(new FlowLayout(FlowLayout.LEFT));toolbar.add(new JLabel("Show:"));toolbar.add(view);toolbar.add(count);results.add(toolbar,BorderLayout.NORTH);
        details.setEditable(false);details.setLineWrap(true);details.setWrapStyleWord(true);
        var right=new JSplitPane(JSplitPane.VERTICAL_SPLIT,new JScrollPane(table),new JScrollPane(details));right.setResizeWeight(.8);results.add(right);
        var settingsScroll=new JScrollPane(settingsPanel,JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED,JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
        settingsScroll.setMinimumSize(new Dimension(425,0));settingsScroll.getVerticalScrollBar().setUnitIncrement(20);
        var split=new JSplitPane(JSplitPane.HORIZONTAL_SPLIT,settingsScroll,results);split.setResizeWeight(.34);split.setDividerLocation(440);add(split);
        var bottom=new JPanel(new MigLayout("fillx","[grow][][][][]"));bottom.add(status,"span,growx,wrap");bottom.add(progress,"span,growx,wrap");
        bottom.add(run);bottom.add(cancel);bottom.add(export);bottom.add(shortlist);var close=new JButton("Close");bottom.add(close);add(bottom,BorderLayout.SOUTH);
        run.addActionListener(e->start());cancel.addActionListener(e->{if(runner!=null){runner.cancel();status.setText("Cancelling; waiting for active flights to close their output files…");}});
        export.addActionListener(e->exportCSV());shortlist.addActionListener(e->createSimulations());close.addActionListener(e->close());updateButtons();
    }
    private static final class SettingsPanel extends JPanel implements Scrollable {
        SettingsPanel(){super(new MigLayout("fillx,wrap 1","[0:pref,grow,fill]"));}
        @Override public Dimension getPreferredScrollableViewportSize(){return new Dimension(440,600);}
        @Override public int getScrollableUnitIncrement(Rectangle visible,int orientation,int direction){return 20;}
        @Override public int getScrollableBlockIncrement(Rectangle visible,int orientation,int direction){return Math.max(20,visible.height-20);}
        @Override public boolean getScrollableTracksViewportWidth(){return true;}
        @Override public boolean getScrollableTracksViewportHeight(){return false;}
    }
    private static JPanel section(String title){var p=new JPanel(new MigLayout("fillx","[0:pref,grow,fill][72!,fill][72!,fill]"));p.setBorder(BorderFactory.createTitledBorder(title));return p;}
    private static JLabel label(String text){return new JLabel("<html><body style='width:120px'>"+text+"</body></html>");}
    private static void addRange(JPanel p,String text,JTextField[] r){p.add(label(text));p.add(r[0]);p.add(r[1],"wrap");}
    private static String levelLabel(String level){return switch(level){case "ALL"->"All levels";case "BELOW_L1"->"Below L1*";case "REVIEW"->"Needs review";default->level;};}
    private void mountChanged(){if(mount.getSelectedItem() instanceof MountChoice m){diameter.setText(Double.toString(m.diameter*1000));length.setText(Double.toString(m.length*1000));}}
    private static Double optional(JTextField field){return field.getText().isBlank()?null:Double.valueOf(field.getText().strip());}
    private static double value(JTextField field,double fallback){Double v=optional(field);return v==null?fallback:v;}
    private MotorBatchSettings settings(){
        if(!(mount.getSelectedItem() instanceof MountChoice m))throw new IllegalArgumentException("No active motor mount");
        try{threads.commitEdit();}catch(java.text.ParseException e){throw new IllegalArgumentException("Invalid thread count");}
        var bounds=new EnumMap<MotorBatchMetrics.Metric,MotorBatchSettings.Bounds>(MotorBatchMetrics.Metric.class);
        criteria.forEach((key,r)->{Double min=optional(r[0]),max=optional(r[1]);if(min!=null||max!=null)bounds.put(key,new MotorBatchSettings.Bounds(min,max));});
        return new MotorBatchSettings(m.id,((Number)threads.getValue()).intValue(),value(diameter,0)/1000,value(length,0)/1000,
                new HashSet<>(manufacturers.getSelectedValuesList()),search.getText(),(String)motorType.getSelectedItem(),(String)certification.getSelectedItem(),unavailable.isSelected(),
                value(impulse[0],0),value(impulse[1],1e9),value(thrust[0],0),value(thrust[1],1e9),value(burn[0],0),value(burn[1],1e6),
                MotorBatchSettings.DelayPolicy.values()[delay.getSelectedIndex()],bounds,ignoreIntersections.isSelected());
    }
    private void refreshPreview(){
        if(worker!=null)return;
        try {
            var settings=settings();var candidates=MotorCandidateSelector.select(catalogue,settings,source);
            // Prepare only when Run is requested; preview must not compile or execute FC code.
            model.rows=candidates.stream().map(c->new Row(c,List.of(),0,MotorBatchReport.empty(c.exclusion().isEmpty()?Status.NOT_RUN:Status.SCREENED_OUT,
                    c.exclusion().isEmpty()?"READY":c.exclusion()))).toList();model.fireTableDataChanged();report=null;
            count.setText(candidates.size()+" candidates; "+candidates.stream().filter(c->!c.exclusion().isEmpty()).count()+" fit exclusions");
            status.setText(settings.ignoreComponentIntersections()?"Component intersections overridden; ignored intersections remain in results.":"Single flight per motor/delay; ensemble variation is not applied.");
        }catch(RuntimeException e){status.setText(e.getMessage());model.rows=List.of();model.fireTableDataChanged();}
        updateButtons();
    }
    private void start(){
        try {
            var settings=settings();var snapshot=MotorBatchSnapshot.capture(source,settings);
            var candidates=MotorCandidateSelector.select(catalogue,settings,snapshot.baseline());
            if(candidates.isEmpty())throw new IllegalArgumentException("No motors match the selected filters");
            runner=new MotorBatchRunner(snapshot,settings,candidates);report=runner.report();showReport(report);
            worker=new SwingWorker<>() {
                @Override protected MotorBatchReport doInBackground() throws Exception {return runner.run(this::publish);}
                @Override protected void process(List<MotorBatchRunner.Progress> values){var p=values.get(values.size()-1);report=p.report();showReport(report);progress.setMaximum(Math.max(1,p.totalTrials()));progress.setValue(p.completedTrials());status.setText(p.completedTrials()+" / "+p.totalTrials()+" trials; "+report.settings().threads()+" worker threads");}
                @Override protected void done(){
                    try{report=get();status.setText((runner.isCancelled()?"Batch cancelled; partial results saved — ":"Batch finished — ")+report.rows().size()+" motors; peak "+report.peakThreads()+" simultaneous flights.");}
                    catch(Exception e){report=runner.report();status.setText("Batch stopped: "+(e.getCause()==null?e.getMessage():e.getCause().getMessage()));}
                    finally{document.putMotorBatch(report);worker=null;showReport(report);refreshHistory();enableTree(settingsPanel,true);history.setEnabled(true);updateButtons();if(closeAfterCancel)dispose();}
                }
            };
            enableTree(settingsPanel,false);history.setEnabled(false);updateButtons();worker.execute();
        }catch(Exception e){error(e);}
    }
    private void showReport(MotorBatchReport r){model.rows=r.rows();model.fireTableDataChanged();count.setText(r.rows().size()+" motors; "+r.rows().stream().filter(x->x.selected().assessment()==Assessment.MEETS_CRITERIA).count()+" meet criteria");updateButtons();}
    private void refreshHistory(){loadingHistory=true;history.removeAllItems();for(var r:document.getMotorBatches())history.addItem(r);if(report!=null)history.setSelectedItem(report);else history.setSelectedIndex(-1);loadingHistory=false;}
    @SuppressWarnings("unchecked") private void filterView(){((TableRowSorter<Model>)table.getRowSorter()).setRowFilter(new RowFilter<>(){@Override public boolean include(Entry<? extends Model,? extends Integer> entry){var a=model.rows.get(entry.getIdentifier()).selected().assessment();return view.getSelectedIndex()==0 || view.getSelectedIndex()==1&&a==Assessment.MEETS_CRITERIA || view.getSelectedIndex()==2&&a==Assessment.DOES_NOT_MEET_CRITERIA || view.getSelectedIndex()==3&&a==Assessment.NEEDS_REVIEW;}});}
    private List<Row> selected(){return Arrays.stream(table.getSelectedRows()).map(table::convertRowIndexToModel).mapToObj(i->model.rows.get(i)).toList();}
    private void showDetails(){var rows=selected();if(rows.size()!=1){details.setText("");return;}var r=rows.get(0);var m=r.candidate().motor();var classification=MotorCertification.classify(m);var text=new StringBuilder(m.getManufacturer()+" "+m.getDesignation()+"\nCurve: "+m.getDigest()+"\nMotor level: "+levelLabel(classification.level().name())+" ("+classification.impulseClass()+") — "+classification.basis()+"\n"+classification.note()+"\n"+String.join("; ",r.selected().reasons()));if(!r.candidate().fitWarnings().isEmpty())text.append("\nFit warnings (simulation allowed): ").append(String.join("; ",r.candidate().fitWarnings()));if(!r.candidate().ignoredIntersections().isEmpty())text.append("\nIntersection override — ignored: ").append(String.join("; ",r.candidate().ignoredIntersections()));for(var t:r.trials())text.append("\nDelay ").append(t.delay()).append(": ").append(t.status()).append(" ").append(t.assessment()).append(" ").append(t.reasons()).append("\nWarnings: ").append(t.warnings()).append("\n").append(t.error()).append("\nTelemetry: ").append(t.telemetryPath()).append("\nLog: ").append(t.logPath());details.setText(text.toString());details.setCaretPosition(0);}
    private void updateButtons(){run.setEnabled(worker==null&&mount.getItemCount()>0);cancel.setEnabled(worker!=null);export.setEnabled(report!=null);shortlist.setEnabled(worker==null&&report!=null&&!selected().isEmpty());}
    private static void enableTree(Container c,boolean enabled){for(Component child:c.getComponents()){child.setEnabled(enabled);if(child instanceof Container container)enableTree(container,enabled);}}
    private void close(){if(worker!=null){closeAfterCancel=true;runner.cancel();status.setText("Cancelling before closing…");}else dispose();}
    private void error(Exception e){JOptionPane.showMessageDialog(this,e.getMessage(),"Motor batch",JOptionPane.ERROR_MESSAGE);}
    private void exportCSV(){
        String[] labels={"All candidates","Meets criteria only","Selected rows","All delay trials"};
        String choice=(String)JOptionPane.showInputDialog(this,"Export scope","Export CSV",JOptionPane.PLAIN_MESSAGE,null,labels,labels[0]);if(choice==null)return;
        int index=Arrays.asList(labels).indexOf(choice);Set<String> selectedIds=new HashSet<>();selected().forEach(r->selectedIds.add(r.candidate().id()));
        if(index==2&&selectedIds.isEmpty()){error(new IllegalArgumentException("Select one or more result rows first"));return;}
        var chooser=new JFileChooser();chooser.setSelectedFile(new java.io.File("motor-batch-"+report.id().substring(0,8)+".csv"));
        chooser.setFileFilter(new javax.swing.filechooser.FileNameExtensionFilter("CSV files","csv"));if(chooser.showSaveDialog(this)!=JFileChooser.APPROVE_OPTION)return;
        Path path=chooser.getSelectedFile().toPath();if(!path.toString().toLowerCase(Locale.ROOT).endsWith(".csv"))path=Path.of(path+".csv");
        if(java.nio.file.Files.exists(path)&&JOptionPane.showConfirmDialog(this,"Replace "+path.getFileName()+"?","Export CSV",JOptionPane.YES_NO_OPTION)!=JOptionPane.YES_OPTION)return;
        try{MotorBatchCSV.write(path,report,MotorBatchCSV.Scope.values()[index],selectedIds);status.setText("Saved "+path.toAbsolutePath());}catch(Exception e){error(e);}
    }
    private void createSimulations(){
        try {
            var target=MotorBatchSnapshot.createSimulations(report,selected());
            dispose();var frame=new BasicFrame(target);frame.setVisible(true);
            new SimulationRunDialog(frame,target,target.getSimulations().toArray(Simulation[]::new)).setVisible(true);
        }catch(Exception e){error(e);}
    }
    private static final class Model extends AbstractTableModel {
        List<Row> rows=List.of();
        private final String[] names={"Manufacturer","Motor","Length (mm)","Delay (s)","Status","Assessment","Apogee (m)","Rail exit (m/s)","Max speed (m/s)","Max Mach","Max accel (m/s²)","Min stability (cal)","Deploy speed (m/s)","NAR/Tripoli level","Impulse class","Reasons","Ignored intersections","Fit warnings"};
        @Override public int getRowCount(){return rows.size();}@Override public int getColumnCount(){return names.length;}@Override public String getColumnName(int c){return names[c];}
        @Override public Class<?> getColumnClass(int c){return c==2||c==3||c>=6&&c<=12?Double.class:String.class;}
        @Override public Object getValueAt(int r,int c){var row=rows.get(r);var t=row.selected();var m=row.candidate().motor();return switch(c){
            case 0->m.getManufacturer().getDisplayName();case 1->m.getDesignation();case 2->m.getLength()*1000;
            case 3->Double.isNaN(t.delay())?null:t.delay();case 4->t.status().name();case 5->t.assessment().name()+(row.candidate().ignoredIntersections().isEmpty()?"":" (intersection override)")+(row.candidate().fitWarnings().isEmpty()?"":" (fit warning)");
            case 16->String.join("; ",row.candidate().ignoredIntersections());
            case 17->String.join("; ",row.candidate().fitWarnings());
            case 13->levelLabel(MotorCertification.classify(m).level().name());case 14->MotorCertification.classify(m).impulseClass();case 15->String.join("; ",t.reasons());default->{var metrics=new MotorBatchMetrics.Metric[]{MotorBatchMetrics.Metric.APOGEE,MotorBatchMetrics.Metric.RAIL_SPEED,MotorBatchMetrics.Metric.MAX_SPEED,MotorBatchMetrics.Metric.MAX_MACH,MotorBatchMetrics.Metric.MAX_ACCELERATION,MotorBatchMetrics.Metric.MIN_STABILITY,MotorBatchMetrics.Metric.DEPLOY_SPEED};Double v=t.metrics().get(metrics[c-6]);yield v!=null&&Double.isFinite(v)?v:null;}};}
    }
}
