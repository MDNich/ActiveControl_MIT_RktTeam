package info.openrocket.swing.gui.plot;

import info.openrocket.core.document.Simulation;
import info.openrocket.core.simulation.ensemble.*;
import info.openrocket.core.unit.Unit;
import info.openrocket.swing.gui.util.GUIUtil;
import org.jfree.chart.*;
import org.jfree.chart.plot.*;
import org.jfree.chart.title.TextTitle;
import org.jfree.data.statistics.*;
import javax.swing.*;
import java.awt.*;
import java.util.Arrays;
import java.util.Locale;

/** Empirical PDFs; histograms integrate to one and do not assume Gaussian flight outcomes. */
public final class EnsembleDistributionDialog extends JDialog {
    private final EnsembleResult result;
    private final JComboBox<EnsembleMetric> metric=new JComboBox<>(EnsembleMetric.values());
    private final JComboBox<String> branch;
    private final JSpinner bins=new JSpinner(new SpinnerNumberModel(10,2,100,1));
    private final ChartPanel chart=new ChartPanel(null);
    private final JLabel summary=new JLabel(" ");
    public EnsembleDistributionDialog(Window owner, Simulation simulation) {
        super(owner,"Flight outcome distributions — "+simulation.getName(),ModalityType.DOCUMENT_MODAL);
        result=simulation.getSimulatedData().getEnsembleResult();
        if (result==null) throw new IllegalArgumentException("Run an ensemble first");
        branch=new JComboBox<>(simulation.getSimulatedData().getBranches().stream().map(b->b.getName()).toArray(String[]::new));
        bins.setValue(Math.max(2,Math.min(100,(int)Math.ceil(Math.sqrt(result.settings().runs())))));
        var panel=new JPanel(new BorderLayout(8,8));panel.setBorder(BorderFactory.createEmptyBorder(10,10,10,10));
        var controls=new JPanel(new FlowLayout(FlowLayout.LEADING));
        controls.add(new JLabel("Stage"));controls.add(branch);controls.add(new JLabel("Quantity"));controls.add(metric);
        controls.add(new JLabel("Bins"));controls.add(bins);panel.add(controls,BorderLayout.NORTH);panel.add(chart,BorderLayout.CENTER);
        var bottom=new JPanel(new GridLayout(0,1));bottom.add(summary);
        bottom.add(new JLabel("Empirical probability density: bar area totals 1. Undefined quantities are excluded and counted below the title."));
        panel.add(bottom,BorderLayout.SOUTH);setContentPane(panel);
        metric.addActionListener(e->refresh());branch.addActionListener(e->refresh());bins.addChangeListener(e->refresh());
        GUIUtil.setDisposableDialogOptions(this,null);setDefaultCloseOperation(DISPOSE_ON_CLOSE);
        setSize(1080,700);setLocationRelativeTo(owner);refresh();
    }
    private void refresh() {
        var m=(EnsembleMetric)metric.getSelectedItem();Unit unit=m.units().getDefaultUnit();
        var samples=result.samples(branch.getSelectedIndex(),m);
        double[] finite=Arrays.stream(samples).filter(Double::isFinite).map(unit::toUnit).toArray();
        JFreeChart c=createChart(m.toString(),unit.getUnit(),finite,((Number)bins.getValue()).intValue());
        c.addSubtitle(new TextTitle(result.settings().sourceLabel()+" · "+finite.length+"/"+samples.length+" defined outcomes · seed "+result.settings().seed()));
        chart.setChart(c);
        double mean=Arrays.stream(finite).average().orElse(Double.NaN), sum=0;
        for (double value:finite) sum+=(value-mean)*(value-mean);
        double sd=finite.length>1?Math.sqrt(sum/(finite.length-1)):Double.NaN;
        summary.setText(String.format(Locale.ROOT,"Mean: %.6g %s    Sample σ: %.6g %s    Undefined: %d",mean,unit.getUnit(),sd,unit.getUnit(),samples.length-finite.length));
    }
    static JFreeChart createChart(String title,String unit,double[] values,int bins) {
        String x=title+(unit.isEmpty()?"":" ("+unit+")"), y="Probability density"+(unit.isEmpty()?"":" (1/"+unit+")");
        double min=Arrays.stream(values).min().orElse(Double.NaN), max=Arrays.stream(values).max().orElse(Double.NaN);
        if (values.length==0 || max==min) {
            var c=ChartFactory.createXYLineChart(title,x,y,null);
            c.addSubtitle(new TextTitle(values.length==0?"No defined outcomes for this quantity":"All outcomes equal: point mass at "+min+" "+unit+" (no finite-width PDF)"));
            if (values.length>0) { double pad=Math.max(1e-6,Math.max(1,Math.abs(min))*.05);c.getXYPlot().getDomainAxis().setRange(min-pad,min+pad);c.getXYPlot().addDomainMarker(new ValueMarker(min,Color.BLUE,new BasicStroke(2))); }
            return c;
        }
        var histogram=new HistogramDataset();histogram.setType(HistogramType.SCALE_AREA_TO_1);
        histogram.addSeries("Observed outcomes",values,bins,min,max);
        var c=ChartFactory.createHistogram(title,x,y,histogram,PlotOrientation.VERTICAL,false,true,false);
        c.getXYPlot().setForegroundAlpha(.7f);return c;
    }
}
