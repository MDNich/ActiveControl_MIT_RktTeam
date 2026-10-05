package info.openrocket.swing.gui.plot;

import info.openrocket.core.document.*;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.ensemble.*;
import info.openrocket.core.util.TestRockets;
import info.openrocket.swing.util.EnsembleSwingTestCase;
import org.junit.jupiter.api.Test;
import org.jfree.data.xy.IntervalXYDataset;
import java.nio.file.*;
import java.util.List;
import static info.openrocket.core.simulation.FlightDataType.*;
import static org.junit.jupiter.api.Assertions.*;

class EnsemblePlotTest extends EnsembleSwingTestCase {
    private Simulation simulation() {
        var settings=new EnsembleSettings(true,5,2,.05,2,.01,1,1);
        var accumulator=new EnsembleAccumulator(settings);
        for (int run=0;run<5;run++) {
            var b=new FlightDataBranch("Main",TYPE_TIME);
            for (int i=0;i<=100;i++) {
                double t=i/10.;
                b.addPoint();b.setValue(TYPE_TIME,t);b.setValue(TYPE_ALTITUDE,(20+run)*t*(10-t));
                b.setValue(TYPE_AIR_TEMPERATURE,273.15+run+t);
                b.setValue(TYPE_POSITION_X,run*t);b.setValue(TYPE_POSITION_Y,t);
                b.setValue(TYPE_ORIENTATION_QW,run%2==0?1:-1);b.setValue(TYPE_ORIENTATION_QX,0);b.setValue(TYPE_ORIENTATION_QY,0);b.setValue(TYPE_ORIENTATION_QZ,0);
                b.setValue(TYPE_VELOCITY_TOTAL,20);b.setValue(TYPE_ACCELERATION_TOTAL,4);
            }
            accumulator.add(new FlightData(b));
        }
        var rocket=TestRockets.makeEstesAlphaIII();var options=new SimulationOptions();options.setEnsembleSettings(settings);
        return new Simulation(null,rocket,Simulation.Status.LOADED,"Ensemble verification",options,List.of(),accumulator.finish());
    }
    @Test void bandsUseBothAxesCorrectUnitsAndRemainVisibleWithinBounds() throws Exception {
        var simulation=simulation();var config=new SimulationPlotConfiguration("Mean and uncertainty",TYPE_TIME);
        config.addPlotDataType(TYPE_ALTITUDE,0);config.addPlotDataType(TYPE_AIR_TEMPERATURE,1);
        config.setPlotDataUnit(0,TYPE_ALTITUDE.getUnitGroup().getUnit("ft"));
        config.setPlotDataUnit(1,TYPE_AIR_TEMPERATURE.getUnitGroup().getUnit("°C"));
        var plot=SimulationPlot.create(simulation,config,false);var xy=plot.chart.getXYPlot();
        var left=(IntervalXYDataset)xy.getDataset(2);var right=(IntervalXYDataset)xy.getDataset(3);
        double factor=TYPE_ALTITUDE.getUnitGroup().getUnit("ft").toUnit(1);
        assertEquals((550-25*Math.sqrt(2.5))*factor,left.getStartYValue(0,50),1e-9);
        assertEquals(7-Math.sqrt(2.5),right.getStartYValue(0,50),1e-9);
        assertTrue(xy.getRangeAxis(0).getUpperBound()>=left.getEndYValue(0,50));
        plot.setShowBranch(0);assertEquals(Boolean.TRUE,xy.getRenderer(2).getSeriesVisible(0));
        Path path=Path.of("build/ensemble-verification");Files.createDirectories(path);
        org.jfree.chart.ChartUtils.saveChartAsPNG(path.resolve("ensemble-bands.png").toFile(),plot.chart,1200,720);
    }
    @Test void nonmonotoneDomainsRetainTimeOrderAnd3dReadsMeanOnly() {
        var s=simulation();var c=new SimulationPlotConfiguration("Trajectory",TYPE_ALTITUDE);c.addPlotDataType(TYPE_POSITION_X,0);
        var plot=SimulationPlot.create(s,c,false);var d=plot.chart.getXYPlot().getDataset(2);
        assertTrue(d.getXValue(0,50)>d.getXValue(0,100));assertEquals(101,d.getItemCount(0));
        var trajectory=new TrajectoryData(s.getSimulatedData().getBranch(0));
        assertEquals(10,trajectory.at(5).position().x,1e-12);assertEquals(550,trajectory.at(5).position().z,1e-12);
        assertEquals(1,trajectory.at(5).attitude().getW(),1e-12);
    }
    @Test void pdfHasUnitAreaAndIdenticalOrMissingResultsDoNotInventSpread() throws Exception {
        var chart=EnsembleDistributionDialog.createChart("Apogee","m",new double[]{80,100,110,120,150},5);
        var d=(IntervalXYDataset)chart.getXYPlot().getDataset();double area=0;
        for(int i=0;i<d.getItemCount(0);i++) area+=(d.getEndXValue(0,i)-d.getStartXValue(0,i))*d.getYValue(0,i);
        assertEquals(1,area,1e-12);
        assertNull(EnsembleDistributionDialog.createChart("Constant","m",new double[]{10,10},4).getXYPlot().getDataset());
        assertNull(EnsembleDistributionDialog.createChart("Undefined","m",new double[]{},4).getXYPlot().getDataset());
        Path path=Path.of("build/ensemble-verification");Files.createDirectories(path);
        org.jfree.chart.ChartUtils.saveChartAsPNG(path.resolve("ensemble-pdf.png").toFile(),chart,1000,650);
    }
}
