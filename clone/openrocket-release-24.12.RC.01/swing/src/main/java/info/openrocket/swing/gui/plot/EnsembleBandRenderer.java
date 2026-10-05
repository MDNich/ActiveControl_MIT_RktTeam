package info.openrocket.swing.gui.plot;

import org.jfree.chart.axis.ValueAxis;
import org.jfree.chart.plot.*;
import org.jfree.chart.renderer.xy.*;
import org.jfree.data.xy.*;
import java.awt.*;
import java.awt.geom.*;

/** Draw adjacent finite quadrilaterals, preserving time order even for nonmonotone X variables. */
final class EnsembleBandRenderer extends AbstractXYItemRenderer {
    @Override public void drawItem(Graphics2D g, XYItemRendererState state, Rectangle2D area,
            PlotRenderingInfo info, XYPlot plot, ValueAxis domain, ValueAxis range, XYDataset dataset,
            int series, int item, CrosshairState crosshair, int pass) {
        if (item==0 || !getItemVisible(series,item)) return;
        var data=(IntervalXYDataset)dataset;
        double[] xs={data.getXValue(series,item-1),data.getXValue(series,item)};
        double[] low={data.getStartYValue(series,item-1),data.getStartYValue(series,item)};
        double[] high={data.getEndYValue(series,item-1),data.getEndYValue(series,item)};
        for (int i=0;i<2;i++) {
            if (!Double.isFinite(xs[i]) || !Double.isFinite(low[i]) || !Double.isFinite(high[i])) return;
            xs[i]=domain.valueToJava2D(xs[i],area,plot.getDomainAxisEdge());
            low[i]=range.valueToJava2D(low[i],area,plot.getRangeAxisEdge(plot.getRangeAxisIndex(range)));
            high[i]=range.valueToJava2D(high[i],area,plot.getRangeAxisEdge(plot.getRangeAxisIndex(range)));
        }
        var path=new Path2D.Double(); boolean vertical=plot.getOrientation()==PlotOrientation.VERTICAL;
        point(path,xs[0],low[0],vertical,true);point(path,xs[1],low[1],vertical,false);
        point(path,xs[1],high[1],vertical,false);point(path,xs[0],high[0],vertical,false);path.closePath();
        Composite composite=g.getComposite();Paint paint=g.getPaint();
        g.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER,.18f));g.setPaint(getItemPaint(series,item));g.fill(path);
        g.setComposite(composite);g.setPaint(paint);
    }
    private static void point(Path2D p,double x,double y,boolean vertical,boolean first) {
        if (first) p.moveTo(vertical?x:y,vertical?y:x);else p.lineTo(vertical?x:y,vertical?y:x);
    }
}
