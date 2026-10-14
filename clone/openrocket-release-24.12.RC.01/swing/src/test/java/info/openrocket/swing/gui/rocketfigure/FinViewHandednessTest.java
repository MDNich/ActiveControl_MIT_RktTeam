package info.openrocket.swing.gui.rocketfigure;

import static org.junit.jupiter.api.Assertions.*;

import info.openrocket.core.rocketcomponent.FinSet;
import info.openrocket.core.rocketcomponent.TabControlledTrapezoidFinSet;
import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.Transformation;
import info.openrocket.swing.util.BaseTestCase;
import java.awt.Shape;
import java.awt.geom.Line2D;
import java.awt.geom.PathIterator;
import org.junit.jupiter.api.Test;

/** Checks the viewer convention, rather than only agreement between rotation formulas. */
class FinViewHandednessTest extends BaseTestCase {
    private TabControlledTrapezoidFinSet fin() {
        var fin = new TabControlledTrapezoidFinSet(4, .2032, .0508, .1524, .1016,
                .0381, .032512, .04318, Math.toRadians(40));
        fin.setThickness(.009398);
        fin.setCrossSection(FinSet.CrossSection.TRIANGULAR);
        fin.setLeadingEdgeDistance(.018);
        return fin;
    }

    @Test
    void positiveTabDeflectionAtScreenshotAngleIsAboveAndBehindTheTopFin() {
        var fin = fin();
        var view = Transformation.rotate_x(Math.toRadians(158.1 + 180));
        var points = view.transform(fin.getRollCtrlTabPoints());
        assertTrue(points[0].y > points[1].y, "The top tab's trailing edge must project above its hinge");
        assertTrue(points[0].z > points[1].z, "Positive Z is behind the fin, away from the viewer");
        for (int i = 0; i < 4; i++) {
            double phi = Math.toRadians(158.1 + i * 90);
            points = Transformation.rotate_x(phi).transform(fin.getRollCtrlTabPoints());
            var chord = points[0].sub(points[1]);
            assertEquals(fin.getTabChord() * Math.cos(fin.getTabAngle()), chord.x, 1e-10);
            assertEquals(-fin.getTabChord() * Math.sin(fin.getTabAngle()) * Math.sin(phi), chord.y, 1e-10);
            assertEquals(fin.getTabChord() * Math.sin(fin.getTabAngle()) * Math.cos(phi), chord.z, 1e-10);
        }
    }

    @Test
    void bevelAndThicknessUseTheFaceNearestTheNegativeZViewerForEveryFin() {
        var fin = fin();
        fin.setTabAngle(0);
        for (int i = 0; i < 4; i++) {
            double phi = Math.toRadians(158.1 + i * 90);
            var view = Transformation.rotate_x(phi);
            var shapes = new FinSetShapes().getShapesSide(fin, view);
            double side = -Math.signum(Math.cos(phi));
            var crease = new Coordinate(.1524 * .5 + .018, .1016 * .5, side * .009398 / 2);
            var near = view.transform(crease);
            var far = view.transform(crease.setZ(-crease.z));
            assertTrue(near.z < far.z);
            Shape line = shapes[shapes.length - 1].shape;
            assertEquals(0, distance(line, near.x, near.y), 1e-10, "Draw the near-face bevel crease");
            assertTrue(distance(line, far.x, far.y) > 1e-4, "Do not draw the far-face bevel crease");
        }
    }

    @Test
    void topTipThicknessWallFacesTheNegativeZViewerAtScreenshotAngle() {
        var fin = fin();
        fin.setTabAngle(0);
        var view = Transformation.rotate_x(Math.toRadians(158.1 + 180));
        var shapes = new FinSetShapes().getShapesSide(fin, view);
        for (double z : new double[]{-.009398 / 2, .009398 / 2}) {
            var tip = view.transform(new Coordinate(.19, .1016, z));
            assertEquals(0, distance(shapes[3].shape, tip.x, tip.y), 1e-9,
                    "Both borders of the visible tip-thickness wall must be drawn");
        }
    }

    private static double distance(Shape shape, double x, double y) {
        var iterator = shape.getPathIterator(null, 1e-10);
        double[] p = new double[6];
        double x0 = 0, y0 = 0, firstX = 0, firstY = 0, result = Double.POSITIVE_INFINITY;
        while (!iterator.isDone()) {
            int kind = iterator.currentSegment(p);
            if (kind == PathIterator.SEG_MOVETO) {
                firstX = x0 = p[0]; firstY = y0 = p[1];
            } else {
                double x1 = kind == PathIterator.SEG_CLOSE ? firstX : p[0];
                double y1 = kind == PathIterator.SEG_CLOSE ? firstY : p[1];
                result = Math.min(result, Line2D.ptSegDist(x0, y0, x1, y1, x, y));
                x0 = x1; y0 = y1;
            }
            iterator.next();
        }
        return result;
    }
}
