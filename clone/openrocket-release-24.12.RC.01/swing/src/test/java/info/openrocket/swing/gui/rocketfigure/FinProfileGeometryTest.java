package info.openrocket.swing.gui.rocketfigure;

import static org.junit.jupiter.api.Assertions.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.util.*;
import info.openrocket.swing.util.BaseTestCase;
import org.junit.jupiter.api.Test;
import java.awt.*;
import java.awt.geom.*;
import java.awt.image.BufferedImage;
import java.nio.file.*;

class FinProfileGeometryTest extends BaseTestCase {
    @Test
    void roundedHasCircularNoseAndAirfoilHasRoundedNoseAndTaperedTail() {
        double h = .003, chord = .12;
        assertEquals(Math.sqrt(h*h - (h - .001)*(h - .001)),
                FinProfileGeometry.halfThickness(FinSet.CrossSection.ROUNDED, .001, chord, 2*h, 0), 1e-12);
        for (var section : new FinSet.CrossSection[]{FinSet.CrossSection.ROUNDED, FinSet.CrossSection.AIRFOIL}) {
            assertEquals(0, FinProfileGeometry.halfThickness(section, 0, chord, 2*h, 0));
            assertEquals(0, FinProfileGeometry.halfThickness(section, chord, chord, 2*h, 0));
            assertEquals(h, FinProfileGeometry.halfThickness(section, .3*chord, chord, 2*h, 0), 1e-12);
        }
        assertTrue(FinProfileGeometry.halfThickness(FinSet.CrossSection.AIRFOIL, .9*chord, chord, 2*h, 0) < h / 4);
    }

    @Test
    void loftKeepsPlanformThicknessAndControlTabCutout() {
        var fin = new TabControlledTrapezoidFinSet(4, .12, .12, 0, .08, .03, .02, .015, Math.PI/2);
        fin.setThickness(.006);
        var renderer = new FinSetShapes();
        for (var section : new FinSet.CrossSection[]{FinSet.CrossSection.ROUNDED, FinSet.CrossSection.AIRFOIL}) {
            fin.setCrossSection(section);
            var shapes = renderer.getShapesSide(fin, Transformation.IDENTITY);
            assertFalse(shapes[0].shape.contains(.11, .03));
            assertTrue(shapes[0].shape.contains(.09, .03));
            assertTrue(shapes[0].shape.contains(.11, .06));
            for (var face : new FinProfileGeometry(fin).faces(FinSetShapes.previewFinOutline(fin))) {
                for (var point : face.points()) {
                    assertTrue(Double.isFinite(point.x) && Double.isFinite(point.y) && Double.isFinite(point.z));
                    assertTrue(Math.abs(point.z) <= .003 + 1e-12);
                    assertTrue(point.x >= -1e-12 && point.x <= .12 + 1e-12);
                }
            }
            var edgeOn = renderer.getShapesSide(fin, Transformation.rotate_x(Math.PI/2))[0].shape;
            assertEquals(.006, edgeOn.getBounds2D().getHeight(), 1e-10);
        }
    }

    @Test
    void drawsComparisonAtActualThicknessAndObliqueAngles() throws Exception {
        var image = new BufferedImage(1400, 880, BufferedImage.TYPE_INT_RGB);
        var g = image.createGraphics();
        g.setColor(new Color(5, 39, 55)); g.fillRect(0,0,1400,880);
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 18));
        int row = 0;
        for (var section : FinSet.CrossSection.values()) {
            var fin = new TrapezoidFinSet(3, .14, .08, .04, .07);
            fin.setCrossSection(section); fin.setThickness(.010); fin.setLeadingEdgeDistance(.025);
            int y = 50 + row++ * 215;
            g.setColor(Color.WHITE); g.drawString(section.toString(), 20, y);
            // Equal axis scales in both projections; no artificial thickness scaling.
            for (int col = 0; col < 2; col++) {
                var transform = new AffineTransform();
                transform.translate(210 + col * 575, y + 150);
                transform.scale(3400, -3400);
                var shapes = new FinSetShapes().getShapesSide(fin, Transformation.rotate_x(col == 0 ? Math.PI/2 : .85));
                for (var shape : shapes) {
                    g.setStroke(new BasicStroke(1.4f));
                    g.draw(transform.createTransformedShape(shape.shape));
                }
            }
        }
        g.dispose();
        Path directory = Path.of("build/fin-profile-verification"); Files.createDirectories(directory);
        javax.imageio.ImageIO.write(image, "png", directory.resolve("fin-sections.png").toFile());
    }
}
