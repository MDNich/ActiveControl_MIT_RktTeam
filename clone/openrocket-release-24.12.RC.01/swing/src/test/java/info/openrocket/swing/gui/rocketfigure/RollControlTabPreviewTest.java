package info.openrocket.swing.gui.rocketfigure;

import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.util.Transformation;
import info.openrocket.swing.util.BaseTestCase;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.nio.file.*;
import org.junit.jupiter.api.Test;

class RollControlTabPreviewTest extends BaseTestCase {
    @Test
    void rendersAboveAndBelowTheBodyAtPositiveAndNegativeDeflection() throws Exception {
        var image = new BufferedImage(1500, 1040, BufferedImage.TYPE_INT_RGB);
        var g = image.createGraphics();
        g.setColor(new Color(71, 76, 80)); g.fillRect(0, 0, 1500, 1040);
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 18));
        int col = 0;
        for (double deflection : new double[]{-40, 0, 40}) {
            int row = 0;
            for (double rotation : new double[]{15, 165}) {
                var fin = new TabControlledTrapezoidFinSet(4, .30, .075, .225, .18, .060, .05, .065,
                        Math.toRadians(deflection));
                fin.setThickness(.006);
                fin.setCrossSection(FinSet.CrossSection.TRIANGULAR);
                fin.setLeadingEdgeDistance(.025);
                int left = col * 500, top = row++ * 520;
                g.setColor(Color.WHITE);
                g.drawString("Tab " + (int)deflection + "°, view " + (int)rotation + "°", left + 25, top + 30);
                var screen = new AffineTransform();
                screen.translate(left + 25, top + (rotation < 90 ? 380 : 150));
                screen.scale(1400, -1400);
                g.setColor(new Color(175, 175, 175));
                g.setStroke(new BasicStroke(1));
                g.draw(screen.createTransformedShape(new java.awt.geom.Line2D.Double(0, 0, .32, 0)));
                g.setColor(new Color(140, 175, 255));
                g.setStroke(new BasicStroke(3));
                for (var shape : new FinSetShapes().getShapesSide(fin, Transformation.rotate_x(Math.toRadians(rotation))))
                    g.draw(screen.createTransformedShape(shape.shape));
            }
            col++;
        }
        g.dispose();
        var dir = Path.of("build/fin-profile-verification"); Files.createDirectories(dir);
        javax.imageio.ImageIO.write(image, "png", dir.resolve("roll-tabs.png").toFile());
    }
}
