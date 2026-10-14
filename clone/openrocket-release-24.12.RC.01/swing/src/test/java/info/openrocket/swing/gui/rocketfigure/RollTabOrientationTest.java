package info.openrocket.swing.gui.rocketfigure;

import static org.junit.jupiter.api.Assertions.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.util.*;
import info.openrocket.swing.util.BaseTestCase;
import java.awt.*;
import java.awt.geom.*;
import java.awt.image.BufferedImage;
import java.nio.file.*;
import org.junit.jupiter.api.Test;

class RollTabOrientationTest extends BaseTestCase {
    private double error;
    private int checks;

    @Test
    void independentAxisRotationMatchesAllFinInstancesCantAndOrthographicViews() {
        var rocket = new Rocket();
        var stage = new AxialStage(); rocket.addChild(stage);
        var body = new BodyTube(1, .06, .002); stage.addChild(body);
        var fin = new TabControlledTrapezoidFinSet(4, .2032, .0508, .1524, .1016,
                .0381, .032512, .04318, 0);
        body.addChild(fin); fin.setThickness(.009398); fin.setCrossSection(FinSet.CrossSection.SQUARE);
        rocket.enableEvents();
        for (double sweep : new double[]{.1124, .1524, .1924}) {
            fin.setSweep(sweep);
            for (double cant : new double[]{-12, 0, 12}) {
                fin.setCantAngle(Math.toRadians(cant));
                check(rocket, fin);
            }
        }
        System.out.println("TAB_ORIENTATION synthetic corner checks=" + checks + " max_error_m=" + error);
    }

    @Test
    void verifyRequestedRocketWithoutSavingChanges() throws Exception {
        String file = System.getenv("OR_TAB_CHECK_ROCKET");
        org.junit.jupiter.api.Assumptions.assumeTrue(file != null);
        Path source = Path.of(file);
        byte[] before = Files.readAllBytes(source);
        // Motors are irrelevant to this geometry-only check; preserve the design on disk.
        var services = info.openrocket.core.startup.Application.getInjector();
        info.openrocket.core.startup.Application.setInjector(services.createChildInjector(
                new com.google.inject.AbstractModule() {
                    @Override protected void configure() {
                        bind(info.openrocket.core.database.motor.MotorDatabase.class)
                                .toInstance(new info.openrocket.core.database.motor.ThrustCurveMotorSetDatabase());
                    }
                }));
        var document = new info.openrocket.core.file.GeneralRocketLoader(source.toFile()).load();
        var rocket = document.getRocket();
        var fin = (TabControlledTrapezoidFinSet) rocket.getSelectedConfiguration().getActiveComponents().stream()
                .filter(c -> c instanceof TabControlledTrapezoidFinSet).findFirst().orElseThrow();
        double savedAngle = fin.getTabAngle();
        check(rocket, fin);
        fin.setTabAngle(savedAngle);
        render(fin);
        renderScreenshotAngle(fin);
        var output = Path.of("build/fin-profile-verification/roll-tab-orientation.txt");
        Files.writeString(output, "Source: " + source + "\nFin: " + fin.getName()
                + "\nSaved tab angle (deg): " + Math.toDegrees(savedAngle)
                + "\nFin cant (deg): " + Math.toDegrees(fin.getCantAngle())
                + "\nFin count: " + fin.getFinCount() + "\nCorner checks: " + checks
                + "\nMaximum error against independent Rodrigues rotation (m): " + error
                + "\nTab chord (mm): " + 1000 * fin.getTabChord()
                + "\nChordwise projection at saved angle (mm): " + 1000 * fin.getTabChord() * Math.cos(savedAngle)
                + "\nOut-of-plane displacement (mm): " + 1000 * fin.getTabChord() * Math.sin(savedAngle) + "\n");
        assertArrayEquals(before, Files.readAllBytes(source));
        System.out.println(Files.readString(output));
    }

    private void check(Rocket rocket, TabControlledTrapezoidFinSet fin) {
        var renderer = new FinSetShapes();
        for (double degrees : new double[]{-90, -40, 0, 40, 90}) {
            fin.setTabAngle(Math.toRadians(degrees));
            var neutral = neutral(fin);
            var axis = neutral[2].sub(neutral[1]).multiply(1 / fin.getTabSpan());
            var rotated = fin.getRollCtrlTabPoints();
            for (int i = 0; i < 4; i++) assertCoordinate(about(neutral[i], neutral[1], axis, -fin.getTabAngle()), rotated[i]);
            var u = neutral[0].sub(neutral[1]);
            var v = rotated[0].sub(rotated[1]);
            assertEquals(-fin.getTabAngle(), Math.atan2(axis.dot(u.cross(v)), u.dot(v)), 1e-10);
            for (var instance : rocket.getSelectedConfiguration().getActiveInstances().get(fin)) {
                double azimuth = fin.getAngleOffset() + instance.instanceNumber * 2 * Math.PI / fin.getFinCount();
                for (double roll : new double[]{0, Math.toRadians(158.1), Math.toRadians(293.6)}) {
                    for (boolean back : new boolean[]{false, true}) {
                        var transform = Transformation.rotate_x(roll).applyTransformation(instance.transform);
                        var expected = new Rectangle2D.Double();
                        boolean first = true;
                        for (var p : neutral) for (double thickness : new double[]{-.5, .5}) {
                            var local = about(p.add(0, 0, thickness * fin.getThickness()), neutral[1], axis, -fin.getTabAngle());
                            var world = world(fin, local, azimuth);
                            var reference = rotateX(world, roll);
                            var actual = transform.applyTransformation(fin.getCantRotation())
                                    .applyTransformation(fin.getRollCtrlTabRotation())
                                    .transform(p.add(0, 0, thickness * fin.getThickness()));
                            assertCoordinate(reference, actual);
                            double x = back ? reference.z : reference.x;
                            if (first) { expected.setRect(x, reference.y, 0, 0); first = false; }
                            else expected.add(x, reference.y);
                        }
                        var shapes = back ? renderer.getShapesBack(fin, transform) : renderer.getShapesSide(fin, transform);
                        // No fillets on the synthetic fixture; in side view the roll tab has stable index 2.
                        Shape tab = back ? shapes[shapes.length - 1].shape : shapes[2].shape;
                        // The actual model has rear-view fillets appended after the tab.
                        if (back && fin.getFilletRadius() > 0) tab = shapes[shapes.length - 2].shape;
                        var bounds = tab.getBounds2D();
                        assertEquals(expected.getMinX(), bounds.getMinX(), 1e-9);
                        assertEquals(expected.getMaxX(), bounds.getMaxX(), 1e-9);
                        assertEquals(expected.getMinY(), bounds.getMinY(), 1e-9);
                        assertEquals(expected.getMaxY(), bounds.getMaxY(), 1e-9);
                    }
                }
            }
        }
    }

    private void assertCoordinate(Coordinate expected, Coordinate actual) {
        error = Math.max(error, expected.sub(actual).length()); checks++;
        assertEquals(expected.x, actual.x, 1e-10);
        assertEquals(expected.y, actual.y, 1e-10);
        assertEquals(expected.z, actual.z, 1e-10);
    }

    private static Coordinate[] neutral(TabControlledTrapezoidFinSet fin) {
        // The fin root follows the cylindrical body when canted. Use that planform,
        // but construct the tab and rotate it independently of the production code.
        var planform = fin.getFinPoints();
        var trailingRoot = planform[planform.length - 1];
        var d = planform[planform.length - 2].sub(trailingRoot);
        d = d.multiply(1 / d.length());
        var root = trailingRoot.add(d.multiply(fin.getTabOffset()));
        var tip = root.add(d.multiply(fin.getTabSpan()));
        var inward = new Coordinate(-d.y, d.x, 0).multiply(fin.getTabChord());
        return new Coordinate[]{root, root.add(inward), tip.add(inward), tip};
    }

    /** Independent Rodrigues formula, rather than the production Euler-matrix composition. */
    private static Coordinate about(Coordinate p, Coordinate hinge, Coordinate axis, double angle) {
        var v = p.sub(hinge);
        return hinge.add(v.multiply(Math.cos(angle))).add(axis.cross(v).multiply(Math.sin(angle)))
                .add(axis.multiply(axis.dot(v) * (1 - Math.cos(angle))));
    }
    private static Coordinate rotateX(Coordinate p, double a) {
        return new Coordinate(p.x, p.y * Math.cos(a) - p.z * Math.sin(a), p.y * Math.sin(a) + p.z * Math.cos(a));
    }
    private static Coordinate world(TabControlledTrapezoidFinSet fin, Coordinate p, double azimuth) {
        var origin = fin.getParent().getComponentLocations()[0].add(fin.getPosition());
        double x = p.x - fin.getLength() / 2, cant = fin.getCantAngle();
        var canted = new Coordinate(x * Math.cos(cant) + p.z * Math.sin(cant), p.y + fin.getBodyRadius(),
                -x * Math.sin(cant) + p.z * Math.cos(cant));
        return origin.add(fin.getLength() / 2, 0, 0).add(rotateX(canted, azimuth));
    }

    private static void render(TabControlledTrapezoidFinSet fin) throws Exception {
        var image = new BufferedImage(1400, 850, BufferedImage.TYPE_INT_RGB);
        var g = image.createGraphics();
        g.setColor(new Color(71, 76, 80)); g.fillRect(0, 0, 1400, 850);
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 20));
        g.setColor(Color.WHITE);
        g.drawString(fin.getName() + ": all four tabs at " + String.format(java.util.Locale.ROOT, "%+.0f°", Math.toDegrees(fin.getTabAngle())), 25, 30);
        String[] labels = {"Above body (0°)", "Far side (90°)", "Below body (180°)", "Near side (270°)"};
        var neutral = neutral(fin);
        for (int i = 0; i < 4; i++) {
            double azimuth = i * Math.PI / 2;
            var view = Transformation.rotate_x(azimuth);
            int x = 25 + (i % 2) * 700, y = 75 + (i / 2) * 390;
            g.setColor(Color.WHITE); g.drawString(labels[i], x, y);
            var chord = view.linearTransform(fin.getRollCtrlTabPoints()[0].sub(fin.getRollCtrlTabPoints()[1]));
            double angle = Math.toDegrees(Math.atan2(chord.y, chord.x));
            g.drawString(String.format(java.util.Locale.ROOT, "Projected chord angle: %+.1f°", angle), x, y + 30);
            var screen = new AffineTransform();
            screen.translate(x + 25, y + (i == 0 ? 300 : i == 2 ? 65 : 210));
            screen.scale(2700, -2700);
            g.setColor(new Color(160, 160, 160)); g.setStroke(new BasicStroke(1));
            g.draw(screen.createTransformedShape(new Line2D.Double(0, 0, fin.getRootChord() * 1.05, 0)));
            g.setColor(new Color(140, 175, 255)); g.setStroke(new BasicStroke(2));
            for (var shape : new FinSetShapes().getShapesSide(fin, view)) g.draw(screen.createTransformedShape(shape.shape));
            // Orange is the independently rotated tab centre plane, with no thickness/hidden-edge effects.
            var oracle = new Path2D.Double();
            var axis = neutral[2].sub(neutral[1]).multiply(1 / fin.getTabSpan());
            for (int j = 0; j < 4; j++) {
                var p = rotateX(about(neutral[j], neutral[1], axis, -fin.getTabAngle()), azimuth);
                if (j == 0) oracle.moveTo(p.x, p.y); else oracle.lineTo(p.x, p.y);
            }
            oracle.closePath();
            g.setColor(new Color(255, 210, 100));
            g.setStroke(new BasicStroke(1.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10, new float[]{6, 5}, 0));
            g.draw(screen.createTransformedShape(oracle));
        }
        g.setColor(Color.WHITE); g.drawString("Blue: app preview    ·    Gold dashed: independent centre-plane calculation", 25, 825);
        g.dispose();
        var dir = Path.of("build/fin-profile-verification"); Files.createDirectories(dir);
        javax.imageio.ImageIO.write(image, "png", dir.resolve("roll-tab-orientation.png").toFile());
    }

    private static void renderScreenshotAngle(TabControlledTrapezoidFinSet fin) throws Exception {
        var image = new BufferedImage(1600, 860, BufferedImage.TYPE_INT_RGB);
        var g = image.createGraphics();
        g.setColor(new Color(71, 76, 80)); g.fillRect(0, 0, image.getWidth(), image.getHeight());
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 23)); g.setColor(Color.WHITE);
        g.drawString("Demonstrator MainFins · view 158.1° · tabs +40° · thickness 9.398 mm", 30, 40);
        g.drawString("All four fins", 30, 90);
        g.drawString("Top fin enlarged", 850, 90);
        for (int panel = 0; panel < 2; panel++) {
            var screen = new AffineTransform();
            screen.translate(panel == 0 ? 100 : 880, panel == 0 ? 460 : 590);
            screen.scale(panel == 0 ? 2200 : 3200, panel == 0 ? -2200 : -3200);
            double radius = fin.getBodyRadius();
            g.setColor(new Color(155, 155, 155)); g.setStroke(new BasicStroke(1));
            g.draw(screen.createTransformedShape(new Rectangle2D.Double(-.015, -radius, .24, 2 * radius)));
            for (int i = 0; i < 4; i++) {
                if (panel == 1 && i != 2) continue;
                double phi = Math.toRadians(158.1 + 90 * i) + fin.getAngleOffset();
                var transform = Transformation.rotate_x(phi)
                        .applyTransformation(new Transformation(0, radius, 0));
                g.setColor(new Color(140, 175, 255)); g.setStroke(new BasicStroke(2));
                for (var shape : new FinSetShapes().getShapesSide(fin, transform))
                    g.draw(screen.createTransformedShape(shape.shape));
            }
        }
        g.setColor(Color.WHITE);
        g.drawString("Positive Z points away from the viewer. The top tab's trailing edge is above and behind its hinge.", 30, 820);
        g.dispose();
        javax.imageio.ImageIO.write(image, "png", Path.of("build/fin-profile-verification/roll-tabs-158.1.png").toFile());
    }
}
