package info.openrocket.swing.gui.figure3d.geometry;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.util.Coordinate;
import info.openrocket.swing.util.BaseTestCase;
import java.util.ArrayList;
import org.junit.jupiter.api.Test;

class FinRendererTest extends BaseTestCase {
    @Test
    void renderedTabCornersMatchIndependentHingeRotationBeforeFinCant() {
        var vertices = new ArrayList<Coordinate>();
        var gl = mock(com.jogamp.opengl.TraceGL2.class, withSettings().stubOnly().defaultAnswer(call -> {
            if (call.getMethod().getName().equals("glVertex3d"))
                vertices.add(new Coordinate(call.getArgument(0), call.getArgument(1), call.getArgument(2)));
            return RETURNS_DEFAULTS.answer(call);
        }));
        var rocket = new Rocket();
        var stage = new AxialStage(); rocket.addChild(stage);
        var body = new BodyTube(1, .06, .002); stage.addChild(body);
        var fin = new TabControlledTrapezoidFinSet(4, .2032, .0508, .1124, .1016,
                .0381, .032512, .04318, 0);
        body.addChild(fin);
        fin.setCrossSection(FinSet.CrossSection.SQUARE);
        fin.setThickness(.009398);
        rocket.enableEvents();
        var renderer = new FinRenderer();
        for (double cant : new double[]{-12, 0, 12}) {
            fin.setCantAngle(Math.toRadians(cant));
            var outline = fin.getFinPoints();
            var root = outline[outline.length - 1];
            var axis = outline[outline.length - 2].sub(root);
            axis = axis.multiply(1 / axis.length());
            var trailing = root.add(axis.multiply(fin.getTabOffset()));
            var hinge = trailing.add(-axis.y * fin.getTabChord(), axis.x * fin.getTabChord(), 0);
            for (double degrees : new double[]{-90, -40, 0, 40, 90}) {
                fin.setTabAngle(Math.toRadians(degrees));
                vertices.clear();
                for (var surface : new Geometry.Surface[]{Geometry.Surface.INSIDE,
                        Geometry.Surface.OUTSIDE, Geometry.Surface.EDGES})
                    renderer.renderFinSet(gl, fin, surface);
                for (double span : new double[]{0, fin.getTabSpan()})
                    for (double chord : new double[]{0, fin.getTabChord()})
                        for (double thickness : new double[]{-.5, .5}) {
                            var v = axis.multiply(span).add(axis.y * chord, -axis.x * chord,
                                    thickness * fin.getThickness());
                            double angle = -fin.getTabAngle();
                            var expected = hinge.add(v.multiply(Math.cos(angle)))
                                    .add(axis.cross(v).multiply(Math.sin(angle)))
                                    .add(axis.multiply(axis.dot(v) * (1 - Math.cos(angle))))
                                    .add(0, fin.getBodyRadius(), 0);
                            assertTrue(vertices.stream().anyMatch(p -> p.sub(expected).length() < 1e-10),
                                    "Missing tab corner for cant=" + cant + ", tab=" + degrees + ": " + expected);
                        }
            }
        }
    }

    @Test
    void allProfilesEmitFiniteSurfacesAndRotateControlTabsAboutTheirHinge() {
        var vertices = new ArrayList<Coordinate>();
        var normals = new ArrayList<Coordinate>();
        // Mock the concrete wrapper: GL2 has too many methods for a generated proxy class.
        var gl = mock(com.jogamp.opengl.TraceGL2.class, withSettings().stubOnly().defaultAnswer(call -> {
            if (call.getMethod().getName().equals("glVertex3d"))
                vertices.add(new Coordinate(call.getArgument(0), call.getArgument(1), call.getArgument(2)));
            if (call.getMethod().getName().equals("glNormal3d"))
                normals.add(new Coordinate(call.getArgument(0), call.getArgument(1), call.getArgument(2)));
            return RETURNS_DEFAULTS.answer(call);
        }));
        var fin = new TabControlledTrapezoidFinSet(4, .12, .12, 0, .08, .03, .02, .015, 0);
        fin.setThickness(.006);
        var renderer = new FinRenderer();
        for (var section : FinSet.CrossSection.values()) {
            fin.setCrossSection(section);
            for (double angle : new double[]{0, -Math.PI/2, Math.PI/2}) {
                fin.setTabAngle(angle);
                vertices.clear(); normals.clear();
                for (var surface : new Geometry.Surface[]{Geometry.Surface.INSIDE,
                        Geometry.Surface.OUTSIDE, Geometry.Surface.EDGES})
                    renderer.renderFinSet(gl, fin, surface);
                assertFalse(vertices.isEmpty());
                assertFalse(normals.isEmpty());
                for (var p : vertices)
                    assertTrue(Double.isFinite(p.x) && Double.isFinite(p.y) && Double.isFinite(p.z));
                for (var n : normals) assertEquals(1, n.length(), 1e-9);
                double maxZ = vertices.stream().mapToDouble(p -> Math.abs(p.z)).max().orElseThrow();
                if (angle == 0) assertEquals(.003, maxZ, 1e-9);
                else assertTrue(maxZ >= .0199, section + " tab is not deflected in the mesh");
            }
        }
        // Reuse the same renderer for ordinary fins as well: tessellator lifetime is per call.
        var ordinary = new TrapezoidFinSet(3, .12, .08, .02, .08);
        for (int i = 0; i < 3; i++) renderer.renderFinSet(gl, ordinary, Geometry.Surface.INSIDE);
    }
}
