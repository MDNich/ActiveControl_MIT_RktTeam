package info.openrocket.core.rocketcomponent;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import info.openrocket.core.util.BaseTestCase;
import info.openrocket.core.util.BoundingBox;
import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.Transformation;

class TabControlledTrapezoidFinSetTest extends BaseTestCase {

    private static final double EPSILON = 1e-10;

    private TabControlledTrapezoidFinSet fin() {
        TabControlledTrapezoidFinSet fin = new TabControlledTrapezoidFinSet(
                4, 0.12, 0.12, 0, 0.08, 0.03, 0.02, 0.015, 0);
        fin.setThickness(0.006);
        return fin;
    }

    @ParameterizedTest
    @ValueSource(doubles = {-90, -60, -30, 0, 30, 60, 90})
    void tabRotatesAboutItsUpstreamMidThicknessHinge(double degrees) {
        TabControlledTrapezoidFinSet fin = fin();
        double angle = Math.toRadians(degrees);
        fin.setTabAngle(angle);
        assertEquals(angle, fin.getTabAngle(), EPSILON);
        Coordinate[] points = fin.getRollCtrlTabPoints();
        assertPoint(new Coordinate(0.10, 0.015, 0), points[1]);
        assertPoint(new Coordinate(0.10, 0.045, 0), points[2]);
        assertPoint(new Coordinate(0.10 + 0.02 * Math.cos(angle), 0.015, -0.02 * Math.sin(angle)), points[0]);
        assertPoint(new Coordinate(0.10 + 0.02 * Math.cos(angle), 0.045, -0.02 * Math.sin(angle)), points[3]);
        assertEquals(0.02, points[0].sub(points[1]).length(), EPSILON);
        assertEquals(0.03, points[3].sub(points[0]).length(), EPSILON);

        // Neither face is the pivot: the two hinge faces stay symmetric about Z=0.
        Transformation rotation = fin.getRollCtrlTabRotation();
        Coordinate hinge = fin.getRollCtrlTabNeutralPoints()[1];
        Coordinate upper = rotation.transform(hinge.add(0, 0, 0.003));
        Coordinate lower = rotation.transform(hinge.add(0, 0, -0.003));
        assertPoint(hinge, upper.add(lower).multiply(0.5));
        assertEquals(0.006, upper.sub(lower).length(), EPSILON);
        assertPoint(new Coordinate(0.003 * Math.sin(angle), 0, 0.003 * Math.cos(angle)), upper.sub(hinge));
    }

    @ParameterizedTest
    @ValueSource(doubles = {-0.04, 0, 0.04})
    void hingeFollowsEitherTrailingEdgeSweep(double sweep) {
        TabControlledTrapezoidFinSet fin = fin();
        fin.setSweep(sweep);
        Coordinate[] neutral = fin.getRollCtrlTabNeutralPoints();
        Coordinate axis = neutral[2].sub(neutral[1]).multiply(1 / fin.getTabSpan());
        for (double degrees : new double[]{-90, 45, 90}) {
            fin.setTabAngle(Math.toRadians(degrees));
            Coordinate[] points = fin.getRollCtrlTabPoints();
            assertPoint(neutral[1], points[1]);
            assertPoint(neutral[2], points[2]);
            assertEquals(0, points[0].sub(points[1]).dot(axis), EPSILON);
            assertEquals(fin.getTabChord(), points[0].sub(points[1]).length(), EPSILON);
            assertEquals(-fin.getTabChord() * Math.sin(Math.toRadians(degrees)), points[0].z, EPSILON);
        }
    }

    @Test
    void allAngleEntryPointsRespectTheNinetyDegreeLimit() {
        TabControlledTrapezoidFinSet fin = new TabControlledTrapezoidFinSet(
                4, 0.12, 0.12, 0, 0.08, 0.03, 0.02, 0.015, Math.PI);
        assertEquals(Math.PI / 2, fin.getTabAngle(), EPSILON);
        fin.setTabShape(0.03, 0.02, 0.015, -Math.PI);
        assertEquals(-Math.PI / 2, fin.getTabAngle(), EPSILON);
        TabControlledTrapezoidFinSet linked = fin();
        fin.addConfigListener(linked);
        fin.setTabAngle(Math.PI);
        assertEquals(Math.PI / 2, fin.getTabAngle(), EPSILON);
        assertEquals(Math.PI / 2, linked.getTabAngle(), EPSILON);
        fin.setTabAngle(Double.NaN);
        assertEquals(0, fin.getTabAngle(), EPSILON);
    }

    @Test
    void boundingBoxIncludesDeflectedTabAndItsRotatedThickness() {
        TabControlledTrapezoidFinSet fin = fin();
        for (double angle : new double[]{-Math.PI / 2, 0, Math.PI / 2}) {
            fin.setTabAngle(angle);
            BoundingBox bounds = fin.getInstanceBoundingBox();
            Coordinate halfThickness = fin.getRollCtrlTabRotation().linearTransform(new Coordinate(0, 0, 0.003));
            for (Coordinate point : fin.getRollCtrlTabPoints()) {
                for (double side : new double[]{-1, 1}) {
                    Coordinate corner = point.add(halfThickness.multiply(side));
                    assertTrue(corner.x >= bounds.min.x - EPSILON && corner.x <= bounds.max.x + EPSILON);
                    assertTrue(corner.y >= bounds.min.y - EPSILON && corner.y <= bounds.max.y + EPSILON);
                    assertTrue(corner.z >= bounds.min.z - EPSILON && corner.z <= bounds.max.z + EPSILON);
                }
            }
        }
    }

    private void assertPoint(Coordinate expected, Coordinate actual) {
        assertEquals(expected.x, actual.x, EPSILON);
        assertEquals(expected.y, actual.y, EPSILON);
        assertEquals(expected.z, actual.z, EPSILON);
    }
}
