package info.openrocket.swing.gui.rocketfigure;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.awt.Shape;
import java.awt.geom.Line2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Rectangle2D;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import info.openrocket.core.rocketcomponent.BodyTube;
import info.openrocket.core.rocketcomponent.FinSet;
import info.openrocket.core.rocketcomponent.TabControlledTrapezoidFinSet;
import info.openrocket.core.rocketcomponent.TrapezoidFinSet;
import info.openrocket.core.rocketcomponent.position.AxialMethod;
import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.Transformation;
import info.openrocket.swing.util.BaseTestCase;

class FinSetShapesTest extends BaseTestCase {

	private static final double EPSILON = 1e-9;
	private final FinSetShapes renderer = new FinSetShapes();

	private TrapezoidFinSet triangularFin() {
		TrapezoidFinSet fin = new TrapezoidFinSet(3, 0.12, 0.06, 0.08, 0.1);
		fin.setThickness(0.006);
		fin.setCrossSection(FinSet.CrossSection.TRIANGULAR);
		fin.setLeadingEdgeDistance(0.015);
		return fin;
	}

	@ParameterizedTest
	@ValueSource(doubles = {-0.08, 0, 0.08})
	void creaseIsParallelToLeadingEdgeForEitherSweep(double sweep) {
		TrapezoidFinSet fin = triangularFin();
		fin.setSweep(sweep);
		assertCrease(fin, Transformation.IDENTITY,
				new Coordinate(0.015, 0), new Coordinate(sweep + 0.015, 0.1));
	}

	@Test
	void creaseUpdatesWithAngleDistanceAndThickness() {
		TrapezoidFinSet fin = triangularFin();
		fin.setLeadingEdgeAngle(Math.PI / 2);
		assertCrease(fin, Transformation.IDENTITY, new Coordinate(0.003, 0), new Coordinate(0.083, 0.1));
		fin.setThickness(0.012);
		assertCrease(fin, Transformation.IDENTITY, new Coordinate(0.006, 0), new Coordinate(0.086, 0.1));
		fin.setLeadingEdgeDistance(0.02);
		assertCrease(fin, Transformation.IDENTITY, new Coordinate(0.02, 0), new Coordinate(0.1, 0.1));
		fin.setLeadingEdgeAngle(0); // Automatic 20-degree included angle.
		double distance = 0.006 / Math.tan(Math.toRadians(10));
		assertCrease(fin, Transformation.IDENTITY,
				new Coordinate(distance, 0), new Coordinate(0.08 + distance, 0.1));
	}

	@ParameterizedTest
	@ValueSource(doubles = {0, 0.005})
	void narrowTipClipsCreaseAtTrailingEdge(double tipChord) {
		TrapezoidFinSet fin = triangularFin();
		fin.setTipChord(tipChord);
		double fraction = (0.12 - 0.015) / (0.12 - tipChord);
		assertCrease(fin, Transformation.IDENTITY,
				new Coordinate(0.015, 0), new Coordinate(0.015 + 0.08 * fraction, 0.1 * fraction));
	}

	@Test
	void creaseEndsOnCurvedRootOfCantedFin() {
		BodyTube body = new BodyTube(0.4, 0.1);
		TrapezoidFinSet fin = triangularFin();
		body.addChild(fin);
		fin.setAxialMethod(AxialMethod.TOP);
		fin.setAxialOffset(0.1);
		fin.setCantAngle(Math.toRadians(30));
		RocketComponentShapes[] shapes = renderer.getShapesSide(fin, Transformation.IDENTITY);
		assertEquals(4, shapes.length);
		PathIterator crease = shapes[3].shape.getPathIterator(null);
		double[] start = new double[6];
		assertEquals(PathIterator.SEG_MOVETO, crease.currentSegment(start));
		Transformation surface = fin.getCantRotation().applyTransformation(new Transformation(0, 0, 0.003));
		Coordinate[] root = surface.transform(fin.getRootPoints());
		double distanceToRoot = Double.POSITIVE_INFINITY;
		for (int i = 1; i < root.length; i++) {
			distanceToRoot = Math.min(distanceToRoot, Line2D.ptSegDist(
					root[i - 1].x, root[i - 1].y, root[i].x, root[i].y, start[0], start[1]));
		}
		assertEquals(0, distanceToRoot, EPSILON);
		// Simply shifting the outer edge would leave the crease inside the body.
		Coordinate unclipped = surface.transform(fin.getFinPoints()[0].add(0.015, 0, 0));
		assertTrue(start[1] > unclipped.y);
		crease.next();
		double[] end = new double[6];
		assertEquals(PathIterator.SEG_LINETO, crease.currentSegment(end));
		Coordinate tip = surface.transform(new Coordinate(0.095, 0.1));
		assertEquals(tip.x, end[0], EPSILON);
		assertEquals(tip.y, end[1], EPSILON);
		crease.next();
		assertTrue(crease.isDone());
	}

	@Test
	void creaseUsesFinCantAndInstanceTransform() {
		TrapezoidFinSet fin = triangularFin();
		fin.setCantAngle(Math.toRadians(12));
		Transformation instance = new Transformation(0.7, 0.2, 0.1)
				.applyTransformation(Transformation.rotate_x(Math.toRadians(37)));
		Transformation combined = instance.applyTransformation(fin.getCantRotation());
		assertCrease(fin, instance, combined.transform(new Coordinate(0.015, 0, 0.003)),
				combined.transform(new Coordinate(0.095, 0.1, 0.003)));
	}

	@Test
	void otherCrossSectionsKeepTheirExistingShapes() {
		TrapezoidFinSet fin = triangularFin();
		for (FinSet.CrossSection section : FinSet.CrossSection.values()) {
			fin.setCrossSection(section);
			assertEquals(section == FinSet.CrossSection.TRIANGULAR ? 4 : 3,
					renderer.getShapesSide(fin, Transformation.IDENTITY).length);
		}
	}

	@Test
	void noCreaseForDegenerateOrFullyBeveledFins() {
		TrapezoidFinSet fin = triangularFin();
		fin.setThickness(0);
		assertEquals(3, renderer.getShapesSide(fin, Transformation.IDENTITY).length);
		fin.setThickness(0.006);
		fin.setLeadingEdgeDistance(0.3);
		assertEquals(3, renderer.getShapesSide(fin, Transformation.IDENTITY).length);
		fin.setLeadingEdgeDistance(0.015);
		fin.setHeight(0);
		assertEquals(3, renderer.getShapesSide(fin, Transformation.IDENTITY).length);
	}

	@Test
	void controlledTrapezoidAlsoShowsCrease() {
		TabControlledTrapezoidFinSet fin = new TabControlledTrapezoidFinSet();
		fin.setCrossSection(FinSet.CrossSection.TRIANGULAR);
		fin.setLeadingEdgeDistance(0.01);
		assertCrease(fin, Transformation.IDENTITY,
				new Coordinate(0.01, 0), new Coordinate(0.035, 0.03));
	}

	private TabControlledTrapezoidFinSet controlFin(double angle) {
		TabControlledTrapezoidFinSet fin = new TabControlledTrapezoidFinSet(
				4, 0.12, 0.12, 0, 0.08, 0.03, 0.02, 0.015, Math.toRadians(angle));
		fin.setThickness(0.006);
		return fin;
	}

	@ParameterizedTest
	@ValueSource(doubles = {-90, -60, 0, 60, 90})
	void controlTabHasRotatedThicknessAndVisibleRearDeflection(double degrees) {
		TabControlledTrapezoidFinSet fin = controlFin(degrees);
		double angle = Math.toRadians(degrees);
		Shape side = renderer.getShapesSide(fin, Transformation.IDENTITY)[2].shape;
		assertEquals(0.02 * Math.cos(angle) + 0.006 * Math.abs(Math.sin(angle)), side.getBounds2D().getWidth(), EPSILON);
		assertEquals(0.03, side.getBounds2D().getHeight(), EPSILON);
		RocketComponentShapes[] back = renderer.getShapesBack(fin, Transformation.IDENTITY);
		assertEquals(2, back.length);
		Rectangle2D rearBounds = back[1].shape.getBounds2D();
		assertEquals(0.02 * Math.abs(Math.sin(angle)) + 0.006 * Math.cos(angle), rearBounds.getWidth(), EPSILON);
		assertEquals(-0.01 * Math.sin(angle), rearBounds.getCenterX(), EPSILON);
	}

	@Test
	void fixedFinLeavesTheControlTabOpeningVisible() {
		TabControlledTrapezoidFinSet fin = controlFin(90);
		RocketComponentShapes[] shapes = renderer.getShapesSide(fin, Transformation.IDENTITY);
		assertFalse(shapes[0].shape.contains(0.11, 0.03));
		assertTrue(shapes[0].shape.contains(0.09, 0.03));
		assertTrue(shapes[0].shape.contains(0.11, 0.06));
		// At full deflection the plate remains selectable by its 6 mm edge.
		assertTrue(shapes[2].shape.contains(0.10, 0.03));
	}

	@Test
	void positiveAndNegativeDeflectionsMoveOppositeWaysWhenRocketIsRotated() {
		Transformation rotation = Transformation.rotate_x(Math.PI / 2);
		Shape positive = renderer.getShapesSide(controlFin(90), rotation)[2].shape;
		Shape negative = renderer.getShapesSide(controlFin(-90), rotation)[2].shape;
		assertEquals(0.02, positive.getBounds2D().getHeight(), EPSILON);
		assertEquals(-negative.getBounds2D().getCenterY(), positive.getBounds2D().getCenterY(), EPSILON);
		assertTrue(positive.getBounds2D().getCenterY() > 0);
		assertTrue(negative.getBounds2D().getCenterY() < 0);
	}

	@Test
	void tabHingeThicknessIsCenteredAfterCantAndInstanceRotation() {
		TabControlledTrapezoidFinSet fin = controlFin(60);
		fin.setCantAngle(Math.toRadians(10));
		Transformation instance = new Transformation(0.5, 0.2, 0.1)
				.applyTransformation(Transformation.rotate_x(0.7));
		Transformation complete = instance.applyTransformation(fin.getCantRotation())
				.applyTransformation(fin.getRollCtrlTabRotation());
		Coordinate normal = complete.linearTransform(new Coordinate(0, 0, 1));
		Coordinate hingeFace = fin.getRollCtrlTabNeutralPoints()[1].add(0, 0, Math.copySign(0.003, normal.z));
		Coordinate expected = complete.transform(hingeFace);
		Shape tab = renderer.getShapesSide(fin, instance)[2].shape;
		assertEquals(0, distanceToBoundary(tab, expected.x, expected.y), EPSILON);
	}

	@ParameterizedTest
	@ValueSource(doubles = {0, 45, 90, 135, 180})
	void sideProjectionIncludesActualThickness(double angle) {
		TrapezoidFinSet fin = triangularFin();
		fin.setCrossSection(FinSet.CrossSection.SQUARE);
		double rotation = Math.toRadians(angle);
		Transformation transform = Transformation.rotate_x(rotation);
		for (double thickness : new double[]{0, 0.006, 0.012}) {
			fin.setThickness(thickness);
			RocketComponentShapes[] shapes = renderer.getShapesSide(fin, transform);
			Rectangle2D bounds = shapes[0].shape.getBounds2D().createUnion(shapes[2].shape.getBounds2D());
			assertEquals(0.1 * Math.abs(Math.cos(rotation)) + thickness * Math.abs(Math.sin(rotation)),
					bounds.getHeight(), EPSILON);
		}
	}

	@Test
	void triangularThicknessStartsAtZeroAndReachesFullWidthAtCrease() {
		TrapezoidFinSet fin = triangularFin();
		Shape face = renderer.getShapesSide(fin, Transformation.rotate_x(Math.PI / 2))[0].shape;
		PathIterator points = face.getPathIterator(null);
		double[] point = new double[6];
		boolean foundLeadingEdge = false;
		boolean foundFullThickness = false;
		while (!points.isDone()) {
			if (points.currentSegment(point) != PathIterator.SEG_CLOSE) {
				if (Math.abs(point[0]) < EPSILON) {
					assertEquals(0, point[1], EPSILON);
					foundLeadingEdge = true;
				}
				if (Math.abs(point[0] - 0.015) < EPSILON) {
					assertEquals(-0.003, point[1], EPSILON);
					foundFullThickness = true;
				}
			}
			points.next();
		}
		assertTrue(foundLeadingEdge && foundFullThickness);
	}

	@Test
	void filletsAreTangentToFinFacesAndBodyAtConfiguredRadius() {
		TrapezoidFinSet fin = triangularFin();
		fin.setCrossSection(FinSet.CrossSection.SQUARE);
		new BodyTube(0.5, 0.05).addChild(fin);
		fin.setFilletRadius(0.01);
		RocketComponentShapes[] shapes = renderer.getShapesBack(fin, Transformation.IDENTITY);
		Shape fillet = shapes[shapes.length - 1].shape;
		double centerZ = 0.003 + 0.01;
		double centerY = Math.sqrt(0.06 * 0.06 - centerZ * centerZ);
		Rectangle2D bounds = fillet.getBounds2D();
		assertEquals(2 * centerZ * 0.05 / 0.06, bounds.getWidth(), EPSILON);
		assertEquals(centerY - 0.05, bounds.getMaxY(), EPSILON);
		double angle = Math.acos(centerZ / 0.06);
		for (int i = 0; i <= 16; i++) {
			double theta = angle * i / 16;
			double z = -centerZ + 0.01 * Math.cos(theta);
			double y = centerY - 0.01 * Math.sin(theta) - 0.05;
			assertEquals(0, distanceToBoundary(fillet, z, y), EPSILON);
		}
	}

	@Test
	void filletEndsStayBelowSweptFinOutline() {
		TrapezoidFinSet fin = triangularFin();
		new BodyTube(0.5, 0.05).addChild(fin);
		fin.setFilletRadius(0.01);
		Shape fillet = renderer.getShapesSide(fin, Transformation.IDENTITY)[3].shape;
		PathIterator points = fillet.getPathIterator(null);
		double[] point = new double[6];
		while (!points.isDone()) {
			if (points.currentSegment(point) != PathIterator.SEG_CLOSE && point[0] < 0.08) {
				assertTrue(point[1] <= point[0] * 0.1 / 0.08 + EPSILON);
			}
			points.next();
		}
	}

	@Test
	void filletsFollowInstanceRotationAndDisappearAtZeroRadius() {
		TrapezoidFinSet fin = triangularFin();
		fin.setCrossSection(FinSet.CrossSection.SQUARE);
		new BodyTube(0.5, 0.05).addChild(fin);
		assertEquals(3, renderer.getShapesSide(fin, Transformation.IDENTITY).length);
		assertEquals(1, renderer.getShapesBack(fin, Transformation.IDENTITY).length);
		fin.setFilletRadius(0.01);
		Shape back = renderer.getShapesBack(fin, Transformation.IDENTITY)[1].shape;
		Transformation rotated = Transformation.rotate_x(Math.PI / 2);
		Shape side = renderer.getShapesSide(fin, rotated)[3].shape;
		// Rotating 90 degrees about the rocket axis projects the rear-view width vertically.
		assertEquals(back.getBounds2D().getWidth(), side.getBounds2D().getHeight(), EPSILON);
		Shape moved = renderer.getShapesSide(fin, new Transformation(0.4, 0.2, 0).applyTransformation(rotated))[3].shape;
		assertEquals(side.getBounds2D().getMinX() + 0.4, moved.getBounds2D().getMinX(), EPSILON);
		assertEquals(side.getBounds2D().getMinY() + 0.2, moved.getBounds2D().getMinY(), EPSILON);
		fin.setFilletRadius(0);
		assertEquals(3, renderer.getShapesSide(fin, rotated).length);
		assertEquals(1, renderer.getShapesBack(fin, rotated).length);
	}

	private double distanceToBoundary(Shape shape, double x, double y) {
		PathIterator path = shape.getPathIterator(null, EPSILON);
		double[] point = new double[6];
		double startX = 0, startY = 0, previousX = 0, previousY = 0;
		double distance = Double.POSITIVE_INFINITY;
		while (!path.isDone()) {
			int type = path.currentSegment(point);
			if (type == PathIterator.SEG_MOVETO) {
				startX = point[0];
				startY = point[1];
			} else {
				if (type == PathIterator.SEG_CLOSE) {
					point[0] = startX;
					point[1] = startY;
				}
				distance = Math.min(distance, Line2D.ptSegDist(previousX, previousY, point[0], point[1], x, y));
			}
			previousX = point[0];
			previousY = point[1];
			path.next();
		}
		return distance;
	}

	private void assertCrease(FinSet fin, Transformation transformation, Coordinate start, Coordinate end) {
		RocketComponentShapes[] shapes = renderer.getShapesSide(fin, transformation);
		assertEquals(fin instanceof TabControlledTrapezoidFinSet ? 5 : 4, shapes.length);
		Shape crease = shapes[shapes.length - 1].shape;
		PathIterator path = crease.getPathIterator(null);
		double[] point = new double[6];
		assertEquals(PathIterator.SEG_MOVETO, path.currentSegment(point));
		assertEquals(start.x, point[0], EPSILON);
		assertEquals(start.y, point[1], EPSILON);
		path.next();
		assertEquals(PathIterator.SEG_LINETO, path.currentSegment(point));
		assertEquals(end.x, point[0], EPSILON);
		assertEquals(end.y, point[1], EPSILON);
		path.next();
		assertTrue(path.isDone());
	}
}
