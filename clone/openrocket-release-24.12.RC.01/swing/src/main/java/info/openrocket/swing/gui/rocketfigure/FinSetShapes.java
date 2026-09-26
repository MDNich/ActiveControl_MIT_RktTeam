package info.openrocket.swing.gui.rocketfigure;

import java.awt.Shape;
import java.awt.geom.Area;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import info.openrocket.core.rocketcomponent.FinSet;
import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.rocketcomponent.SymmetricComponent;
import info.openrocket.core.rocketcomponent.TabControlledTrapezoidFinSet;
import info.openrocket.core.rocketcomponent.TrapezoidFinSet;
import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.MathUtil;
import info.openrocket.core.util.Transformation;


public class FinSetShapes extends RocketComponentShapes {
	@Override
	public Class<? extends RocketComponent> getShapeClass() {
		return FinSet.class;
	}


	@Override
	public RocketComponentShapes[] getShapesSide(final RocketComponent component,
												 final Transformation transformation){
		final FinSet finset = (FinSet) component;
		final Transformation compositeTransform = transformation.applyTransformation(finset.getCantRotation());
		ArrayList<RocketComponentShapes> shapeList = new ArrayList<>();
		Shape[] finShapes = plateShapes(finset, previewFinOutline(finset), compositeTransform, true);
		shapeList.add(new RocketComponentShapes(finShapes[0], finset));
		Shape[] tabShapes = plateShapes(finset, finset.getTabPointsWithRoot(), compositeTransform, false);
		Path2D.Double tab = new Path2D.Double(tabShapes[0]);
		tab.append(tabShapes[1], false);
		shapeList.add(new RocketComponentShapes(tab, finset));
		if (finset instanceof TabControlledTrapezoidFinSet controlled) {
			shapeList.add(new RocketComponentShapes(controlTabShape(controlled, compositeTransform, false), finset));
		}
		shapeList.add(new RocketComponentShapes(finShapes[1], finset));
		Path2D fillets = filletShape(finset, compositeTransform, false);
		if (fillets.getCurrentPoint() != null) {
			shapeList.add(new RocketComponentShapes(fillets, finset));
		}

		if (finset instanceof TrapezoidFinSet && finset.getCrossSection() == FinSet.CrossSection.TRIANGULAR) {
			// The crease lies on the visible face, rather than the fin's centre plane.
			double side = Math.copySign(1, compositeTransform.linearTransform(new Coordinate(0, 0, 1)).z);
			Transformation surface = compositeTransform.applyTransformation(
					new Transformation(0, 0, side * finset.getThickness() / 2));
			Path2D bevel = leadingEdgeBevelShape(finset, surface);
			if (bevel.getCurrentPoint() != null) {
				shapeList.add(new RocketComponentShapes(bevel, finset));
			}
		}

		return shapeList.toArray(new RocketComponentShapes[0]);
	}

	private static Shape controlTabShape(TabControlledTrapezoidFinSet finset, Transformation transform, boolean back) {
		// Rotate the plate and its thickness together before cant and instance rotation.
		Transformation tabTransform = transform.applyTransformation(finset.getRollCtrlTabRotation());
		Shape[] faces = plateShapes(finset, finset.getRollCtrlTabNeutralPoints(), tabTransform, false, back);
		Path2D.Double tab = new Path2D.Double(faces[0]);
		tab.append(faces[1], false);
		return tab;
	}

	/** Leave an opening in the fixed fin where the control tab can swing out. */
	private static Coordinate[] previewFinOutline(FinSet finset) {
		Coordinate[] outline = finset.getFinPointsWithRoot();
		if (!(finset instanceof TabControlledTrapezoidFinSet controlled)
				|| controlled.getTabSpan() <= 0 || controlled.getTabChord() <= 0) {
			return outline;
		}
		int trailingEdge = finset.getFinPoints().length - 2;
		double edgeLength = outline[trailingEdge].sub(outline[trailingEdge + 1]).length();
		if (controlled.getTabOffset() < 0 || controlled.getTabOffset() + controlled.getTabSpan() > edgeLength) {
			return outline;
		}
		Coordinate[] tab = controlled.getRollCtrlTabNeutralPoints();
		Path2D boundary = projectPath(outline, Transformation.IDENTITY, false, true);
		for (int i : new int[]{1, 2}) {
			boolean inside = boundary.contains(tab[i].x, tab[i].y);
			for (int j = 0; !inside && j < outline.length; j++) {
				Coordinate a = outline[j];
				Coordinate b = outline[(j + 1) % outline.length];
				inside = Line2D.ptSegDist(a.x, a.y, b.x, b.y, tab[i].x, tab[i].y) <= MathUtil.EPSILON;
			}
			// Preserve the original outline for tab dimensions that extend outside the fin.
			if (!inside) {
				return outline;
			}
		}
		ArrayList<Coordinate> fixedFin = new ArrayList<>();
		for (int i = 0; i < outline.length; i++) {
			fixedFin.add(outline[i]);
			if (i == trailingEdge) {
				fixedFin.addAll(Arrays.asList(tab[3], tab[2], tab[1], tab[0]));
			}
		}
		return fixedFin.toArray(new Coordinate[0]);
	}

	/** Project the visible face and edge walls of a plate with its actual thickness. */
	private static Shape[] plateShapes(FinSet finset, Coordinate[] outline, Transformation transform, boolean bevel) {
		return plateShapes(finset, outline, transform, bevel, false);
	}

	private static Shape[] plateShapes(FinSet finset, Coordinate[] outline, Transformation transform, boolean bevel, boolean back) {
		ArrayList<Coordinate> boundary = new ArrayList<>();
		for (int i = 0; i < outline.length; i++) {
			Coordinate a = outline[i];
			Coordinate b = outline[(i + 1) % outline.length];
			boundary.add(a);
			// Keep the break in slope at the tip and root of a triangular nose.
			double da = bevel ? bevelFraction(finset, a) : 1;
			double db = bevel ? bevelFraction(finset, b) : 1;
			if ((da < 1 && db > 1) || (da > 1 && db < 1)) {
				boundary.add(a.add(b.sub(a).multiply((1 - da) / (db - da))));
			}
		}
		Coordinate[] front = new Coordinate[boundary.size()];
		Coordinate[] rear = new Coordinate[boundary.size()];
		Coordinate normal = transform.linearTransform(new Coordinate(0, 0, 1));
		double side = Math.copySign(1, back ? normal.x : normal.z);
		double area = 0;
		for (int i = 0; i < boundary.size(); i++) {
			Coordinate point = boundary.get(i);
			Coordinate next = boundary.get((i + 1) % boundary.size());
			area += point.x * next.y - next.x * point.y;
			double halfThickness = finset.getThickness() / 2 * (bevel ? MathUtil.clamp(bevelFraction(finset, point), 0, 1) : 1);
			front[i] = point.add(0, 0, side * halfThickness);
			rear[i] = point.add(0, 0, -side * halfThickness);
		}
		Path2D.Double edges = new Path2D.Double();
		for (int i = 0; i < boundary.size(); i++) {
			int next = (i + 1) % boundary.size();
			Coordinate edge = boundary.get(next).sub(boundary.get(i));
			Coordinate outward = new Coordinate(edge.y, -edge.x, 0).multiply(Math.signum(area));
			Coordinate facing = transform.linearTransform(outward);
			if ((back ? facing.x : facing.z) > MathUtil.EPSILON) {
				edges.append(projectPath(new Coordinate[]{front[i], front[next], rear[next], rear[i]},
						transform, back, true), false);
			}
		}
		return new Shape[]{projectPath(front, transform, back, true), edges};
	}

	/** Chordwise position relative to the triangular nose's end; other profiles are plates. */
	private static double bevelFraction(FinSet finset, Coordinate point) {
		if (!(finset instanceof TrapezoidFinSet) || finset.getCrossSection() != FinSet.CrossSection.TRIANGULAR) {
			return 1;
		}
		Coordinate[] points = finset.getFinPoints();
		double height = points[1].y - points[0].y;
		double distance = finset.getLeadingEdgeDistance();
		if (Math.abs(height) <= MathUtil.EPSILON || distance <= MathUtil.EPSILON) {
			return 1;
		}
		double leadingX = points[0].x + (point.y - points[0].y) * (points[1].x - points[0].x) / height;
		return (point.x - leadingX) / distance;
	}

	private static Path2D.Double projectPath(Coordinate[] points, Transformation transform, boolean back, boolean close) {
		Path2D.Double path = new Path2D.Double();
		for (int i = 0; i < points.length; i++) {
			Coordinate point = transform.transform(points[i]);
			double x = back ? point.z : point.x;
			if (i == 0) {
				path.moveTo(x, point.y);
			} else {
				path.lineTo(x, point.y);
			}
		}
		if (close && points.length > 0) {
			path.closePath();
		}
		return path;
	}

	/**
	 * Circular concave fillets tangent to the fin faces and local body cross-section,
	 * following the sampled root (the same local-section approximation as the mass model).
	 * The projected silhouette hides the loft mesh and clips the fillet height to the
	 * fin planform near swept leading and trailing edges.
	 */
	private static Path2D filletShape(FinSet finset, Transformation transform, boolean back) {
		Path2D.Double path = new Path2D.Double();
		double radius = finset.getFilletRadius();
		if (!(finset.getParent() instanceof SymmetricComponent body) || !Double.isFinite(radius)
				|| radius <= 0 || finset.getSpan() <= MathUtil.EPSILON) {
			return path;
		}
		Coordinate[] roots = finset.getRootPoints();
		if (roots.length < 2) {
			return path;
		}
		// Add stations even on straight tubes so the ends can taper with the fin.
		ArrayList<Coordinate> stations = new ArrayList<>();
		double step = Math.max(finset.getLength() / 24, MathUtil.EPSILON);
		for (int i = 1; i < roots.length; i++) {
			Coordinate a = roots[i - 1];
			Coordinate delta = roots[i].sub(a);
			int divisions = Math.max(1, (int) Math.ceil(Math.abs(delta.x) / step));
			for (int j = 0; j < divisions; j++) {
				stations.add(a.add(delta.multiply((double) j / divisions)));
			}
		}
		stations.add(roots[roots.length - 1]);
		final int arcSteps = 16;
		for (double side : new double[]{-1, 1}) {
			Coordinate[] previous = null;
			for (Coordinate root : stations) {
				double bodyRadius = body.getRadius(finset.getFinFront().x + root.x);
				double halfThickness = finset.getThickness() / 2 * MathUtil.clamp(bevelFraction(finset, root), 0, 1);
				if (!Double.isFinite(bodyRadius) || bodyRadius <= halfThickness) {
					previous = null;
					continue;
				}
				double localRadius = Math.min(radius, availableFinHeight(finset, root));
				double centerZ = halfThickness + localRadius;
				double centerY = Math.sqrt(MathUtil.pow2(bodyRadius + localRadius) - MathUtil.pow2(centerZ));
				double angle = Math.acos(centerZ / (bodyRadius + localRadius));
				ArrayList<Coordinate> section = new ArrayList<>();
				section.add(new Coordinate(root.x,
						root.y + Math.sqrt(bodyRadius * bodyRadius - halfThickness * halfThickness) - bodyRadius,
						side * halfThickness));
				for (int i = 0; i <= arcSteps; i++) {
					double theta = angle * i / arcSteps;
					section.add(new Coordinate(root.x, root.y + centerY - localRadius * Math.sin(theta) - bodyRadius,
							side * (centerZ - localRadius * Math.cos(theta))));
				}
				// Close each end along the body surface, rather than through the tube.
				double bodyAngle = Math.asin(centerZ / (bodyRadius + localRadius));
				double footAngle = Math.asin(halfThickness / bodyRadius);
				for (int i = 1; i <= arcSteps; i++) {
					double theta = bodyAngle + (footAngle - bodyAngle) * i / arcSteps;
					section.add(new Coordinate(root.x, root.y + bodyRadius * (Math.cos(theta) - 1),
							side * bodyRadius * Math.sin(theta)));
				}
				Coordinate[] current = section.toArray(new Coordinate[0]);
				appendProjectedFace(path, current, transform, back);
				if (previous != null) {
					for (int i = 0; i < current.length; i++) {
						int next = (i + 1) % current.length;
						appendProjectedFace(path, new Coordinate[]{previous[i], previous[next], current[next], current[i]},
								transform, back);
					}
				}
				previous = current;
			}
		}
		return new Path2D.Double(new Area(path));
	}

	private static double availableFinHeight(FinSet finset, Coordinate root) {
		double height = 0;
		Coordinate[] outline = finset.getFinPoints();
		for (int i = 1; i < outline.length; i++) {
			Coordinate a = outline[i - 1];
			Coordinate b = outline[i];
			if (root.x < Math.min(a.x, b.x) || root.x > Math.max(a.x, b.x)) {
				continue;
			}
			double y = a.x == b.x ? Math.max(a.y, b.y) : a.y + (b.y - a.y) * (root.x - a.x) / (b.x - a.x);
			height = Math.max(height, y - root.y);
		}
		return height;
	}

	/** All projected faces use the same winding so Area merges overlaps without holes. */
	private static void appendProjectedFace(Path2D.Double path, Coordinate[] face, Transformation transform, boolean back) {
		Coordinate[] projected = transform.transform(face.clone());
		double area = 0;
		for (int i = 0; i < projected.length; i++) {
			Coordinate a = projected[i];
			Coordinate b = projected[(i + 1) % projected.length];
			area += (back ? a.z : a.x) * b.y - (back ? b.z : b.x) * a.y;
		}
		if (Math.abs(area) < 1e-18) {
			return;
		}
		for (int i = 0; i < projected.length; i++) {
			Coordinate point = projected[area > 0 ? i : projected.length - 1 - i];
			double x = back ? point.z : point.x;
			if (i == 0) {
				path.moveTo(x, point.y);
			} else {
				path.lineTo(x, point.y);
			}
		}
		path.closePath();
	}

	/**
	 * Draw the crease where the triangular nose reaches the full plate thickness.
	 * The existing leading-edge distance is measured along the chord (local X).
	 * Clip the parallel crease to the actual outline, including the curved root of
	 * a canted fin, before applying the same projection as the fin.
	 */
	private static Path2D leadingEdgeBevelShape(FinSet finset, Transformation transformation) {
		Path2D.Double crease = new Path2D.Double();
		double distance = finset.getLeadingEdgeDistance();
		if (!Double.isFinite(distance) || distance <= 0 || finset.getSpan() <= MathUtil.EPSILON) {
			return crease;
		}

		Coordinate[] outline = previewFinOutline(finset);
		Coordinate start = outline[0].add(distance, 0, 0);
		Coordinate direction = outline[1].sub(outline[0]);
		Path2D.Double boundary = new Path2D.Double();
		boundary.moveTo(outline[0].x, outline[0].y);
		for (int i = 1; i < outline.length; i++) {
			boundary.lineTo(outline[i].x, outline[i].y);
		}
		boundary.closePath();

		// Intersect the infinite crease line with each boundary edge. Using every
		// intersection also handles a non-convex root without drawing through the body.
		ArrayList<Double> intersections = new ArrayList<>();
		for (int i = 0; i < outline.length; i++) {
			Coordinate a = outline[i];
			Coordinate edge = outline[(i + 1) % outline.length].sub(a);
			double determinant = direction.x * edge.y - direction.y * edge.x;
			if (determinant == 0) {
				continue;
			}
			Coordinate offset = a.sub(start);
			double edgeFraction = (offset.x * direction.y - offset.y * direction.x) / determinant;
			if (edgeFraction >= 0 && edgeFraction <= 1) {
				intersections.add((offset.x * edge.y - offset.y * edge.x) / determinant);
			}
		}
		Collections.sort(intersections);
		for (int i = 1; i < intersections.size(); i++) {
			double from = intersections.get(i - 1);
			double to = intersections.get(i);
			Coordinate midpoint = start.add(direction.multiply((from + to) / 2));
			if (to <= from || !boundary.contains(midpoint.x, midpoint.y)) {
				continue;
			}
			Coordinate a = transformation.transform(start.add(direction.multiply(from)));
			Coordinate b = transformation.transform(start.add(direction.multiply(to)));
			crease.moveTo(a.x, a.y);
			crease.lineTo(b.x, b.y);
		}
		return crease;
	}

	@Override
	public RocketComponentShapes[] getShapesBack(final RocketComponent component, final Transformation transformation) {

		FinSet finset = (FinSet) component;
		
		Shape[] toReturn;

		if (MathUtil.equals(finset.getCantAngle(), 0)) {
			toReturn = uncantedShapesBack(finset, transformation);
		} else {
			toReturn = cantedShapesBack(finset, transformation);
		}


		Transformation compositeTransform = transformation.applyTransformation(finset.getCantRotation());
		if (finset instanceof TabControlledTrapezoidFinSet controlled) {
			toReturn = Arrays.copyOf(toReturn, toReturn.length + 1);
			toReturn[toReturn.length - 1] = controlTabShape(controlled, compositeTransform, true);
		}
		Path2D fillets = filletShape(finset, compositeTransform, true);
		if (fillets.getCurrentPoint() != null) {
			// The rear-facing plate hides fillets on the thinner leading-edge section.
			Area visibleFillets = new Area(fillets);
			for (Shape fin : toReturn) {
				visibleFillets.subtract(new Area(fin));
			}
			toReturn = Arrays.copyOf(toReturn, toReturn.length + 1);
			toReturn[toReturn.length - 1] = visibleFillets;
		}
		return RocketComponentShapes.toArray(toReturn, finset);
	}

	public static Path2D.Float generatePath(final Coordinate[] points){
		Path2D.Float finShape = new Path2D.Float();
		for( int i = 0; i < points.length; i++){
			Coordinate curPoint = points[i];
			if (i == 0)
				finShape.moveTo(curPoint.x, curPoint.y);
			else
				finShape.lineTo(curPoint.x, curPoint.y);
		}
		return finShape;
	}
	
	private static Shape[] uncantedShapesBack(FinSet finset,
			Transformation transformation) {
		
		double thickness = finset.getThickness();
		double height = finset.getSpan();
		double tabHeight = finset.getTabHeight();
		
		// Generate base coordinates for a single fin
		Coordinate[] c = new Coordinate[4];
		c[0]=new Coordinate(0, 0,-thickness/2);
        c[1]=new Coordinate(0, 0,thickness/2);
        c[2]=new Coordinate(0,height,thickness/2);
        c[3]=new Coordinate(0,height,-thickness/2);

		// Generate base coordinates for a single fin tab
		Coordinate[] cTab = new Coordinate[4];
		cTab[0]=new Coordinate(0, 0,-thickness/2);
		cTab[1]=new Coordinate(0, 0,thickness/2);
		cTab[2]=new Coordinate(0, -tabHeight,thickness/2);
		cTab[3]=new Coordinate(0, -tabHeight,-thickness/2);

		// y translate the back view (if there is a fin point with non-zero y value)
		Coordinate[] points = finset.getFinPoints();
		double yOffset = Double.MAX_VALUE;
		for (Coordinate point : points) {
			yOffset = MathUtil.min(yOffset, point.y);
		}
		final Transformation translateOffsetY = new Transformation(0, yOffset, 0);
		final Transformation compositeTransform = transformation.applyTransformation(translateOffsetY);

		// Make polygon
		Shape p = makePolygonBack(c, compositeTransform);

		if (tabHeight != 0 && finset.getTabLength() != 0) {
			Shape pTab = makePolygonBack(cTab, compositeTransform);
			return new Shape[]{p, pTab};
		}
		else {
			return new Shape[]{p};
		}
	}

	private static Shape[] cantedShapesBack(FinSet finset,
												Transformation transformation) {
		if (finset.getTabHeight() == 0 || finset.getTabLength() == 0) {
			return cantedShapesBackFins(finset, transformation);
		}

		Shape[] toReturn;
		Shape[] shapesFin = cantedShapesBackFins(finset, transformation);
		Shape[] shapesTab = cantedShapesBackTabs(finset, transformation);

		toReturn = Arrays.copyOf(shapesFin, shapesFin.length + shapesTab.length);
		System.arraycopy(shapesTab, 0, toReturn, shapesFin.length, shapesTab.length);

		return toReturn;
	}

	private static Shape[] cantedShapesBackFins(FinSet finset,
												Transformation transformation) {
		double thickness = finset.getThickness();
		
		Coordinate[] sidePoints;
		Coordinate[] backPoints;
		int maxIndex;

		Coordinate[] points = finset.getFinPoints();
		
		// this loop finds the index @ max-y, as visible from the back
		for (maxIndex = points.length-1; maxIndex > 0; maxIndex--) {
			if (points[maxIndex-1].y < points[maxIndex].y)
				break;
		}
		 
		Transformation cantTransform = finset.getCantRotation();
		final Transformation compositeTransform = transformation.applyTransformation(cantTransform);
		
		sidePoints = new Coordinate[points.length];
		backPoints = new Coordinate[2*(points.length-maxIndex)];
		double sign = Math.copySign(1.0, finset.getCantAngle());

		// Calculate points for the visible side panel
		for (int i=0; i < points.length; i++) {
			sidePoints[i] = points[i].add(0,0,sign*thickness/2);
		}

		// Calculate points for the back portion
		int i=0;
		for (int j=points.length-1; j >= maxIndex; j--, i++) {
			backPoints[i] = points[j].add(0,0,sign*thickness/2);
		}
		for (int j=maxIndex; j <= points.length-1; j++, i++) {
			backPoints[i] = points[j].add(0,0,-sign*thickness/2);
		}
		
		// Generate shapes
		Shape[] s;
		if (thickness > 0.0005) {
			s = new Shape[2];
			s[0] = makePolygonBack(sidePoints,compositeTransform);
			s[1] = makePolygonBack(backPoints,compositeTransform);
		} else {
			s = new Shape[1];
			s[0] = makePolygonBack(sidePoints,compositeTransform);
		}
		
		return s;
	}

	private static Shape[] cantedShapesBackTabs(FinSet finset,
											Transformation transformation) {
		double thickness = finset.getThickness();

		Coordinate[] sidePoints;
		Coordinate[] backPoints;
		int minIndex;

		Coordinate[] points = finset.getTabPointsWithRoot();

		// this loop finds the index @ min-y, as visible from the back
		for (minIndex = points.length-1; minIndex > 0; minIndex--) {
			if (points[minIndex-1].y > points[minIndex].y)
				break;
		}

		Transformation cantTransform = finset.getCantRotation();
		final Transformation compositeTransform = transformation.applyTransformation(cantTransform);

		sidePoints = new Coordinate[points.length];
		backPoints = new Coordinate[2*(points.length-minIndex)];
		double sign = Math.copySign(1.0, finset.getCantAngle());

		// Calculate points for the visible side panel
		for (int i=0; i < points.length; i++) {
			sidePoints[i] = points[i].add(0,0,sign*thickness/2);
		}

		// Calculate points for the back portion
		int i=0;
		for (int j=points.length-1; j >= minIndex; j--, i++) {
			backPoints[i] = points[j].add(0,0,sign*thickness/2);
		}
		for (int j=minIndex; j <= points.length-1; j++, i++) {
			backPoints[i] = points[j].add(0,0,-sign*thickness/2);
		}

		// Generate shapes
		Shape[] s;
		if (thickness > 0.0005) {
			s = new Shape[2];
			s[0] = makePolygonBack(sidePoints,compositeTransform);
			s[1] = makePolygonBack(backPoints,compositeTransform);
		} else {
			s = new Shape[1];
			s[0] = makePolygonBack(sidePoints,compositeTransform);
		}

		return s;
	}
	
	private static Shape makePolygonBack(Coordinate[] array, final Transformation t) {
		Path2D.Float p;

		// Make polygon
		p = new Path2D.Float();
		for (int i=0; i < array.length; i++) {
			Coordinate a = t.transform(array[i] );
			if (i==0)
				p.moveTo(a.z, a.y);
			else
				p.lineTo(a.z, a.y);			
		}
		p.closePath();
		return p;
	}

}
