package info.openrocket.swing.gui.rocketfigure;

import info.openrocket.core.rocketcomponent.TabControlledTrapezoidFinSet;
import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.Transformation;
import java.awt.Rectangle;
import java.awt.Shape;
import java.awt.geom.*;
import java.util.*;
import java.util.List;

/** Hides edges occluded by the other half of a fin/tab assembly, without changing its geometry. */
final class FinTabVisibility {
    private static final double EPS = 1e-10;
    private final List<Triangle> fixed;
    private final List<Triangle> tab;
    private final boolean back;

    FinTabVisibility(TabControlledTrapezoidFinSet fin, Transformation transform, boolean back) {
        this.back = back;
        var profile = new FinProfileGeometry(fin);
        fixed = triangles(profile.faces(FinSetShapes.previewFinOutline(fin)), transform, back);
        tab = triangles(profile.faces(fin.getRollCtrlTabNeutralPoints()),
                transform.applyTransformation(fin.getRollCtrlTabRotation()), back);
    }

    Shape fixed(Shape shape) { return clip(shape, fixed, tab); }
    Shape tab(Shape shape) {
        Shape clipped = clip(shape, tab, fixed);
        var lines = new Path2D.Double(clipped);
        var surfaces = new ArrayList<>(fixed);
        surfaces.addAll(tab);
        // A finite-thickness tab intersects the fixed fin at the hinge. The intersection
        // of the visible faces is the seam; neither solid's original edge follows it.
        for (var a : fixed) if (back ? a.area < 0 : a.area > 0)
            for (var b : tab) if (back ? b.area < 0 : b.area > 0)
                intersection(lines, a, b, surfaces);
        return new VisibleOutline(shape, lines);
    }

    private static void intersection(Path2D.Double lines, Triangle a, Triangle b, List<Triangle> surfaces) {
        if (Math.max(a.a.x, Math.max(a.b.x, a.c.x)) < Math.min(b.a.x, Math.min(b.b.x, b.c.x)) - EPS
                || Math.min(a.a.x, Math.min(a.b.x, a.c.x)) > Math.max(b.a.x, Math.max(b.b.x, b.c.x)) + EPS
                || Math.max(a.a.y, Math.max(a.b.y, a.c.y)) < Math.min(b.a.y, Math.min(b.b.y, b.c.y)) - EPS
                || Math.min(a.a.y, Math.min(a.b.y, a.c.y)) > Math.max(b.a.y, Math.max(b.b.y, b.c.y)) + EPS) return;
        Coordinate[] vertices = {a.a, a.b, a.c};
        double[] delta = new double[3];
        boolean coplanar = true;
        for (int i = 0; i < 3; i++) {
            var p = vertices[i];
            var w = b.weights(p.x, p.y);
            delta[i] = p.z - (w[0] * b.a.z + w[1] * b.b.z + w[2] * b.c.z);
            coplanar &= Math.abs(delta[i]) < EPS;
        }
        if (coplanar) return;
        var points = new ArrayList<Coordinate>();
        for (int i = 0; i < 3; i++) {
            int j = (i + 1) % 3;
            if (Math.abs(delta[i]) < EPS) points.add(vertices[i]);
            if (delta[i] * delta[j] < 0) points.add(vertices[i].add(vertices[j].sub(vertices[i])
                    .multiply(delta[i] / (delta[i] - delta[j]))));
        }
        Coordinate from = null, to = null;
        double length = EPS;
        for (var p : points) for (var q : points) {
            double distance = Math.hypot(p.x - q.x, p.y - q.y);
            if (distance > length) { length = distance; from = p; to = q; }
        }
        if (from == null) return;
        var overlap = b.intersect(from.x, from.y, to.x, to.y, false);
        if (overlap == null) return;
        var first = from.add(to.sub(from).multiply(overlap.from));
        var last = from.add(to.sub(from).multiply(overlap.to));
        if (Math.hypot(last.x - first.x, last.y - first.y) <= EPS) return;
        segment(lines, first.x, first.y, last.x, last.y, List.of(a), surfaces);
    }

    /** x/y are screen coordinates; z is depth towards the viewer. */
    private record Triangle(Coordinate a, Coordinate b, Coordinate c, double area) {
        double[] weights(double x, double y) {
            double u = ((b.x - x) * (c.y - y) - (b.y - y) * (c.x - x)) / area;
            double v = ((c.x - x) * (a.y - y) - (c.y - y) * (a.x - x)) / area;
            return new double[]{u, v, 1 - u - v};
        }
        Interval intersect(double x0, double y0, double x1, double y1, boolean other) {
            if (Math.max(x0, x1) < Math.min(a.x, Math.min(b.x, c.x)) - EPS
                    || Math.min(x0, x1) > Math.max(a.x, Math.max(b.x, c.x)) + EPS
                    || Math.max(y0, y1) < Math.min(a.y, Math.min(b.y, c.y)) - EPS
                    || Math.min(y0, y1) > Math.max(a.y, Math.max(b.y, c.y)) + EPS) return null;
            double[] from = weights(x0, y0), to = weights(x1, y1);
            double low = 0, high = 1;
            for (int i = 0; i < 3; i++) {
                double delta = to[i] - from[i];
                if (Math.abs(delta) < EPS) {
                    if (from[i] < -EPS) return null;
                } else if (delta > 0) low = Math.max(low, -from[i] / delta);
                else high = Math.min(high, -from[i] / delta);
            }
            if (high - low <= EPS) return null;
            double depth0 = from[0] * a.z + from[1] * b.z + from[2] * c.z;
            double depth1 = to[0] * a.z + to[1] * b.z + to[2] * c.z;
            return new Interval(low, high, depth0, depth1 - depth0, other);
        }
    }

    private record Interval(double from, double to, double depth0, double slope, boolean other) {
        double depth(double t) { return depth0 + slope * t; }
    }

    private static List<Triangle> triangles(List<FinProfileGeometry.Face> faces,
            Transformation transform, boolean back) {
        var result = new ArrayList<Triangle>();
        for (var face : faces) {
            var p = transform.transform(face.points().clone());
            if (back) for (int i = 0; i < p.length; i++) p[i] = new Coordinate(p[i].z, p[i].y, p[i].x);
            for (int i = 1; i < p.length - 1; i++) {
                var a = p[0]; var b = p[i]; var c = p[i + 1];
                double area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
                if (Math.abs(area) > 1e-18) result.add(new Triangle(a, b, c, area));
            }
        }
        return result;
    }

    private static Shape clip(Shape source, List<Triangle> own, List<Triangle> other) {
        var visible = new Path2D.Double();
        var iterator = source.getPathIterator(null, 1e-7);
        double[] p = new double[6];
        double startX = 0, startY = 0, x = 0, y = 0;
        boolean hidden = false;
        while (!iterator.isDone()) {
            int kind = iterator.currentSegment(p);
            if (kind == PathIterator.SEG_MOVETO) {
                startX = x = p[0]; startY = y = p[1];
            } else {
                double nextX = kind == PathIterator.SEG_CLOSE ? startX : p[0];
                double nextY = kind == PathIterator.SEG_CLOSE ? startY : p[1];
                hidden |= segment(visible, x, y, nextX, nextY, own, other);
                x = nextX; y = nextY;
            }
            iterator.next();
        }
        // Keep the original paths when there was no overlap, including zero-thickness fins.
        return hidden ? new VisibleOutline(source, visible) : source;
    }

    private static boolean segment(Path2D.Double path, double x0, double y0, double x1, double y1,
            List<Triangle> own, List<Triangle> other) {
        var intervals = new ArrayList<Interval>();
        for (var triangle : own) {
            var interval = triangle.intersect(x0, y0, x1, y1, false);
            if (interval != null) intervals.add(interval);
        }
        for (var triangle : other) {
            var interval = triangle.intersect(x0, y0, x1, y1, true);
            if (interval != null) intervals.add(interval);
        }
        var boundaries = new TreeSet<Double>();
        boundaries.add(0.0); boundaries.add(1.0);
        for (var interval : intervals) { boundaries.add(interval.from); boundaries.add(interval.to); }
        var coarse = new ArrayList<>(boundaries);
        // Depth ordering can change inside an overlap, not just at projected triangle edges.
        for (int i = 1; i < coarse.size(); i++) {
            double low = coarse.get(i - 1), high = coarse.get(i), mid = (low + high) / 2;
            var active = intervals.stream().filter(a -> a.from <= mid && mid <= a.to).toList();
            for (int a = 0; a < active.size(); a++) for (int b = a + 1; b < active.size(); b++) {
                var first = active.get(a); var second = active.get(b);
                double delta = first.slope - second.slope;
                if (Math.abs(delta) < 1e-18) continue;
                double t = (second.depth0 - first.depth0) / delta;
                if (t > low + EPS && t < high - EPS) boundaries.add(t);
            }
        }
        var cuts = new ArrayList<Double>();
        for (double value : boundaries) {
            if (!cuts.isEmpty() && value - cuts.get(cuts.size() - 1) < EPS)
                cuts.set(cuts.size() - 1, value);
            else cuts.add(value);
        }
        boolean hidden = false;
        double visibleStart = Double.NaN;
        for (int i = 1; i < cuts.size(); i++) {
            double low = cuts.get(i - 1), high = cuts.get(i), mid = (low + high) / 2;
            double ownDepth = Double.NEGATIVE_INFINITY, otherDepth = Double.NEGATIVE_INFINITY;
            for (var a : intervals) if (a.from <= mid && mid <= a.to) {
                if (a.other) otherDepth = Math.max(otherDepth, a.depth(mid));
                else ownDepth = Math.max(ownDepth, a.depth(mid));
            }
            boolean covered = Double.isFinite(ownDepth) && otherDepth > ownDepth + EPS;
            if (!covered && Double.isNaN(visibleStart)) visibleStart = low;
            if (covered || i == cuts.size() - 1) {
                if (!Double.isNaN(visibleStart)) {
                    double end = covered ? low : high;
                    if ((end - visibleStart) * Math.hypot(x1 - x0, y1 - y0) > EPS) {
                        path.moveTo(x0 + (x1 - x0) * visibleStart, y0 + (y1 - y0) * visibleStart);
                        path.lineTo(x0 + (x1 - x0) * end, y0 + (y1 - y0) * end);
                    }
                    visibleStart = Double.NaN;
                }
            }
            hidden |= covered;
        }
        return hidden;
    }

    /** Open visible lines must not change selection hit-testing or the rocket's drawing bounds. */
    private record VisibleOutline(Shape selection, Path2D.Double lines) implements Shape {
        public Rectangle getBounds() { return selection.getBounds(); }
        public Rectangle2D getBounds2D() { return selection.getBounds2D(); }
        public boolean contains(double x, double y) { return selection.contains(x, y); }
        public boolean contains(Point2D p) { return selection.contains(p); }
        public boolean contains(double x, double y, double w, double h) { return selection.contains(x, y, w, h); }
        public boolean contains(Rectangle2D r) { return selection.contains(r); }
        public boolean intersects(double x, double y, double w, double h) { return selection.intersects(x, y, w, h); }
        public boolean intersects(Rectangle2D r) { return selection.intersects(r); }
        public PathIterator getPathIterator(AffineTransform t) { return lines.getPathIterator(t); }
        public PathIterator getPathIterator(AffineTransform t, double flatness) { return lines.getPathIterator(t, flatness); }
    }
}
