package info.openrocket.swing.gui.rocketfigure;

import info.openrocket.core.rocketcomponent.FinSet;
import info.openrocket.core.util.Coordinate;
import java.util.*;

/** Drawing geometry only: section categories do not specify a measured or named airfoil. */
public final class FinProfileGeometry {
    public record Face(Coordinate[] points, int side) {} // +1/-1 faces, 0 edge caps
    private final FinSet fin;
    private final Coordinate[] outline;
    private static final int STEPS = 16;

    public FinProfileGeometry(FinSet fin) {
        this.fin = fin;
        this.outline = fin.getFinPointsWithRoot();
    }

    /** Symmetric section within the unchanged chord and specified maximum thickness. */
    public static double halfThickness(FinSet.CrossSection section, double x, double chord,
            double thickness, double leadingDistance) {
        if (chord <= 1e-12 || thickness <= 0) return 0;
        double u = Math.max(0, Math.min(1, x / chord)), half = thickness / 2;
        return switch (section) {
            case SQUARE -> half;
            case TRIANGULAR -> half * Math.min(1, x / Math.max(leadingDistance, 1e-12));
            case ROUNDED -> {
                double radius = Math.min(half, chord / 2);
                double d = Math.min(x, chord - x);
                double v = Math.max(0, Math.min(1, d / radius));
                yield half * Math.sqrt(Math.max(0, 1 - (1 - v) * (1 - v)));
            }
            case AIRFOIL -> {
                // Generic symmetric section: round nose, maximum at 30% chord, tapered tail.
                double v = u <= .3 ? u / .3 : (u - .3) / .7;
                yield half * (u <= .3 ? Math.sqrt(Math.max(0, 1 - (1 - v) * (1 - v)))
                        : (1 - v) * (1 + v - v * v));
            }
        };
    }

    private double[] chord(double y) {
        double min = Double.POSITIVE_INFINITY, max = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < outline.length; i++) {
            Coordinate a = outline[i], b = outline[(i + 1) % outline.length];
            if (y < Math.min(a.y, b.y) - 1e-12 || y > Math.max(a.y, b.y) + 1e-12) continue;
            if (Math.abs(b.y - a.y) < 1e-12) {
                min = Math.min(min, Math.min(a.x, b.x));
                max = Math.max(max, Math.max(a.x, b.x));
            } else {
                double x = a.x + (b.x - a.x) * (y - a.y) / (b.y - a.y);
                min = Math.min(min, x); max = Math.max(max, x);
            }
        }
        return Double.isFinite(min) ? new double[]{min, max} : new double[]{0, 0};
    }

    public double halfThickness(Coordinate point) {
        double[] c = chord(point.y);
        return halfThickness(fin.getCrossSection(), Math.max(0, point.x - c[0]), c[1] - c[0],
                fin.getThickness(), fin.getLeadingEdgeDistance());
    }

    private Coordinate surface(double x, double y, int side) {
        var p = new Coordinate(x, y, 0);
        return p.setZ(side * halfThickness(p));
    }

    /** Fixed-size section sampling clusters vertices around the small rounded nose/tail. */
    private double fraction(int i, double chord) {
        if (fin.getCrossSection() == FinSet.CrossSection.ROUNDED) {
            double r = Math.min(.5, fin.getThickness() / (2 * Math.max(chord, 1e-12)));
            if (i <= STEPS) return r * (1 - Math.cos(Math.PI * i / (2 * STEPS)));
            return 1 - r * (1 - Math.sin(Math.PI * (i - STEPS - 1) / (2 * STEPS)));
        }
        if (fin.getCrossSection() == FinSet.CrossSection.TRIANGULAR) {
            double end = Math.min(1, fin.getLeadingEdgeDistance() / Math.max(chord, 1e-12));
            return i <= STEPS ? end * i / STEPS : end + (1 - end) * (i - STEPS) / (STEPS + 1);
        }
        if (i <= STEPS) return .3 * (1 - Math.cos(Math.PI * i / (2 * STEPS)));
        return .3 + .7 * (i - STEPS) / (STEPS + 1);
    }

    private double edgeX(Coordinate[] boundary, int edge, double y) {
        Coordinate a = boundary[edge], b = boundary[(edge + 1) % boundary.length];
        return a.x + (b.x - a.x) * (y - a.y) / (b.y - a.y);
    }

    /** Loft scanline strips, retaining concave outlines, canted roots and control-tab cutouts. */
    public List<Face> faces(Coordinate[] boundary) {
        List<Face> faces = new ArrayList<>();
        if (boundary.length < 3) return faces;
        var levels = new TreeSet<Double>();
        for (Coordinate p : boundary) levels.add(p.y);
        double min = levels.first(), max = levels.last();
        for (int i = 1; i < 8; i++) levels.add(min + (max - min) * i / 8);
        var ys = new ArrayList<>(levels);
        int samples = 2 * STEPS + 1;
        for (int j = 1; j < ys.size(); j++) {
            double low = ys.get(j - 1), high = ys.get(j), middle = (low + high) / 2;
            if (high - low < 1e-12) continue;
            var crossings = new ArrayList<Integer>();
            for (int k = 0; k < boundary.length; k++) {
                Coordinate a = boundary[k], b = boundary[(k + 1) % boundary.length];
                if (middle > Math.min(a.y, b.y) && middle < Math.max(a.y, b.y)) crossings.add(k);
            }
            crossings.sort(Comparator.comparingDouble(k -> edgeX(boundary, k, middle)));
            for (int k = 1; k < crossings.size(); k += 2) {
                int left = crossings.get(k - 1), right = crossings.get(k);
                Coordinate[][] top = new Coordinate[2][samples + 1];
                for (int row = 0; row < 2; row++) {
                    double y = row == 0 ? low : high;
                    double x0 = edgeX(boundary, left, y), x1 = edgeX(boundary, right, y);
                    double[] full = chord(y);
                    for (int i = 0; i <= samples; i++) {
                        double x = full[0] + (full[1] - full[0]) * fraction(i, full[1] - full[0]);
                        x = Math.max(x0, Math.min(x1, x));
                        top[row][i] = surface(x, y, 1);
                    }
                }
                for (int i = 1; i <= samples; i++) {
                    Coordinate[] positive = {top[0][i - 1], top[0][i], top[1][i], top[1][i - 1]};
                    faces.add(new Face(positive, 1));
                    Coordinate[] negative = new Coordinate[4];
                    for (int v = 0; v < 4; v++) {
                        Coordinate p = positive[3 - v];
                        negative[v] = p.setZ(-p.z);
                    }
                    faces.add(new Face(negative, -1));
                }
            }
        }
        // Close root, tip and cut walls; curved caps follow the same surface profile.
        double area = 0;
        for (int i = 0; i < boundary.length; i++) {
            Coordinate a = boundary[i], b = boundary[(i + 1) % boundary.length];
            area += a.x * b.y - b.x * a.y;
        }
        for (int i = 0; i < boundary.length; i++) {
            Coordinate a = boundary[i], b = boundary[(i + 1) % boundary.length];
            var divisions = new TreeSet<Double>();
            for (int n = 0; n <= 64; n++) divisions.add(n / 64.0);
            if (Math.abs(a.y - b.y) < 1e-12 && Math.abs(a.x - b.x) > 1e-12) {
                double[] c = chord(a.y);
                for (int n = 0; n <= samples; n++) {
                    double x = c[0] + (c[1] - c[0]) * fraction(n, c[1] - c[0]);
                    double t = (x - a.x) / (b.x - a.x);
                    if (t > 0 && t < 1) divisions.add(t);
                }
            }
            var ts = new ArrayList<>(divisions);
            for (int n = 1; n < ts.size(); n++) {
                Coordinate p = a.add(b.sub(a).multiply(ts.get(n - 1)));
                Coordinate q = a.add(b.sub(a).multiply(ts.get(n)));
                Coordinate[] face = {surface(p.x, p.y, 1), surface(p.x, p.y, -1),
                        surface(q.x, q.y, -1), surface(q.x, q.y, 1)};
                if (area < 0) Collections.reverse(Arrays.asList(face));
                faces.add(new Face(face, 0));
            }
        }
        return faces;
    }

    /** Surface contour at a chord fraction, or where a rounded edge meets the flat face. */
    public Coordinate[] contour(boolean trailing) {
        var points = new ArrayList<Coordinate>();
        double min = Arrays.stream(outline).mapToDouble(p -> p.y).min().orElse(0);
        double max = Arrays.stream(outline).mapToDouble(p -> p.y).max().orElse(0);
        for (int i = 0; i <= 48; i++) {
            double y = min + (max - min) * i / 48;
            double[] c = chord(y);
            double chord = c[1] - c[0];
            double distance = fin.getCrossSection() == FinSet.CrossSection.ROUNDED
                    ? Math.min(fin.getThickness() / 2, chord / 2) : chord * .3;
            points.add(surface(trailing ? c[1] - distance : c[0] + distance, y, 1));
        }
        return points.toArray(Coordinate[]::new);
    }
}
