package info.openrocket.core.aerodynamics.barrowman;

/**
 * Experimental small-incidence, supersonic pressure integral for an exposed
 * trapezoidal panel. x is aft from the root leading edge, y is outboard.
 * The body acts as an image plane at the root; FinSetCalc adds its usual
 * body/fin interference factor once after this isolated-fin calculation.
 */
public final class FiniteWingPressure {
    private static final double MIN_MACH = 1.1;
    private static final double MAX_MACH = 5;
    private static final int INTERVALS = 156;
    private static final int STRIPS = 48;
    private static final int CHORD_POINTS = 32;

    /** Area times CN slope (m²/rad), and its first moment (m³/rad). */
    public record Load(double force, double moment) { }

    private final Load[] table = new Load[INTERVALS + 1];
    private final double edgeCosine;
    private final double aftLengthOverSpan;

    public FiniteWingPressure(double root, double tip, double span, double sweep) {
        if (!(root > 0 && tip >= 0 && span > 0 && Double.isFinite(root + tip + span + sweep))) {
            throw new IllegalArgumentException("Invalid trapezoidal fin dimensions");
        }
        edgeCosine = span / Math.hypot(span, sweep);
        aftLengthOverSpan = Math.max(tip, root - sweep) / span;
        for (int i = 0; i <= INTERVALS; i++) {
            double mach = MIN_MACH + (MAX_MACH - MIN_MACH) * i / INTERVALS;
            double beta = Math.sqrt(mach * mach - 1);
            double areaSlope = 0, firstMoment = 0;
            for (int j = 0; j < STRIPS; j++) {
                double fraction = (j + 0.5) / STRIPS;
                double chord = root + fraction * (tip - root);
                double area = chord * span / (STRIPS * CHORD_POINTS);
                double tipDistance = span * (1 - fraction);
                for (int k = 0; k < CHORD_POINTS; k++) {
                    double x = sweep * fraction + chord * (k + 0.5) / CHORD_POINTS;
                    // Rectangular-wing linear tip-cone solution, applied as a
                    // local approximation downstream of the swept tip corner.
                    double downstream = x - sweep;
                    double relief = downstream <= beta * tipDistance ? 1 :
                            2 / Math.PI * Math.asin(Math.sqrt(beta * tipDistance / downstream));
                    // Two-face Ackeret pressure jump per radian of incidence.
                    double load = 4 / beta * area * relief;
                    areaSlope += load;
                    firstMoment += load * x;
                }
            }
            table[i] = new Load(areaSlope, firstMoment);
        }
    }

    public Load at(double mach) {
        double index = Math.max(0, Math.min(INTERVALS, (mach - MIN_MACH) * INTERVALS / (MAX_MACH - MIN_MACH)));
        int i = Math.min((int)index, INTERVALS - 1);
        double t = index - i;
        Load a = table[i], b = table[i + 1];
        return new Load(a.force + t * (b.force - a.force), a.moment + t * (b.moment - a.moment));
    }

    /** Smooth handover only inside the small-angle, supersonic-leading-edge envelope. */
    public double weight(double mach, double incidence) {
        if (!(mach > MIN_MACH && mach < MAX_MACH)) return 0;
        // The single-tip solution requires its Mach cone to clear the image
        // plane at the aftmost point of the panel (beta * span >= aft length).
        double coneClearance = Math.sqrt(mach * mach - 1) - aftLengthOverSpan;
        return smooth((mach - MIN_MACH) / 0.4) * smooth((mach * edgeCosine - 1) / 0.15)
                * smooth(coneClearance / 0.15)
                * smooth((MAX_MACH - mach) / 0.35)
                * smooth((Math.toRadians(8) - Math.abs(incidence)) / Math.toRadians(5));
    }

    private static double smooth(double value) {
        double t = Math.max(0, Math.min(1, value));
        return t * t * (3 - 2 * t);
    }
}
