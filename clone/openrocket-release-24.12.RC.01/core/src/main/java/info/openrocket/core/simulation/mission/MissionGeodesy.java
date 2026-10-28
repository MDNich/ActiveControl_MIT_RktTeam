package info.openrocket.core.simulation.mission;

/** WGS84 geometry primitives. Height conversion requires an explicit geoid separation. */
public final class MissionGeodesy {
    private static final double A = 6378137.0;
    private static final double F = 1 / 298.257223563;
    private MissionGeodesy() { }

    public static double ellipsoidalHeight(double msl, double separation) {
        if (!Double.isFinite(msl) || !Double.isFinite(separation) || !Double.isFinite(msl + separation))
            throw new IllegalArgumentException("Finite height and explicit geoid separation required");
        return msl + separation;
    }

    public static double[] ecef(double latitudeDeg, double longitudeDeg, double ellipsoidalHeightM) {
        if (!Double.isFinite(latitudeDeg) || Math.abs(latitudeDeg) > 90
                || !Double.isFinite(longitudeDeg) || Math.abs(longitudeDeg) > 180
                || !Double.isFinite(ellipsoidalHeightM)) throw new IllegalArgumentException("Invalid WGS84 location");
        double lat = Math.toRadians(latitudeDeg), lon = Math.toRadians(longitudeDeg);
        double e2 = F * (2 - F), sin = Math.sin(lat), cos = Math.cos(lat);
        double n = A / Math.sqrt(1 - e2 * sin * sin);
        return new double[] {(n + ellipsoidalHeightM) * cos * Math.cos(lon),
                (n + ellipsoidalHeightM) * cos * Math.sin(lon),
                (n * (1 - e2) + ellipsoidalHeightM) * sin};
    }
}
