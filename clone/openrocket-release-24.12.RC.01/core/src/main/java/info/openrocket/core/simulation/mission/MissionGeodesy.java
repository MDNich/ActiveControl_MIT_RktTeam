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

    /** Inverse ECEF, including poles; height is WGS84 ellipsoidal metres. */
    public static double[] geodetic(double[] point) {
        double x = point[0], y = point[1], z = point[2], p = Math.hypot(x, y), e2 = F * (2 - F);
        if (p < 1e-9) return new double[] {Math.copySign(90, z), 0, Math.abs(z) - A * (1 - F)};
        double lat = Math.atan2(z, p * (1 - e2));
        for (int i = 0; i < 12; i++) {
            double n = A / Math.sqrt(1 - e2 * Math.sin(lat) * Math.sin(lat));
            double next = Math.atan2(z + e2 * n * Math.sin(lat), p);
            if (Math.abs(next - lat) < 1e-14) { lat = next; break; }
            lat = next;
        }
        double n = A / Math.sqrt(1 - e2 * Math.sin(lat) * Math.sin(lat));
        return new double[] {Math.toDegrees(lat), Math.toDegrees(Math.atan2(y, x)), p / Math.cos(lat) - n};
    }
    public static double[] enu(double[] target, double latitude, double longitude, double height) {
        double[] origin = ecef(latitude, longitude, height);
        double x = target[0] - origin[0], y = target[1] - origin[1], z = target[2] - origin[2];
        double lat = Math.toRadians(latitude), lon = Math.toRadians(longitude);
        return new double[] {-Math.sin(lon) * x + Math.cos(lon) * y,
                -Math.sin(lat) * Math.cos(lon) * x - Math.sin(lat) * Math.sin(lon) * y + Math.cos(lat) * z,
                Math.cos(lat) * Math.cos(lon) * x + Math.cos(lat) * Math.sin(lon) * y + Math.sin(lat) * z};
    }
    public static double[] pointing(double[] target, double latitude, double longitude, double height) {
        double[] local = enu(target, latitude, longitude, height); double horizontal = Math.hypot(local[0], local[1]);
        if (Math.hypot(horizontal, local[2]) < 10 || horizontal < 1) return null;
        return new double[] {(Math.toDegrees(Math.atan2(local[0], local[1])) + 360) % 360, Math.toDegrees(Math.atan2(local[2], horizontal))};
    }
}
