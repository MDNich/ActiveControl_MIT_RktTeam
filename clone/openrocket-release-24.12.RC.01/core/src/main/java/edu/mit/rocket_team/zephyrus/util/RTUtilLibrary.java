package edu.mit.rocket_team.zephyrus.util;

import info.openrocket.core.util.Coordinate;

public class RTUtilLibrary {

    public static long unsigned32(long value) { return value & 0xffff_ffffL; }
    public static long elapsed32(long now, long then) { return unsigned32(now - then); }

    /** One run's clock and action log. Production output always uses System.out.println. */
    public static final class Trace {
        private static final java.util.concurrent.atomic.AtomicLong IDS = new java.util.concurrent.atomic.AtomicLong();
        private final long run = IDS.incrementAndGet();
        private final java.util.function.Consumer<String> sink;
        private long nowUs;
        public Trace() { this(line -> System.out.println(line)); }
        public Trace(java.util.function.Consumer<String> sink) { this.sink = java.util.Objects.requireNonNull(sink); }
        public void time(long timeUs) {
            if (timeUs < nowUs) throw new IllegalArgumentException("FC clock cannot run backwards");
            nowUs = timeUs;
        }
        public long micros() { return unsigned32(nowUs); }
        public long millis() { return unsigned32(nowUs / 1000); }
        public long bootUs() { return nowUs; }
        public void log(String action, String detail) {
            sink.accept("ZEPHYRUS run=" + run + " boot_us=" + nowUs + " action=" + action + " " + detail);
        }
    }

    /**
     * Converts rocket roll & pitch into IMU-measured X, Y, Z angles.
     *
     * @param thisRollAngle  roll angle in radians (rotation about Z)
     * @param thisPitchAngle pitch angle in radians (rotation about X)
     * @return Coordinate(xAngle, yAngle, zAngle) in radians
     */
    public static Coordinate convertToImuAngles(double thisRollAngle, double thisPitchAngle) {

        double cr = Math.cos(thisRollAngle);
        double sr = Math.sin(thisRollAngle);
        double cp = Math.cos(thisPitchAngle);
        double sp = Math.sin(thisPitchAngle);

        // Rotation matrix R = Rz(roll) * Rx(pitch)
        double r11 = cr;
        double r12 = -sr * cp;
        double r13 = sr * sp;

        double r21 = sr;
        double r22 = cr * cp;
        double r23 = -cr * sp;

        double r31 = 0.0;
        double r32 = sp;
        double r33 = cp;

        // Extract IMU Euler angles (X-Y-Z order)
        double xAngle = Math.atan2(r32, r33);   // rotation about X
        double yAngle = Math.asin(-r31);        // rotation about Y (≈ 0)
        double zAngle = Math.atan2(r21, r11);   // rotation about Z

        return new Coordinate(xAngle, yAngle, zAngle);
    }

}
