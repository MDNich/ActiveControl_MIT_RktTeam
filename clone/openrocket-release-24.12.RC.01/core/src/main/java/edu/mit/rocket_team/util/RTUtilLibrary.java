package edu.mit.rocket_team.util;

import info.openrocket.core.util.Coordinate;

/** Unsigned firmware-clock arithmetic, run-scoped action tracing and legacy orientation helpers. */
public class RTUtilLibrary {

    /**
     * Keeps the low 32 bits as a nonnegative long, reproducing an unsigned Arduino clock value.
     *
     * @param value replacement or input value
     * @return computed integer value
     */
    public static long unsigned32(long value) {
        return value & 0xffff_ffffL;
    }

    /**
     * Computes wrapping unsigned 32-bit elapsed time using the same units as the supplied clocks.
     *
     * @param now now
     * @param then then
     * @return computed integer value
     */
    public static long elapsed32(long now, long then) {
        return unsigned32(now - then);
    }

    /** One run's clock and action log. Production output always uses System.out.println. */
    public static final class Trace {
        private static final java.util.concurrent.atomic.AtomicLong IDS =
                new java.util.concurrent.atomic.AtomicLong();
        private final long run = IDS.incrementAndGet();
        private final java.util.function.Consumer<String> sink;
        private long nowUs;

        /** Creates a Trace value from the supplied . */
        public Trace() {
            this(line -> System.out.println(line));
        }

        /**
         * Creates a Trace value from the supplied sink.
         *
         * @param sink destination for formatted log lines
         */
        public Trace(java.util.function.Consumer<String> sink) {
            this.sink = java.util.Objects.requireNonNull(sink);
        }

        /**
         * Advances the monotonic virtual boot clock in microseconds; moving backwards is rejected.
         *
         * @param timeUs time in microseconds
         * @throws IllegalArgumentException when rejected: FC clock cannot run backwards
         */
        public void time(long timeUs) {
            if (timeUs < nowUs) throw new IllegalArgumentException("FC clock cannot run backwards");
            nowUs = timeUs;
        }

        /**
         * Returns the low 32 bits of virtual microseconds since boot.
         *
         * @return the low 32 bits of virtual microseconds since boot
         */
        public long micros() {
            return unsigned32(nowUs);
        }

        /**
         * Returns virtual milliseconds since boot modulo 2³².
         *
         * @return virtual milliseconds since boot modulo 2³²
         */
        public long millis() {
            return unsigned32(nowUs / 1000);
        }

        /**
         * Returns the unwrapped virtual boot time in microseconds.
         *
         * @return the unwrapped virtual boot time in microseconds
         */
        public long bootUs() {
            return nowUs;
        }

        /**
         * Writes a run-tagged action line containing the current boot time to the configured sink.
         *
         * @param action action identifier or state-entry action
         * @param detail action-specific log fields
         */
        public void log(String action, String detail) {
            sink.accept(
                    "ZEPHYRUS run="
                            + run
                            + " boot_us="
                            + nowUs
                            + " action="
                            + action
                            + " "
                            + detail);
        }
    }

    /**
     * Converts rocket roll &amp; pitch into IMU-measured X, Y, Z angles.
     *
     * @param thisRollAngle roll angle in radians (rotation about Z)
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
        double xAngle = Math.atan2(r32, r33); // rotation about X
        double yAngle = Math.asin(-r31); // rotation about Y (≈ 0)
        double zAngle = Math.atan2(r21, r11); // rotation about Z

        return new Coordinate(xAngle, yAngle, zAngle);
    }
}
