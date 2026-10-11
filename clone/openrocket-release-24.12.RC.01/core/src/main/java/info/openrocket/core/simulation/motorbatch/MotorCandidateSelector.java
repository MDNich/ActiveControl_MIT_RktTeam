package info.openrocket.core.simulation.motorbatch;

import info.openrocket.core.motor.ThrustCurveMotor;
import info.openrocket.core.rocketcomponent.MotorMount;
import info.openrocket.core.document.Simulation;
import java.util.*;

/** Catalogue filters and mechanical screening are distinct, so rejected fits remain explainable. */
public final class MotorCandidateSelector {
    /** Prevents instantiation of this utility class. */
    private MotorCandidateSelector() {}

    public record Candidate(
            String id,
            ThrustCurveMotor motor,
            String exclusion,
            List<String> ignoredIntersections,
            List<String> fitWarnings) {
        /**
         * Creates a Candidate value from the supplied id, motor, exclusion, ignored intersections,
         * fit warnings.
         *
         * @param id stable identifier
         * @param motor motor
         * @param exclusion exclusion
         * @param ignoredIntersections ignored intersections
         * @param fitWarnings fit warnings
         */
        public Candidate {
            ignoredIntersections = List.copyOf(ignoredIntersections);
            fitWarnings = List.copyOf(fitWarnings);
        }

        /**
         * Creates a Candidate value from the supplied id, motor, exclusion, ignored intersections.
         *
         * @param id stable identifier
         * @param motor motor
         * @param exclusion exclusion
         * @param ignoredIntersections ignored intersections
         */
        public Candidate(
                String id,
                ThrustCurveMotor motor,
                String exclusion,
                List<String> ignoredIntersections) {
            this(id, motor, exclusion, ignoredIntersections, List.of());
        }

        /**
         * Creates a Candidate value from the supplied id, motor, exclusion.
         *
         * @param id stable identifier
         * @param motor motor
         * @param exclusion exclusion
         */
        public Candidate(String id, ThrustCurveMotor motor, String exclusion) {
            this(id, motor, exclusion, List.of(), List.of());
        }
    }

    /**
     * Filters motor curves by mount geometry, impulse, certification and the configured search
     * limits.
     *
     * @param preferredCurves preferred curves
     * @param s s
     * @param baseline baseline
     * @return the resulting List&lt;Candidate&gt;
     */
    public static List<Candidate> select(
            List<ThrustCurveMotor> preferredCurves, MotorBatchSettings s, Simulation baseline) {
        List<Candidate> result = new ArrayList<>();
        for (ThrustCurveMotor m : preferredCurves) {
            if (!s.manufacturers().contains(m.getManufacturer().getDisplayName())) continue;
            if (s.hideUnavailable() && !m.isAvailable()) continue;
            if (!s.motorType().equals("ALL") && !s.motorType().equals(m.getMotorType().name()))
                continue;
            if (!s.certificationLevel().equals("ALL")
                    && !s.certificationLevel()
                            .equals(MotorCertification.classify(m).level().name())) continue;
            String name =
                    (m.getManufacturer() + " " + m.getDesignation() + " " + m.getCommonName())
                            .toLowerCase(Locale.ROOT);
            if (Arrays.stream(s.search().toLowerCase(Locale.ROOT).split("\\s+"))
                    .anyMatch(t -> !name.contains(t))) continue;
            if (!in(m.getTotalImpulseEstimate(), s.minImpulse(), s.maxImpulse())
                    || !in(m.getAverageThrustEstimate(), s.minThrust(), s.maxThrust())
                    || !in(m.getBurnTimeEstimate(), s.minBurn(), s.maxBurn())) continue;
            // Nominal class tolerance follows the existing picker; the actual bore test below is
            // strict.
            if (m.getDiameter() <= s.nominalDiameter() - .0015
                    || m.getDiameter() >= s.nominalDiameter() + .0004) continue;
            result.add(screen("motor-" + (result.size() + 1), m, s, baseline));
        }
        return List.copyOf(result);
    }

    /**
     * Shared by preview and execution; rerun against the captured rocket and current override
     * setting.
     */
    static Candidate screen(
            String id, ThrustCurveMotor motor, MotorBatchSettings settings, Simulation baseline) {
        MotorMount mount = MotorBatchSnapshot.target(baseline, settings.mountId());
        var exclusions = new ArrayList<String>();
        var ignored = new ArrayList<String>();
        var warnings = new ArrayList<String>();
        if (!(motor.getDiameter() > 0) || !(motor.getLength() > 0))
            exclusions.add("DIMENSIONS_UNKNOWN");
        else if (motor.getDiameter() > mount.getMotorMountDiameter() + 1e-6)
            exclusions.add("DIAMETER_TOO_LARGE");
        // A motor can extend forward out of its mount; this threshold is not the rocket's usable
        // length.
        if (motor.getLength() > settings.maximumLength() + 1e-6) warnings.add("MOTOR_TOO_LONG");
        for (String conflict :
                MotorClearance.conflicts(baseline.getActiveConfiguration(), mount, motor)) {
            boolean intersection =
                    conflict.startsWith("MOTOR_INTERSECTS:")
                            || conflict.startsWith("CLEARANCE_REVIEW_ENVELOPE:");
            if (settings.ignoreComponentIntersections() && intersection) ignored.add(conflict);
            else exclusions.add(conflict);
        }
        return new Candidate(id, motor, String.join("; ", exclusions), ignored, warnings);
    }

    /**
     * Tests whether a finite candidate value falls within the optional search bounds.
     *
     * @param v v
     * @param lo lo
     * @param hi hi
     * @return true when the condition described above holds
     */
    private static boolean in(double v, double lo, double hi) {
        return Double.isFinite(v) && v >= lo && v <= hi;
    }
}
