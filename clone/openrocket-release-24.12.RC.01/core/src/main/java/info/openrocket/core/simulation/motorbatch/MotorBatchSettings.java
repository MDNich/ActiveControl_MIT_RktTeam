package info.openrocket.core.simulation.motorbatch;

import java.util.*;

/** Immutable, SI-valued inputs captured before dispatching any worker. */
public record MotorBatchSettings(UUID mountId, int threads, double nominalDiameter, double maximumLength,
        Set<String> manufacturers, String search, String motorType, String certificationLevel, boolean hideUnavailable,
        double minImpulse, double maxImpulse, double minThrust, double maxThrust,
        double minBurn, double maxBurn, DelayPolicy delayPolicy, Map<MotorBatchMetrics.Metric, Bounds> criteria,
        boolean ignoreComponentIntersections) {
    public static final int DEFAULT_THREADS = 4;
    public enum DelayPolicy { KEEP_BASELINE, BEST_LISTED }
    /** Existing callers keep strict clearance screening unless they explicitly enable the override. */
    public MotorBatchSettings(UUID mountId, int threads, double nominalDiameter, double maximumLength,
            Set<String> manufacturers, String search, String motorType, String certificationLevel, boolean hideUnavailable,
            double minImpulse, double maxImpulse, double minThrust, double maxThrust,
            double minBurn, double maxBurn, DelayPolicy delayPolicy, Map<MotorBatchMetrics.Metric, Bounds> criteria) {
        this(mountId, threads, nominalDiameter, maximumLength, manufacturers, search, motorType, certificationLevel,
                hideUnavailable, minImpulse, maxImpulse, minThrust, maxThrust, minBurn, maxBurn, delayPolicy, criteria, false);
    }
    public MotorBatchSettings withIgnoreComponentIntersections(boolean ignore) {
        return new MotorBatchSettings(mountId, threads, nominalDiameter, maximumLength, manufacturers, search, motorType,
                certificationLevel, hideUnavailable, minImpulse, maxImpulse, minThrust, maxThrust, minBurn, maxBurn,
                delayPolicy, criteria, ignore);
    }
    public record Bounds(Double minimum, Double maximum) {
        public Bounds {
            if (minimum != null && !Double.isFinite(minimum) || maximum != null && !Double.isFinite(maximum)
                    || minimum != null && maximum != null && minimum > maximum)
                throw new IllegalArgumentException("Invalid criterion range");
        }
    }
    public MotorBatchSettings {
        Objects.requireNonNull(mountId); Objects.requireNonNull(delayPolicy);
        if (threads < 1 || threads > 256) throw new IllegalArgumentException("Thread count must be between 1 and 256");
        if (!Double.isFinite(nominalDiameter) || nominalDiameter <= 0 || !Double.isFinite(maximumLength) || maximumLength <= 0)
            throw new IllegalArgumentException("Mount diameter and available length must be positive");
        double[] ranges = {minImpulse,maxImpulse,minThrust,maxThrust,minBurn,maxBurn};
        for (double x : ranges) if (!Double.isFinite(x) || x < 0) throw new IllegalArgumentException("Filter ranges must be finite and nonnegative");
        if(minImpulse>maxImpulse || minThrust>maxThrust || minBurn>maxBurn) throw new IllegalArgumentException("Filter minimum exceeds maximum");
        manufacturers = Set.copyOf(manufacturers); criteria = Map.copyOf(criteria);
        search = Objects.requireNonNull(search).strip(); motorType = Objects.requireNonNull(motorType);
        Objects.requireNonNull(certificationLevel);
        if(!certificationLevel.equals("ALL"))MotorCertification.Level.valueOf(certificationLevel);
    }
    public static MotorBatchSettings defaults(UUID mount, double diameter, double length) {
        return new MotorBatchSettings(mount, DEFAULT_THREADS, diameter, length, Set.of(), "", "ALL", "ALL", false,
                0, 1e9, 0, 1e9, 0, 1e6, DelayPolicy.KEEP_BASELINE,
                Map.of(MotorBatchMetrics.Metric.RAIL_SPEED, new Bounds(15.0,null),
                       MotorBatchMetrics.Metric.MIN_STABILITY, new Bounds(1.0,null)));
    }
}
