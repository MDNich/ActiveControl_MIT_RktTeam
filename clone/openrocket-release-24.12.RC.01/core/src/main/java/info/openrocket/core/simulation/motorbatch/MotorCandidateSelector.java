package info.openrocket.core.simulation.motorbatch;

import info.openrocket.core.motor.ThrustCurveMotor;
import info.openrocket.core.rocketcomponent.MotorMount;
import info.openrocket.core.document.Simulation;
import java.util.*;

/** Catalogue filters and mechanical screening are distinct, so rejected fits remain explainable. */
public final class MotorCandidateSelector {
    private MotorCandidateSelector() { }
    public record Candidate(String id, ThrustCurveMotor motor, String exclusion, List<String> ignoredIntersections) {
        public Candidate { ignoredIntersections = List.copyOf(ignoredIntersections); }
        public Candidate(String id, ThrustCurveMotor motor, String exclusion) { this(id, motor, exclusion, List.of()); }
    }
    public static List<Candidate> select(List<ThrustCurveMotor> preferredCurves, MotorBatchSettings s, Simulation baseline) {
        List<Candidate> result = new ArrayList<>();
        for (ThrustCurveMotor m : preferredCurves) {
            if (!s.manufacturers().contains(m.getManufacturer().getDisplayName())) continue;
            if (s.hideUnavailable() && !m.isAvailable()) continue;
            if (!s.motorType().equals("ALL") && !s.motorType().equals(m.getMotorType().name())) continue;
            if(!s.certificationLevel().equals("ALL") && !s.certificationLevel().equals(MotorCertification.classify(m).level().name()))continue;
            String name = (m.getManufacturer()+" "+m.getDesignation()+" "+m.getCommonName()).toLowerCase(Locale.ROOT);
            if (Arrays.stream(s.search().toLowerCase(Locale.ROOT).split("\\s+")).anyMatch(t -> !name.contains(t))) continue;
            if (!in(m.getTotalImpulseEstimate(),s.minImpulse(),s.maxImpulse()) ||
                !in(m.getAverageThrustEstimate(),s.minThrust(),s.maxThrust()) ||
                !in(m.getBurnTimeEstimate(),s.minBurn(),s.maxBurn())) continue;
            // Nominal class tolerance follows the existing picker; the actual bore test below is strict.
            if (m.getDiameter() <= s.nominalDiameter()-.0015 || m.getDiameter() >= s.nominalDiameter()+.0004) continue;
            result.add(screen("motor-"+(result.size()+1),m,s,baseline));
        }
        return List.copyOf(result);
    }
    /** Shared by preview and execution; rerun against the captured rocket and current override setting. */
    static Candidate screen(String id, ThrustCurveMotor motor, MotorBatchSettings settings, Simulation baseline) {
        MotorMount mount=MotorBatchSnapshot.target(baseline,settings.mountId());
        var exclusions=new ArrayList<String>();
        var ignored=new ArrayList<String>();
        if (!(motor.getDiameter()>0) || !(motor.getLength()>0)) exclusions.add("DIMENSIONS_UNKNOWN");
        else if (motor.getDiameter()>mount.getMotorMountDiameter()+1e-6) exclusions.add("DIAMETER_TOO_LARGE");
        else if (motor.getLength()>settings.maximumLength()+1e-6) exclusions.add("MOTOR_TOO_LONG");
        for (String conflict:MotorClearance.conflicts(baseline.getActiveConfiguration(),mount,motor)) {
            boolean intersection=conflict.startsWith("MOTOR_INTERSECTS:") || conflict.startsWith("CLEARANCE_REVIEW_ENVELOPE:");
            if (settings.ignoreComponentIntersections() && intersection) ignored.add(conflict);
            else exclusions.add(conflict);
        }
        return new Candidate(id,motor,String.join("; ",exclusions),ignored);
    }
    private static boolean in(double v,double lo,double hi) { return Double.isFinite(v)&&v>=lo&&v<=hi; }
}
