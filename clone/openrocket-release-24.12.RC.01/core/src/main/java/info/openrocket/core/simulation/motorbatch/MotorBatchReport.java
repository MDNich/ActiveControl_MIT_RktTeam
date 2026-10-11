package info.openrocket.core.simulation.motorbatch;

import java.util.*;
import jakarta.json.JsonObject;

/** Immutable publication from the worker pool; Swing and saving never inspect running simulations. */
public record MotorBatchReport(String id,String created,MotorBatchSettings settings,JsonObject input,
                               List<Row> rows,int peakThreads) {
    public enum Status { SCREENED_OUT, COMPLETE, INCOMPLETE, ERROR, CANCELLED, NOT_RUN }
    public enum Assessment { MEETS_CRITERIA, DOES_NOT_MEET_CRITERIA, NEEDS_REVIEW }
    public record Trial(int number,double delay,Status status,Assessment assessment,List<String> reasons,
                        Map<MotorBatchMetrics.Metric,Double> metrics,String warnings,String error,
                        String telemetryPath,String logPath,double deploymentRank,double apogeeRank) {
        public Trial { reasons=List.copyOf(reasons);metrics=Map.copyOf(metrics); }
    }
    public record Row(MotorCandidateSelector.Candidate candidate,List<Trial> trials,int planned,Trial selected) {
        public Row { trials=List.copyOf(trials);Objects.requireNonNull(selected); }
    }
    public MotorBatchReport { rows=List.copyOf(rows); }
    public static Trial empty(Status status,String reason) {
        boolean rejected=status==Status.SCREENED_OUT && (reason.contains("MOTOR_INTERSECTS:") || reason.contains("DIAMETER_TOO_LARGE")
                || reason.contains("MOTOR_TOO_LONG") || reason.contains("PLUGGED_WITH_MOTOR_EJECTION_RECOVERY"));
        return new Trial(0,Double.NaN,status,rejected?Assessment.DOES_NOT_MEET_CRITERIA:Assessment.NEEDS_REVIEW,List.of(reason),Map.of(),"","","","",Double.NaN,Double.NaN);
    }
    @Override public String toString() { return created+" — "+rows.size()+" motors ("+settings.threads()+" threads)"; }
}
