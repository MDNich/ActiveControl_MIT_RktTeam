package info.openrocket.core.simulation.motorbatch;

import java.util.*;
import jakarta.json.JsonObject;

/**
 * Immutable publication from the worker pool; Swing and saving never inspect running simulations.
 */
public record MotorBatchReport(
        String id,
        String created,
        MotorBatchSettings settings,
        JsonObject input,
        List<Row> rows,
        int peakThreads) {
    public enum Status {
        SCREENED_OUT,
        COMPLETE,
        INCOMPLETE,
        ERROR,
        CANCELLED,
        NOT_RUN
    }

    public enum Assessment {
        MEETS_CRITERIA,
        DOES_NOT_MEET_CRITERIA,
        NEEDS_REVIEW
    }

    public record Trial(
            int number,
            double delay,
            Status status,
            Assessment assessment,
            List<String> reasons,
            Map<MotorBatchMetrics.Metric, Double> metrics,
            String warnings,
            String error,
            String telemetryPath,
            String logPath,
            double deploymentRank,
            double apogeeRank) {
        /**
         * Creates a Trial value from the supplied number, delay, status, assessment, reasons,
         * metrics, warnings, error, telemetry path, log path, deployment rank, apogee rank.
         *
         * @param number number
         * @param delay delay
         * @param status stat in microseconds
         * @param assessment assessment
         * @param reasons reasons
         * @param metrics metrics
         * @param warnings warnings
         * @param error error
         * @param telemetryPath telemetry path
         * @param logPath log path
         * @param deploymentRank deployment rank
         * @param apogeeRank apogee rank
         */
        public Trial {
            reasons = List.copyOf(reasons);
            metrics = Map.copyOf(metrics);
        }
    }

    public record Row(
            MotorCandidateSelector.Candidate candidate,
            List<Trial> trials,
            int planned,
            Trial selected) {
        /**
         * Creates a Row value from the supplied candidate, trials, planned, selected.
         *
         * @param candidate candidate
         * @param trials trials
         * @param planned planned
         * @param selected whether this item is selected
         */
        public Row {
            trials = List.copyOf(trials);
            Objects.requireNonNull(selected);
        }
    }

    /**
     * Creates a MotorBatchReport with the supplied dependencies and per-instance state.
     *
     * @param id stable identifier
     * @param created created
     * @param settings validated configuration for this operation
     * @param input input snapshot for this operation
     * @param rows rows
     * @param peakThreads peak threads
     */
    public MotorBatchReport {
        rows = List.copyOf(rows);
    }

    /**
     * Creates a report with settings and no completed trials.
     *
     * @param status stat in microseconds
     * @param reason reason
     * @return the resulting Trial
     */
    public static Trial empty(Status status, String reason) {
        boolean rejected =
                status == Status.SCREENED_OUT
                        && (reason.contains("MOTOR_INTERSECTS:")
                                || reason.contains("DIAMETER_TOO_LARGE")
                                || reason.contains("MOTOR_TOO_LONG")
                                || reason.contains("PLUGGED_WITH_MOTOR_EJECTION_RECOVERY"));
        return new Trial(
                0,
                Double.NaN,
                status,
                rejected ? Assessment.DOES_NOT_MEET_CRITERIA : Assessment.NEEDS_REVIEW,
                List.of(reason),
                Map.of(),
                "",
                "",
                "",
                "",
                Double.NaN,
                Double.NaN);
    }

    /**
     * Returns the display representation used by this value in logs or UI controls.
     *
     * @return the display representation used by this value in logs or UI controls
     */
    @Override
    public String toString() {
        return created + " — " + rows.size() + " motors (" + settings.threads() + " threads)";
    }
}
