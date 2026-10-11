package info.openrocket.core.simulation.motorbatch;

import info.openrocket.core.document.Simulation;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.*;
import info.openrocket.core.motor.IgnitionEvent;
import info.openrocket.core.util.MathUtil;
import java.util.*;
import static info.openrocket.core.simulation.FlightDataType.*;

public final class MotorBatchMetrics {
    /** Prevents instantiation of this utility class. */
    private MotorBatchMetrics() {}

    public enum Metric {
        APOGEE("Apogee AGL (m)", "apogee_agl_m"),
        RAIL_SPEED("Rail exit (m/s)", "rail_exit_speed_m_s"),
        MAX_SPEED("Max speed before recovery (m/s)", "max_speed_before_recovery_m_s"),
        MAX_MACH("Max Mach before recovery", "max_mach_before_recovery"),
        MAX_ACCELERATION(
                "Max acceleration before recovery (m/s²)", "max_acceleration_before_recovery_m_s2"),
        MIN_STABILITY("Min ascent stability (cal)", "min_ascent_stability_cal"),
        DEPLOY_SPEED("Max deployment speed (m/s)", "max_deployment_speed_m_s"),
        LANDING_VERTICAL("Landing descent speed (m/s)", "landing_descent_speed_m_s"),
        LANDING_SPEED("Ground contact speed (m/s)", "ground_contact_speed_m_s"),
        LAUNCH_MASS("Launch mass (kg)", "launch_mass_kg"),
        THRUST_WEIGHT("Average thrust / launch weight", "average_thrust_launch_weight"),
        APOGEE_TIME("Time to apogee (s)", "time_to_apogee_s"),
        DURATION("Flight duration (s)", "flight_duration_s"),
        LANDING_DISTANCE("Landing distance (m)", "landing_distance_m");
        public final String label, column;

        /**
         * Creates a Metric value from the supplied label, column.
         *
         * @param label initial label
         * @param column zero-based column index
         */
        Metric(String label, String column) {
            this.label = label;
            this.column = column;
        }

        /**
         * Returns the display representation used by this value in logs or UI controls.
         *
         * @return the display representation used by this value in logs or UI controls
         */
        @Override
        public String toString() {
            return label;
        }
    }

    public record Evaluation(
            Map<Metric, Double> values,
            boolean complete,
            List<String> reasons,
            double delayDeploymentSpeed,
            double delayApogeeDistance) {}

    /**
     * Evaluates flight criteria and ascent-only extrema for one motor trial.
     *
     * @param simulation simulation being configured or executed
     * @param settings validated configuration for this operation
     * @return the resulting Evaluation
     */
    public static Evaluation evaluate(Simulation simulation, MotorBatchSettings settings) {
        var values = new EnumMap<Metric, Double>(Metric.class);
        List<String> reasons = new ArrayList<>();
        FlightData data = simulation.getSimulatedData();
        if (data == null || data.getBranchCount() == 0)
            return new Evaluation(values, false, List.of("NO_FLIGHT_DATA"), Double.NaN, Double.NaN);
        FlightDataBranch b = data.getBranch(0); // The batch validates exactly one active stage.
        double rail = eventTime(b, FlightEvent.Type.LAUNCHROD),
                apogee = eventTime(b, FlightEvent.Type.APOGEE);
        double recovery = eventTime(b, FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT),
                ground = eventTime(b, FlightEvent.Type.GROUND_HIT);
        double last = b.getLast(TYPE_TIME), end = Double.isFinite(recovery) ? recovery : last;
        values.put(
                Metric.APOGEE,
                b.getMaximum(TYPE_ALTITUDE)); // Engine TYPE_ALTITUDE is launch-relative.
        values.put(Metric.RAIL_SPEED, at(b, TYPE_VELOCITY_TOTAL, rail));
        values.put(Metric.MAX_SPEED, extreme(b, TYPE_VELOCITY_TOTAL, 0, end, false));
        values.put(Metric.MAX_MACH, extreme(b, TYPE_MACH_NUMBER, 0, end, false));
        values.put(Metric.MAX_ACCELERATION, extreme(b, TYPE_ACCELERATION_TOTAL, 0, end, false));
        values.put(
                Metric.MIN_STABILITY,
                extreme(
                        b,
                        TYPE_STABILITY,
                        rail,
                        Double.isFinite(apogee) ? Math.min(apogee, end) : Double.NaN,
                        true));
        double deploySpeed = Double.NaN, rankedSpeed = Double.NaN, rankedDistance = Double.NaN;
        var deployed = new HashSet<UUID>();
        for (var e : b.getEvents())
            if (e.getType() == FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT) {
                double v = at(b, TYPE_VELOCITY_TOTAL, e.getTime());
                deploySpeed = Double.isNaN(deploySpeed) ? v : Math.max(deploySpeed, v);
                if (e.getSource() != null) deployed.add(e.getSource().getID());
                if (e.getSource() instanceof RecoveryDevice device
                        && device.getDeploymentConfigurations()
                                        .get(simulation.getFlightConfigurationId())
                                        .getDeployEvent()
                                == DeploymentConfiguration.DeployEvent.EJECTION
                        && Double.isNaN(rankedSpeed)) {
                    rankedSpeed = v;
                    rankedDistance = Math.abs(e.getTime() - apogee);
                }
            }
        values.put(Metric.DEPLOY_SPEED, deploySpeed);
        values.put(Metric.LANDING_VERTICAL, Math.max(0, -at(b, TYPE_VELOCITY_Z, ground)));
        values.put(Metric.LANDING_SPEED, at(b, TYPE_VELOCITY_TOTAL, ground));
        double mass = at(b, TYPE_MASS, 0), gravity = at(b, TYPE_GRAVITY, 0), thrust = 0;
        for (var m : simulation.getActiveConfiguration().getAllMotors()) {
            if (m.getMotor() != null
                    && (m.getIgnitionEvent() == IgnitionEvent.LAUNCH
                            || m.getIgnitionEvent() == IgnitionEvent.AUTOMATIC)
                    && m.getIgnitionDelay() == 0)
                thrust +=
                        m.getMotor().getAverageThrustEstimate()
                                * m.getMount().getMotorCountIncludingAssemblyCopies();
        }
        values.put(Metric.LAUNCH_MASS, mass);
        values.put(Metric.THRUST_WEIGHT, thrust / (mass * gravity));
        values.put(Metric.APOGEE_TIME, apogee);
        values.put(Metric.DURATION, last);
        values.put(
                Metric.LANDING_DISTANCE,
                Math.hypot(at(b, TYPE_POSITION_X, ground), at(b, TYPE_POSITION_Y, ground)));
        if (!Double.isFinite(rail)) reasons.add("NO_RAIL_EXIT");
        if (!Double.isFinite(apogee)) reasons.add("NO_APOGEE");
        if (!Double.isFinite(ground)) reasons.add("NO_GROUND_CONTACT_OR_TIME_LIMIT");
        if (b.getLastEvent(FlightEvent.Type.SIM_ABORT) != null) reasons.add("SIMULATION_ABORT");
        for (var c : simulation.getActiveConfiguration().getActiveComponents())
            if (c instanceof RecoveryDevice r
                    && r.getDeploymentConfigurations()
                                    .get(simulation.getFlightConfigurationId())
                                    .getDeployEvent()
                            != DeploymentConfiguration.DeployEvent.NEVER
                    && !deployed.contains(r.getID()))
                reasons.add("RECOVERY_MISSING:" + r.getName());
        boolean complete = reasons.isEmpty();
        settings.criteria()
                .forEach(
                        (metric, bounds) -> {
                            double v = values.getOrDefault(metric, Double.NaN);
                            if (!Double.isFinite(v)) reasons.add(metric.name() + "_MISSING");
                            else {
                                if (bounds.minimum() != null && v < bounds.minimum())
                                    reasons.add(metric.name() + "_LOW");
                                if (bounds.maximum() != null && v > bounds.maximum())
                                    reasons.add(metric.name() + "_HIGH");
                            }
                        });
        return new Evaluation(
                Collections.unmodifiableMap(values),
                complete,
                List.copyOf(reasons),
                rankedSpeed,
                rankedDistance);
    }

    /**
     * Finds the time of a relevant flight event for metric boundaries.
     *
     * @param b b
     * @param type type
     * @return computed numeric value
     */
    public static double eventTime(FlightDataBranch b, FlightEvent.Type type) {
        return b.getEvents().stream()
                .filter(e -> e.getType() == type)
                .mapToDouble(FlightEvent::getTime)
                .findFirst()
                .orElse(Double.NaN);
    }

    /**
     * Interpolates a recorded quantity at a physical-time boundary.
     *
     * @param b times value
     * @param type type
     * @param t t
     * @return computed numeric value
     */
    public static double at(FlightDataBranch b, FlightDataType type, double t) {
        var times = b.get(TYPE_TIME);
        var values = b.get(type);
        if (!Double.isFinite(t)
                || times == null
                || values == null
                || times.isEmpty()
                || times.size() != values.size()) return Double.NaN;
        return MathUtil.interpolate(times, values, t);
    }

    /**
     * Finds a finite minimum or maximum within the requested flight interval.
     *
     * @param b time value
     * @param type type
     * @param start start
     * @param end end
     * @param min lower allowed bound
     * @return computed numeric value
     */
    static double extreme(
            FlightDataBranch b, FlightDataType type, double start, double end, boolean min) {
        if (!Double.isFinite(start) || !Double.isFinite(end) || end < start) return Double.NaN;
        var time = b.get(TYPE_TIME);
        var values = b.get(type);
        if (time == null || values == null) return Double.NaN;
        double result = min ? Double.POSITIVE_INFINITY : Double.NEGATIVE_INFINITY;
        // Include interpolated interval boundaries as well as recorded samples.
        double[] boundaries = {at(b, type, start), at(b, type, end)};
        for (double v : boundaries)
            if (Double.isFinite(v)) result = min ? Math.min(result, v) : Math.max(result, v);
        for (int i = 0; i < time.size(); i++)
            if (time.get(i) >= start && time.get(i) <= end && Double.isFinite(values.get(i)))
                result = min ? Math.min(result, values.get(i)) : Math.max(result, values.get(i));
        return Double.isFinite(result) ? result : Double.NaN;
    }
}
