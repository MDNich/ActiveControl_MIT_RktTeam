package info.openrocket.core.simulation.flightcomputer;

import edu.mit.rocket_team.FC.RTFC;
import edu.mit.rocket_team.control.RTRollController;
import edu.mit.rocket_team.control.airbrakes.RTAirbrakesController;
import edu.mit.rocket_team.util.RTUtilLibrary.Trace;
import edu.mit.rocket_team.util.data.RTFudgedAirbrakesData;
import java.util.*;
import java.util.function.DoubleUnaryOperator;

/** Controller histories and scheduled work belong to one flight only. */
public final class FlightComputerControllerRuntime {
    public record Values(
            double measurement,
            double baseline,
            double error,
            double p,
            double i,
            double d,
            double weight,
            double requested,
            double output,
            boolean active,
            boolean valid,
            boolean saturated,
            long sampleUs,
            long startUs,
            long completionUs) {}

    private record Input(
            FlightComputerSignals.Sample measurement,
            FlightComputerSignals.Sample derivative,
            FlightComputerSignals.Sample velocity,
            double altitude,
            double flightSeconds,
            double acceleration,
            double rollRate,
            long calibrationUs) {}

    private static final class Block {
        final FlightComputerController c;
        final RTRollController roll;
        final RTAirbrakesController apogee;
        long nextStart,
                lastStart = -1,
                finish = -1,
                lastControl = -1,
                lastSample = -1,
                lastDerivativeSample = -1,
                epoch,
                workEpoch,
                lastValid = -1;
        long averageStart = -1, averageLast = -1;
        int averageCount;
        double averageSum, integral, lastMeasurement, derivative, baseline = Double.NaN;
        boolean active, speedActive, closed;
        String state = "";
        Input input;
        Values values;

        /**
         * Creates a Block value from the supplied c, trace.
         *
         * @param c selected channel or controller
         * @param trace run-local virtual clock and action-log sink
         */
        Block(FlightComputerController c, Trace trace) {
            this.c = c;
            roll =
                    c.type() == FlightComputerController.Type.LEGACY_ROLL
                            ? new RTRollController(trace)
                            : null;
            apogee =
                    c.type() == FlightComputerController.Type.LEGACY_APOGEE
                            ? new RTAirbrakesController(trace)
                            : null;
            if (roll != null) {
                roll.begin();
                roll.setPidGains((float) c.n("kp"), (float) c.n("ki"), (float) c.n("kd"));
            }
            if (apogee != null)
                apogee.setPidGains((float) c.n("kp"), (float) c.n("ki"), (float) c.n("kd"));
        }

        /** Resets all controller tasks and their per-run PID, capture and output state. */
        void reset() {
            integral = derivative = 0;
            lastControl = lastSample = lastDerivativeSample = -1;
            speedActive = false;
            if (roll != null) roll.resetPidHistory();
            if (apogee != null) apogee.resetPidHistory();
        }
    }

    private final FlightComputerDesign design;
    private final List<Block> blocks;
    private final Trace trace;
    private final Map<String, Map<String, info.openrocket.core.simulation.FlightDataType>>
            plotChannels = new HashMap<>();

    /**
     * Returns the cached diagnostic channels for the configured controller blocks.
     *
     * @param id stable identifier
     * @return the cached diagnostic channels for the configured controller blocks
     */
    public Map<String, info.openrocket.core.simulation.FlightDataType> plotChannels(String id) {
        return plotChannels.get(id);
    }

    private final Map<String, Double> outputs = new LinkedHashMap<>();
    private final Map<String, DoubleUnaryOperator> targetSaturation = new HashMap<>();

    /**
     * Creates a FlightComputerControllerRuntime with the supplied dependencies and per-instance
     * state.
     *
     * @param design external flight-computer design snapshot
     * @param trace run-local virtual clock and action-log sink
     */
    public FlightComputerControllerRuntime(FlightComputerDesign design, Trace trace) {
        this.design = design;
        this.trace = trace;
        blocks =
                design.controllers().stream()
                        .filter(FlightComputerController::enabled)
                        .sorted(Comparator.comparing(FlightComputerController::id))
                        .map(c -> new Block(c, trace))
                        .toList();
        for (var b : blocks) {
            outputs.put(b.c.id(), b.c.n("trim"));
            plotChannels.put(b.c.id(), FlightComputerControllerPlots.channels(design, b.c));
        }
    }

    /**
     * Returns the earliest task start or completion in boot microseconds, or Long.MAX_VALUE when
     * idle.
     *
     * @return the earliest task start or completion in boot microseconds, or Long.MAX_VALUE when
     *     idle
     */
    public long nextDeadlineUs() {
        long next = Long.MAX_VALUE;
        for (var b : blocks) next = Math.min(next, b.finish >= 0 ? b.finish : b.nextStart);
        return next;
    }

    /**
     * Returns the immutable controller definitions scheduled by this runtime.
     *
     * @return the immutable controller definitions scheduled by this runtime
     */
    public List<FlightComputerController> definitions() {
        return blocks.stream().map(b -> b.c).toList();
    }

    /**
     * Returns the currently published logical outputs indexed by controller ID.
     *
     * @return the currently published logical outputs indexed by controller ID
     */
    public Map<String, Double> outputs() {
        return Map.copyOf(outputs);
    }

    /**
     * Returns the IDs of controller blocks currently active in their state gates.
     *
     * @return the IDs of controller blocks currently active in their state gates
     */
    public Set<String> activeIds() {
        var ids = new HashSet<String>();
        blocks.stream().filter(b -> b.active && !b.closed).forEach(b -> ids.add(b.c.id()));
        return Set.copyOf(ids);
    }

    /**
     * Returns the latest diagnostic values for a controller block.
     *
     * @return the latest diagnostic values for a controller block
     */
    public Map<String, Values> values() {
        var result = new LinkedHashMap<String, Values>();
        for (var b : blocks)
            if (b.values != null) {
                var v = b.values;
                result.put(
                        b.c.id(),
                        new Values(
                                v.measurement(),
                                v.baseline(),
                                v.error(),
                                v.p(),
                                v.i(),
                                v.d(),
                                v.weight(),
                                v.requested(),
                                outputs.get(b.c.id()),
                                v.active() && b.active && !b.closed,
                                v.valid(),
                                v.saturated() && b.active && !b.closed,
                                v.sampleUs(),
                                v.startUs(),
                                v.completionUs()));
            }
        return result;
    }

    /**
     * Installs the actuator-aware output constraint used by saturation and anti-windup
     * calculations.
     *
     * @param id stable identifier
     * @param saturates saturates
     */
    public void constrain(String id, DoubleUnaryOperator saturates) {
        targetSaturation.put(id, saturates);
    }

    /** Neutralizes outputs and invalidates pending controller computations. */
    public void close() {
        for (var b : blocks) {
            b.epoch++;
            b.reset();
            b.closed = true;
            outputs.put(b.c.id(), b.c.n("trim"));
            trace.log("controller.close", "id=" + b.c.id());
        }
    }

    /**
     * Completes due work and starts eligible tasks from frozen measured inputs, honoring
     * independent periods and execution costs.
     *
     * @param now last start value
     * @param fc state value
     * @param signals measurement value
     */
    public void dispatch(long now, RTFC fc, FlightComputerSignals signals) {
        String state = fc.getDesignStateId();
        for (var b : blocks) {
            boolean changed = !state.equals(b.state);
            boolean active = b.c.active(state);
            if (changed) {
                b.epoch++;
                b.closed = false;
                if (b.c.s("baseline").equals("entry") && state.equals(b.c.s("captureState"))) {
                    b.baseline = Double.NaN;
                    b.averageStart = b.averageLast = -1;
                    b.averageCount = 0;
                    b.averageSum = 0;
                    b.state = state;
                    capture(
                            b,
                            now,
                            signals.read(b.c.s("source"), b.c.s("channel")),
                            signals.calibratedUs());
                }
            }
            if (active != b.active || changed) {
                if (b.c.flag("resetIntegral") || !active) b.reset();
                if (!active && !b.c.flag("inactiveHold")) outputs.put(b.c.id(), b.c.n("trim"));
                trace.log(
                        "controller.state",
                        "id=" + b.c.id() + " state=" + state + " active=" + active);
            }
            b.active = active;
            b.state = state;
        }
        for (var b : blocks) if (b.finish == now) complete(b, now);
        for (var b : blocks)
            if (b.finish < 0 && b.nextStart == now) {
                var c = b.c;
                var measurement = signals.read(c.s("source"), c.s("channel"));
                b.input =
                        new Input(
                                measurement,
                                signals.read(c.s("derivativeSource"), c.s("derivativeChannel")),
                                signals.read(c.s("velocitySource"), c.s("velocityChannel")),
                                fc.baro.getFilteredAltitude(),
                                Math.max(0, (now / 1000 - fc.getFlightBeginTime()) / 1000.0),
                                signals.read(c.s("velocitySource"), "z").value(),
                                signals.read(c.s("source"), "roll_rate").value(),
                                signals.calibratedUs());
                b.lastStart = now;
                b.finish = Math.addExact(now, c.us("executionUs"));
                b.nextStart = FlightComputerTaskSchedule.nextStart(now, c.us("periodUs"), b.finish);
                b.workEpoch = b.epoch;
                capture(b, now, measurement, signals.calibratedUs());
                trace.log(
                        "controller.start",
                        "id="
                                + c.id()
                                + " sample_us="
                                + measurement.acquisitionUs()
                                + " start_us="
                                + now
                                + " complete_us="
                                + b.finish
                                + " overrun_us="
                                + Math.max(0, c.us("executionUs") - c.us("periodUs")));
            }
        for (var b : blocks) if (b.finish == now) complete(b, now);
    }

    /**
     * Captures a baseline once its configured fixed, ground-calibration or state-entry condition is
     * satisfied.
     *
     * @param b c value
     * @param now average start value
     * @param sample average last value
     * @param calibrationUs calibration in microseconds
     */
    private void capture(
            Block b, long now, FlightComputerSignals.Sample sample, long calibrationUs) {
        var c = b.c;
        if (c.s("baseline").equals("fixed")) {
            b.baseline = c.n("setpoint");
            return;
        }
        if (Double.isFinite(b.baseline) || !sample.fresh(now, c.us("staleUs"))) return;
        if (c.s("baseline").equals("entry") && !b.state.equals(c.s("captureState"))) return;
        boolean ready = c.s("baseline").equals("ground") ? calibrationUs >= 0 : true;
        if (!ready) return;
        if (b.averageStart < 0) b.averageStart = now;
        if (sample.acquisitionUs() != b.averageLast) {
            b.averageSum += sample.value();
            b.averageCount++;
            b.averageLast = sample.acquisitionUs();
        }
        if (now - b.averageStart >= c.us("captureAverageUs") && b.averageCount > 0) {
            b.baseline = b.averageSum / b.averageCount + c.n("setpoint");
            b.reset();
            trace.log(
                    "controller.baseline",
                    "id="
                            + c.id()
                            + " value="
                            + b.baseline
                            + " captured_us="
                            + now
                            + " samples="
                            + b.averageCount);
        }
    }

    /**
     * Bounds a logical output to the configured controller limits.
     *
     * @param value replacement or input value
     * @param low lower allowed bound
     * @param high upper allowed bound
     * @return computed numeric value
     */
    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }

    /**
     * Measures the distance between an unconstrained command and the actuator-constrained output.
     *
     * @param b b
     * @param value replacement or input value
     * @return computed numeric value
     */
    private double saturationDistance(Block b, double value) {
        return Math.max(
                Math.abs(value - clamp(value, b.c.n("minimum"), b.c.n("maximum"))),
                targetSaturation.getOrDefault(b.c.id(), v -> 0).applyAsDouble(value));
    }

    /**
     * Tests whether a candidate output is limited by controller or actuator bounds.
     *
     * @param b b
     * @param value replacement or input value
     * @return true when the condition described above holds
     */
    private boolean saturated(Block b, double value) {
        return saturationDistance(b, value) > 1e-12;
    }

    /**
     * Publishes a finished computation only if its state epoch is still valid.
     *
     * @param b c value
     * @param now last valid value
     * @throws IllegalStateException when rejected: Nonfinite output from controller
     */
    private void complete(Block b, long now) {
        b.finish = -1;
        var c = b.c;
        var input = b.input;
        b.input = null;
        if (b.workEpoch != b.epoch) {
            trace.log("controller.discard", "id=" + c.id() + " reason=state_or_owner_changed");
            return;
        }
        double measurement = input.measurement().value(),
                weight = 1,
                p = 0,
                i = 0,
                d = 0,
                error = Double.NaN,
                requested = c.n("trim"),
                output = outputs.get(c.id());
        boolean active = b.active && !b.closed;
        boolean valid =
                c.type() == FlightComputerController.Type.CONSTANT
                        || (input.measurement().fresh(b.lastStart, c.us("staleUs"))
                                && Double.isFinite(b.baseline));
        if (c.type() != FlightComputerController.Type.CONSTANT
                && !c.s("derivativeSource").isBlank())
            valid &= input.derivative().fresh(b.lastStart, c.us("staleUs"));
        if (c.type() == FlightComputerController.Type.LEGACY_ROLL
                || c.type() == FlightComputerController.Type.LEGACY_APOGEE)
            valid &= input.velocity().fresh(b.lastStart, c.us("staleUs"));
        if (c.type() == FlightComputerController.Type.VELOCITY_PID) {
            valid &= input.velocity().fresh(b.lastStart, c.us("staleUs"));
            if (valid) {
                double speed = Math.abs(input.velocity().value());
                b.speedActive =
                        speed
                                >= (b.speedActive
                                        ? Math.max(
                                                0, c.n("activationSpeed") - c.n("speedHysteresis"))
                                        : c.n("activationSpeed"));
                weight =
                        clamp(
                                Math.pow(
                                        c.n("referenceSpeed") / Math.max(speed, c.n("speedFloor")),
                                        c.n("exponent")),
                                c.n("minimumWeight"),
                                c.n("maximumWeight"));
                active &= b.speedActive;
            }
        }
        if (active && valid) {
            b.lastValid = now;
            double dt = b.lastControl < 0 ? 0 : (b.lastStart - b.lastControl) / 1e6;
            b.lastControl = b.lastStart;
            if (c.type() == FlightComputerController.Type.CONSTANT)
                requested = c.n("trim") + c.n("direction") * c.n("constantValue");
            else {
                error = b.baseline - measurement;
                if (c.flag("wrapAngle")) error = Math.IEEEremainder(error, 360);
                if (!c.s("derivativeSource").isBlank()) {
                    var derivativeSample = input.derivative();
                    if (derivativeSample.acquisitionUs() > b.lastDerivativeSample) {
                        double interval =
                                b.lastDerivativeSample < 0
                                        ? 0
                                        : (derivativeSample.acquisitionUs()
                                                        - b.lastDerivativeSample)
                                                / 1e6;
                        double alpha =
                                b.lastDerivativeSample < 0
                                        ? 1
                                        : interval / (interval + c.us("filterUs") / 1e6);
                        b.derivative += alpha * (derivativeSample.value() - b.derivative);
                        b.lastDerivativeSample = derivativeSample.acquisitionUs();
                    }
                } else if (b.lastSample >= 0
                        && input.measurement().acquisitionUs() > b.lastSample) {
                    double sampleDt = (input.measurement().acquisitionUs() - b.lastSample) / 1e6;
                    double difference = measurement - b.lastMeasurement;
                    if (c.flag("wrapAngle")) difference = Math.IEEEremainder(difference, 360);
                    double alpha = sampleDt / (sampleDt + c.us("filterUs") / 1e6);
                    b.derivative += alpha * (difference / sampleDt - b.derivative);
                }
                if (input.measurement().acquisitionUs() > b.lastSample) {
                    b.lastSample = input.measurement().acquisitionUs();
                    b.lastMeasurement = measurement;
                }
                p = c.n("kp") * error;
                d = -c.n("kd") * b.derivative;
                double candidate =
                        clamp(b.integral + error * dt, -c.n("integralLimit"), c.n("integralLimit"));
                double old =
                        c.n("trim") + c.n("direction") * weight * (p + c.n("ki") * b.integral + d);
                double trial =
                        c.n("trim") + c.n("direction") * weight * (p + c.n("ki") * candidate + d);
                // Only accept integration that stays inside all calibrated target limits or unwinds
                // saturation.
                if (!saturated(b, trial)
                        || saturationDistance(b, trial) < saturationDistance(b, old))
                    b.integral = candidate;
                i = c.n("ki") * b.integral;
                requested = c.n("trim") + c.n("direction") * weight * (p + i + d);
                if (b.roll != null) {
                    b.roll.update(
                            (float) input.flightSeconds(),
                            (float) input.altitude(),
                            (float) input.velocity().value(),
                            (float) (measurement - b.baseline),
                            (float)
                                    (c.s("derivativeSource").isBlank()
                                            ? input.rollRate()
                                            : input.derivative().value()));
                    p = b.roll.getPContribution();
                    i = b.roll.getIContribution();
                    d = b.roll.getDContribution();
                    requested = c.n("trim") + c.n("direction") * b.roll.getAngle();
                }
                if (b.apogee != null) {
                    b.apogee.setTargetAltitude((float) b.baseline);
                    b.apogee.update(
                            (float) (long) input.flightSeconds(),
                            new RTFudgedAirbrakesData(
                                    (float) measurement,
                                    (float) input.velocity().value(),
                                    (float) input.acceleration(),
                                    false));
                    error = b.apogee.getControlError();
                    p = b.apogee.getPContribution();
                    i = b.apogee.getIContribution();
                    d = b.apogee.getDContribution();
                    requested = c.n("trim") + c.n("direction") * b.apogee.getDeployment();
                }
            }
            if (!Double.isFinite(requested))
                throw new IllegalStateException("Nonfinite output from controller " + c.name());
            output = clamp(requested, c.n("minimum"), c.n("maximum"));
        } else {
            b.reset();
            boolean hold =
                    active
                            && !valid
                            && b.lastValid >= 0
                            && now - b.lastValid <= c.us("invalidHoldUs");
            if (!hold && !(!active && c.flag("inactiveHold"))) output = c.n("trim");
        }
        outputs.put(c.id(), output);
        b.values =
                new Values(
                        measurement,
                        b.baseline,
                        error,
                        p,
                        i,
                        d,
                        weight,
                        requested,
                        output,
                        active,
                        valid,
                        saturated(b, requested),
                        input.measurement().acquisitionUs(),
                        b.lastStart,
                        now);
        trace.log(
                "controller.update",
                "id="
                        + c.id()
                        + " measurement="
                        + measurement
                        + " baseline="
                        + b.baseline
                        + " error="
                        + error
                        + " p="
                        + p
                        + " i="
                        + i
                        + " d="
                        + d
                        + " weight="
                        + weight
                        + " requested="
                        + requested
                        + " output="
                        + output
                        + " active="
                        + active
                        + " valid="
                        + valid
                        + " saturated="
                        + b.values.saturated()
                        + " sample_us="
                        + b.values.sampleUs()
                        + " start_us="
                        + b.lastStart
                        + " complete_us="
                        + now);
    }
}
