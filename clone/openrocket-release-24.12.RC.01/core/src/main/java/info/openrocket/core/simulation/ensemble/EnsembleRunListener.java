package info.openrocket.core.simulation.ensemble;

import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.listeners.AbstractSimulationListener;
import info.openrocket.core.models.wind.*;
import info.openrocket.core.util.Coordinate;

/**
 * Per-run perturbations. Burn timing, propellant mass and nominal motor database remain unchanged.
 */
public final class EnsembleRunListener extends AbstractSimulationListener {
    private final EnsembleSettings settings;
    private final GaussianThrustNoise thrustNoise;
    private final double east, north;

    /**
     * Creates a EnsembleRunListener with the supplied dependencies and per-instance state.
     *
     * @param settings validated configuration for this operation
     * @param motorSeed motor seed
     * @param east initial east
     * @param north initial north
     */
    public EnsembleRunListener(
            EnsembleSettings settings, long motorSeed, double east, double north) {
        this.settings = settings;
        this.thrustNoise =
                new GaussianThrustNoise(motorSeed, settings.motorSigma(), settings.noiseInterval());
        this.east = east;
        this.north = north;
    }

    /**
     * Marks the variation listener as simulation infrastructure.
     *
     * @return true when the condition described above holds
     */
    @Override
    public boolean isSystemListener() {
        return true;
    }

    /**
     * Installs this member's deterministic atmospheric variation at simulation start.
     *
     * @param status stat in microseconds
     */
    @Override
    public void startSimulation(SimulationStatus status) {
        // Hold the existing turbulent wind trace fixed, isolating the selected uncertainty sources.
        var conditions = status.getSimulationConditions();
        var wind = conditions.getWindModel();
        if (wind instanceof PinkNoiseWindModel p)
            conditions.setWindModel(p.withSeed(settings.seed()));
        else if (wind instanceof MultiLevelPinkNoiseWindModel p)
            conditions.setWindModel(p.withSeed(settings.seed()));
    }

    /**
     * Adds this member's atmospheric wind perturbation to the modeled wind.
     *
     * @param status stat in microseconds
     * @param wind wind
     * @return the resulting Coordinate
     */
    @Override
    public Coordinate postWindModel(SimulationStatus status, Coordinate wind) {
        return wind.add(east, north, 0);
    }

    /**
     * Adds the seeded Gaussian thrust perturbation during burn and clamps total thrust at zero.
     *
     * @param status stat in microseconds
     * @param thrust thrust
     * @return computed numeric value
     */
    @Override
    public double postSimpleThrustCalculation(SimulationStatus status, double thrust) {
        if (settings.motorSigma() == 0) return Double.NaN;
        double correction = 0;
        for (var motor : status.getActiveMotors()) {
            double t = motor.getMotorTime(status.getSimulationTime());
            if (!motor.isThrusting() || t <= 0 || t >= motor.getBurnTime()) continue;
            double nominal = motor.getMotor().getThrust(t);
            for (int i = 0; i < motor.getMotorCount(); i++) {
                long identity = ((long) motor.getID().hashCode() << 32) ^ i;
                correction += Math.max(0, nominal + thrustNoise.value(t, identity)) - nominal;
            }
        }
        return Math.max(0, thrust + correction);
    }

    /**
     * Limits physics steps to preserve the configured thrust-noise sampling scale.
     *
     * @param status stat in microseconds
     * @param step step
     * @return computed numeric value
     */
    public static double limitStep(SimulationStatus status, double step) {
        for (var listener : status.getSimulationConditions().getSimulationListenerList()) {
            if (listener instanceof EnsembleRunListener e
                    && e.settings.motorSigma() > 0
                    && status.getActiveMotors().stream().anyMatch(MotorClusterState::isThrusting))
                step = Math.min(step, e.settings.noiseInterval() / 4);
        }
        return step;
    }
}
