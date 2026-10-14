package info.openrocket.core.simulation.ensemble;

import info.openrocket.core.simulation.*;
import info.openrocket.core.aerodynamics.AerodynamicForces;
import info.openrocket.core.masscalc.RigidBody;
import info.openrocket.core.simulation.listeners.AbstractSimulationListener;
import info.openrocket.core.models.wind.*;
import info.openrocket.core.util.Coordinate;

/**
 * Per-run perturbations. Mass scales the whole active vehicle, including remaining propellant.
 * Motor thrust/burn timing and the nominal component and motor databases remain unchanged.
 */
public final class EnsembleRunListener extends AbstractSimulationListener {
    private final EnsembleSettings settings;
    private final GaussianThrustNoise thrustNoise;
    private final double east, north, windDirectionOffset;
    private final double massMultiplier, dragMultiplier, rollForceMultiplier, rollDampingMultiplier;

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
        this(settings, new EnsembleRunParameters(1, motorSeed, 288.15, 101325, east, north));
    }

    public EnsembleRunListener(EnsembleSettings settings, EnsembleRunParameters inputs) {
        this.settings = settings;
        this.thrustNoise = new GaussianThrustNoise(inputs.motorSeed(), settings.motorSigma(), settings.noiseInterval());
        this.east = inputs.windEast();
        this.north = inputs.windNorth();
        this.windDirectionOffset = inputs.windDirectionOffset();
        this.massMultiplier = inputs.massMultiplier();
        this.dragMultiplier = inputs.dragMultiplier();
        this.rollForceMultiplier = inputs.rollForceMultiplier();
        this.rollDampingMultiplier = inputs.rollDampingMultiplier();
    }

    /** Scale mass and all inertias together, retaining the CG and burn-dependent mass profile. */
    @Override
    public RigidBody postMassCalculation(SimulationStatus status, RigidBody mass) {
        if (massMultiplier == 1) return null;
        return new RigidBody(mass.cm.setWeight(mass.getMass() * massMultiplier),
                mass.Ixx * massMultiplier, mass.Iyy * massMultiplier, mass.Izz * massMultiplier);
    }

    @Override
    public AerodynamicForces postAerodynamicCalculation(SimulationStatus status, AerodynamicForces forces) {
        if (dragMultiplier == 1 && rollForceMultiplier == 1 && rollDampingMultiplier == 1) return null;
        if (dragMultiplier != 1) {
            forces.setCD(forces.getCD() * dragMultiplier);
            forces.setCDaxial(forces.getCDaxial() * dragMultiplier);
            forces.setFrictionCD(forces.getFrictionCD() * dragMultiplier);
            forces.setPressureCD(forces.getPressureCD() * dragMultiplier);
            forces.setBaseCD(forces.getBaseCD() * dragMultiplier);
            forces.setOverrideCD(forces.getOverrideCD() * dragMultiplier);
        }
        // Preserve other listeners' roll moments, while scaling the forcing/damping terms.
        if (rollForceMultiplier != 1 || rollDampingMultiplier != 1) {
            double force = forces.getCrollForce(), damping = forces.getCrollDamp();
            forces.setCroll(forces.getCroll() + force * (rollForceMultiplier - 1)
                    - damping * (rollDampingMultiplier - 1));
            forces.setCrollForce(force * rollForceMultiplier);
            forces.setCrollDamp(damping * rollDampingMultiplier);
        }
        return forces;
    }

    /** Euler descent does not invoke aerodynamic listeners; apply the same overall Cd factor there. */
    public static double descentDrag(SimulationStatus status, double cd) {
        for (var listener : status.getSimulationConditions().getSimulationListenerList()) {
            if (listener instanceof EnsembleRunListener e) cd *= e.dragMultiplier;
        }
        return cd;
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
        // OR bearings increase clockwise from North: E = speed*sin(bearing).
        double cos = Math.cos(windDirectionOffset), sin = Math.sin(windDirectionOffset);
        return new Coordinate(wind.x * cos + wind.y * sin + east,
                wind.y * cos - wind.x * sin + north, wind.z);
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
