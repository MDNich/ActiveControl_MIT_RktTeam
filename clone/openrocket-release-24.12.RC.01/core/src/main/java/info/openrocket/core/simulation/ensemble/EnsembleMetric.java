package info.openrocket.core.simulation.ensemble;

import info.openrocket.core.simulation.*;
import info.openrocket.core.unit.UnitGroup;
import java.util.function.ToDoubleFunction;

/** One value per complete flight branch, before any trajectory averaging. */
public enum EnsembleMetric {
    MAX_ALTITUDE("Maximum altitude", UnitGroup.UNITS_DISTANCE, FlightData::getMaxAltitude),
    MAX_VELOCITY("Maximum velocity", UnitGroup.UNITS_VELOCITY, FlightData::getMaxVelocity),
    MAX_ACCELERATION("Maximum acceleration (before recovery)", UnitGroup.UNITS_ACCELERATION, FlightData::getMaxAcceleration),
    MAX_MACH("Maximum Mach number", UnitGroup.UNITS_NONE, FlightData::getMaxMachNumber),
    MIN_STABILITY("Minimum stability (entire flight)", FlightDataType.TYPE_STABILITY.getUnitGroup(), d -> d.getBranch(0).getMinimum(FlightDataType.TYPE_STABILITY)),
    TIME_TO_APOGEE("Time to apogee", UnitGroup.UNITS_LONG_TIME, FlightData::getTimeToApogee),
    FLIGHT_TIME("Flight duration", UnitGroup.UNITS_LONG_TIME, FlightData::getFlightTime),
    LAUNCH_ROD_VELOCITY("Launch rod exit velocity", UnitGroup.UNITS_VELOCITY, FlightData::getLaunchRodVelocity),
    DEPLOYMENT_VELOCITY("Recovery deployment velocity", UnitGroup.UNITS_VELOCITY, FlightData::getDeploymentVelocity),
    GROUND_HIT_VELOCITY("Ground impact velocity", UnitGroup.UNITS_VELOCITY, FlightData::getGroundHitVelocity),
    OPTIMUM_DELAY("Optimum delay", UnitGroup.UNITS_LONG_TIME, FlightData::getOptimumDelay),
    LANDING_DISTANCE("Final horizontal distance", UnitGroup.UNITS_DISTANCE, d -> Math.hypot(d.getBranch(0).getLast(FlightDataType.TYPE_POSITION_X), d.getBranch(0).getLast(FlightDataType.TYPE_POSITION_Y)));

    private final String label;
    private final UnitGroup units;
    private final ToDoubleFunction<FlightData> value;
    EnsembleMetric(String label, UnitGroup units, ToDoubleFunction<FlightData> value) {
        this.label = label; this.units = units; this.value = value;
    }
    public UnitGroup units() { return units; }
    public double value(FlightData flight) { return value.applyAsDouble(flight); }
    @Override public String toString() { return label; }
}
