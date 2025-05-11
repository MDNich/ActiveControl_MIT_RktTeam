package info.openrocket.core.simulation.listeners;

import info.openrocket.core.simulation.FlightDataBranch;
import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.simulation.exception.SimulationException;

/**
 * Simulation listener that launches a rocket from a specific altitude.
 * <p>
 * The altitude is read from the system property "openrocket.airstart.altitude"
 * if defined, otherwise a default altitude of 1000 meters is used.
 */
public class MidControlStepLauncher extends AbstractSimulationListener {

	/** Default launch altitude */
	private static double DEFAULT_ALTITUDE = 1000.0;

	/* Required data prior to the one timeStep
	* lastAltitude
	* lastVel
	* lastOrientation
	* lastAngularVel
	*/

	public static SimulationStatus iniStat = null;
	public static SimulationStatus finStat = null;


	private static double iniTimeStep = 0;
	private static double finTimeStep = 0;

	public static FlightDataBranch datStorage;

	public static void provideSimStat(SimulationStatus status) {
		iniStat = status;
	}

	public static SimulationStatus getFinStat() {
		return finStat;
	}

	@Override
	public void startSimulation(SimulationStatus status) throws SimulationException {
		status.copySimStatParameters(iniStat);
		status.setFlightDataBranch(datStorage);
		super.startSimulation(status);
		iniTimeStep = status.getSimulationTime();
	}

	@Override
	public void postStep(SimulationStatus status) throws SimulationException {
		finStat = status.clone();
		finTimeStep = status.getSimulationTime();
		System.out.println("Flight data branch is " + ( datStorage == null ? "null" : "not null" ));
		if(datStorage != null) {
			finStat.setFlightDataBranch(datStorage);
		}
		else {
			finStat.setFlightDataBranch(status.getFlightDataBranch());
		}

		double delta = finTimeStep - iniTimeStep;
		System.out.println("Delta: " + delta);
		throw new SimulationException("One step only, delta " + delta);
	}
}
