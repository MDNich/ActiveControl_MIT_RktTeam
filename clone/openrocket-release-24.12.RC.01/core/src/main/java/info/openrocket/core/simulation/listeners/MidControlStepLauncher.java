package info.openrocket.core.simulation.listeners;

import info.openrocket.core.rocketcomponent.FinSet;
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

	public static FinSet theFinsToModify = null;

	public static SimulationStatus iniStat = null;
	public static SimulationStatus finStat = null;

	public static double theTimeStep = 0.0025;
	public static double wantedTimeStep = 0.0025;

	private static double iniTimeStep = 0;
	private static double finTimeStep = 0;

	public static FlightDataBranch datStorage;

	public static boolean readyToProceed = false;
	public static boolean datIsReadyToCollect = false;

	public static void provideSimStat(SimulationStatus status) {
		iniStat = status;
	}

	public static boolean checkReadyCollection() {
		return datIsReadyToCollect;
	}


	public static SimulationStatus getFinStat() {
		return finStat;
	}

	@Override
	public void startSimulation(SimulationStatus status) throws SimulationException {
		//System.out.println("STARTING SIMULATION");
		//status.copySimStatParameters(iniStat);
		//status.setFlightDataBranch(datStorage);
		super.startSimulation(status);
		iniTimeStep = status.getSimulationTime();
	}

	@Override
	public boolean preStep(SimulationStatus status) throws SimulationException {
		// status.copySimStatParameters(iniStat);
		// status.setFlightDataBranch(datStorage);
		// iniTimeStep = status.getSimulationTime();
		return super.preStep(status);
	}

	@Override
	public void postStep(SimulationStatus status) throws SimulationException {
		finStat = status.clone();
		finTimeStep = status.getSimulationTime();
		//System.out.println("Flight data branch is " + ( datStorage == null ? "null" : "not null" ));
		/*if(datStorage != null) {
			finStat.setFlightDataBranch(datStorage);
		}
		else {
			finStat.setFlightDataBranch(status.getFlightDataBranch());
		}*/
		theTimeStep = finTimeStep - iniTimeStep;
		//System.out.println("[JAVA] READY FOR PYTHON");
		datIsReadyToCollect = true;
		//System.out.println("[JAVA] WAITING FOR PYTHON");
		/*while(!readyToProceed){
			try {
				Thread.sleep(1);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}*/
		readyToProceed = false;
		datIsReadyToCollect = false;



		System.out.println("PROCEEDING TO NEXT STEP");
		//throw new SimulationException("One step only, delta " + delta);
	}


	public static void setCantOfFinDeg(double newCant) {
		theFinsToModify.setCantAngle(Math.PI/180*newCant);
	}
	public static void deltaCantOfFinDeg(double deltaCant) {
		double newCant = getCantOfFinDeg() + deltaCant;
		theFinsToModify.setCantAngle(Math.PI/180*newCant);
	}
	public static double getCantOfFinDeg() {
		return theFinsToModify.getCantAngle()*180/Math.PI;
	}
}
