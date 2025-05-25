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
public class NewControlStepListener extends AbstractSimulationListener {


	public static SimulationStatus initialStat = null;
	public static SimulationStatus latestStatus = null;
	public static double latestTimeStep = -1;

	public static FinSet theFinsToModify = null;

	public static Flag datIsReadyToCollect;
	public static Flag readyToProceed;

	public NewControlStepListener() {
		super();
		datIsReadyToCollect = new Flag();
		readyToProceed = new Flag();
	}


	public static double initial = -1;
	@Override
	public void startSimulation(SimulationStatus status) throws SimulationException {
		status.copySimStatParameters(initialStat);
		super.startSimulation(status);
	}

	@Override
	public boolean preStep(SimulationStatus status) throws SimulationException {
		initial = status.getSimulationTime();
		return super.preStep(status);
	}

	@Override
	public void postStep(SimulationStatus status) throws SimulationException {
		latestStatus = status.clone();
		double finTimeStep = status.getSimulationTime();
		latestTimeStep = finTimeStep - initial;
		System.out.println("[JAVA] READY FOR PYTHON");
		datIsReadyToCollect.engage();
		System.out.println("[JAVA] WAITING FOR PYTHON");
		while(!readyToProceed.get()){
			try {
				Thread.sleep(1);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		readyToProceed.disengage();
		datIsReadyToCollect.disengage();



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
