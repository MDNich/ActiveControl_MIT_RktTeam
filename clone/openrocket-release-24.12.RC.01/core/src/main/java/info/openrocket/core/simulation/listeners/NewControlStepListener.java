package info.openrocket.core.simulation.listeners;

import info.openrocket.core.rocketcomponent.FinSet;
import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.simulation.FlightDataBranch;
import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.simulation.exception.SimulationException;
import info.openrocket.core.util.ArrayList;

import java.util.Iterator;

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
	public static double IdecayFactor = 1;
	public static double servoStepCount = 1023.0;
	public static double invVelSqCoeff = 1;
	public static double iniVel = 10.0;

	public static FinSet theFinsToModify = null;

	public static Flag datIsReadyToCollect;
	public static Flag readyToProceed;

	public static ArrayList<Double> pastOmegaZ;
	public static ArrayList<Double> finCantLog;

	public static Rocket theRocket;


	public static SimulationStatus lastStat = null;

	public static double totErr = 0;


	// THESE WILL BE MODIFIED FROM PYTHON
	public static boolean useRK6 = true;
	public static double velMinThresh = 20;
	public static double kP = 0;
	public static double kI = 0;
	public static double kD = 0;
	public static double kVelRocket = 0;
	public static double kVel2Rocket = 0;
	public static double kVel3Rocket = 0;
	public static double kAccelRocket = 0;
	public static double desiredRotVel = 0;
	public static double constFixed = 0;


	public NewControlStepListener() {
		super();
		pastOmegaZ = new ArrayList<>();
		finCantLog = new ArrayList<>();
		datIsReadyToCollect = new Flag();
		readyToProceed = new Flag();
	}


	public static double initial = -1;
	@Override
	public void startSimulation(SimulationStatus status) throws SimulationException {
		status.copySimStatParameters(initialStat);
		super.startSimulation(status);
		lastStat = status.clone();

	}

	@Override
	public boolean preStep(SimulationStatus status) throws SimulationException {
		initial = status.getSimulationTime();
		lastStat = status.clone();
		return super.preStep(status);
	}

	@Override
	public void postStep(SimulationStatus status) throws SimulationException {
		latestStatus = status.clone();
		double finTimeStep = status.getSimulationTime();
		latestTimeStep = finTimeStep - initial;

		//System.out.println("Controller Engaged");

		theFinsToModify  = getTheFinsToModify(status);
		if (status.getRocketVelocity().length() > velMinThresh) {
			setCantOfFinDeg(finCantController(status));
		}
		else {
			setCantOfFinDeg(0);
		}
		pastOmegaZ.add(status.getRocketRotationVelocity().z);
		finCantLog.add(getCantOfFinDeg());

		//System.out.println("Current altitude: " + status.getRocketPosition().z);
		//System.out.println("Current vel: " + status.getRocketVelocity().length());

		//System.out.println("viewed cant: " + theFinsToModify.getCantAngle()*180/Math.PI + " degrees");
		//status.getConfiguration().getRocket().fireComponentChangeEvent(4); // AERODYNAMIC

		//System.out.println("Controller Disengaged");

		//System.out.println("PROCEEDING TO NEXT STEP");

		/*System.out.println("[JAVA] READY FOR PYTHON");
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



		*/
		if (status.apogeeReached) {
			throw new SimulationException("Apogee => done");
		}

	}


	public static double finCantController(SimulationStatus currentStat) {

		double currentSpeed = currentStat.getRocketVelocity().length();
		if(currentSpeed < velMinThresh) {
			System.out.println("SHOULD NEVER GET HERE");
			return 0;
		}

		double previousCant = theFinsToModify.getCantAngle();
		double translatVel = currentStat.getRocketVelocity().length();
		double rotVel = currentStat.getRocketRotationVelocity().z;
		double lastRotVel = lastStat.getRocketRotationVelocity().z;
		double lastErr = desiredRotVel-lastRotVel;
		double err = desiredRotVel - rotVel;
		lastStat = currentStat.clone();
		totErr = err + totErr*IdecayFactor;


		double thrusting = constFixed;

		thrusting += err*kP;
		//System.out.println(thrusting);

		thrusting += (err-lastErr)*kD;
		//System.out.println(thrusting);
		thrusting += totErr*kI;

		//thrusting *= invVelSqCoeff*iniVel*iniVel/translatVel/translatVel;
		thrusting += -1*rotVel*kVelRocket;
		//thrusting += -1*rotVel*Math.abs(rotVel)*kVel2Rocket;
		//thrusting += -1*rotVel*Math.abs(rotVel*rotVel)*kVel3Rocket;
		//thrusting += -1*(rotVel-lastRotVel)*kAccelRocket;


		//System.out.println(thrusting);
		//System.out.println("----------------");
		return thrusting;

	}


	// don't worry about it
	public static FinSet getTheFinsToModify(SimulationStatus status) {
		ArrayList<FinSet> finSets = new ArrayList<>();
		Rocket rocket = status.getConfiguration().getRocket();
        for (Iterator<RocketComponent> it = rocket.iterator(true); it.hasNext(); ) {
            RocketComponent component = it.next();

			if(component instanceof FinSet) {
				finSets.add((FinSet) component);
			}


        }
		return finSets.get(0);
	}




	public static void setCantOfFinDeg(double newCant) {
		double stepSize = 30.0/servoStepCount;
		double numStepsFromZero = (int) (newCant/stepSize);
		theFinsToModify.setCantAngle(Math.PI/180*numStepsFromZero*stepSize);
	}
	public static void deltaCantOfFinDeg(double deltaCant) {
		double newCant = getCantOfFinDeg() + deltaCant;
		theFinsToModify.setCantAngle(Math.PI/180*newCant);
	}
	public static double getCantOfFinDeg() {
		return theFinsToModify.getCantAngle()*180/Math.PI;
	}
}
