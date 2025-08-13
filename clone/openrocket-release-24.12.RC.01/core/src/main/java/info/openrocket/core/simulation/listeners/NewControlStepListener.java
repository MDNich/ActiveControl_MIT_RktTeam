package info.openrocket.core.simulation.listeners;

import info.openrocket.core.rocketcomponent.FinSet;
import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.simulation.exception.SimulationException;
import info.openrocket.core.util.ArrayList;
import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.Quaternion;

import java.util.Iterator;

import static java.lang.Math.*;

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
	public static ArrayList<Double> pastThetaZ;
	public static ArrayList<Double> finCantLog;

	public static Rocket theRocket;


	public static SimulationStatus lastStat = null;

	public static double totErrVel = 0;
	public static double totErrAng = 0;


	// THESE WILL BE MODIFIED FROM PYTHON
	public static boolean useRK6 = true;
	public static boolean roundToNearest5 = true;
	public static double velMinThresh = 20;
	public static double kP_VEL = 0;
	public static double kP_ANG = 0;
	public static double kI_VEL = 0;
	public static double kI_ANG = 0;
	public static double kD_VEL = 0;
	public static double kD_ANG = 0;
	public static double kVelRocket = 0;
	public static double kVel2Rocket = 0;
	public static double kVel3Rocket = 0;
	public static double kAccelRocket = 0;
	public static double desiredRotVel = 0;
	public static double desiredRotAng = 0;
	public static double constFixed = 0;



	public static boolean velocityPIDon = true;
	public static boolean positionPIDon = true;


	public NewControlStepListener() {
		super();
		pastOmegaZ = new ArrayList<>();
		pastThetaZ = new ArrayList<>();
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
		pastThetaZ.add(toDegrees(toEulerAngles(status.getRocketOrientationQuaternion()).z));
		finCantLog.add(getCantOfFinDeg());

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
		double rotAng = toDegrees(toEulerAngles(currentStat.getRocketOrientationQuaternion()).z);
		double lastRotVel = lastStat.getRocketRotationVelocity().z;
		double lastRotAng = toDegrees(toEulerAngles(lastStat.getRocketOrientationQuaternion()).z);
		double lastErrVel = desiredRotVel - lastRotVel;
		double lastErrAng = desiredRotAng - lastRotAng;
		double errVel = desiredRotVel - rotVel;
		double errAng = desiredRotAng - rotAng;

		lastStat = currentStat.clone();
		double thrusting = constFixed;

		if (velocityPIDon) {
			totErrVel = errVel + totErrVel * IdecayFactor;

			thrusting += errVel * kP_VEL;
			thrusting += (errVel - lastErrVel) * kD_VEL;
			thrusting += totErrVel * kI_VEL;
		}
		if (positionPIDon) {
			totErrAng = errAng + totErrAng * IdecayFactor;

			thrusting += errAng * kP_ANG;
			thrusting += (errAng - lastErrAng) * kD_ANG;
			thrusting += totErrAng * kI_ANG;
		}

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





	public static Coordinate toEulerAngles(Quaternion q) {

		// roll (x-axis rotation)
		double sinr_cosp = 2 * (q.getW() * q.getX() + q.getY() * q.getZ());
		double cosr_cosp = 1 - 2 * (q.getX() * q.getX() + q.getY() * q.getY());
		double angleX = atan2(sinr_cosp, cosr_cosp);

		// pitch (y-axis rotation)
		double sinp = sqrt(1 + 2 * (q.getW() * q.getY() - q.getX() * q.getZ()));
		double cosp = sqrt(1 - 2 * (q.getW() * q.getY() - q.getX() * q.getZ()));
		double angleY = 2 * atan2(sinp, cosp) - PI / 2;

		// yaw (z-axis rotation)
		double siny_cosp = 2 * (q.getW() * q.getZ() + q.getX() * q.getY());
		double cosy_cosp = 1 - 2 * (q.getY() * q.getY() + q.getZ() * q.getZ());
		double angleZ = atan2(siny_cosp, cosy_cosp);

		if (roundToNearest5) {
			angleX = round(angleX / (Math.PI / 180 * 1));// * (Math.PI / 180 * 5);
			angleY = round(angleY / (Math.PI / 180 * 1));// * (Math.PI / 180 * 5);
			angleZ = round(angleZ / (Math.PI / 180 * 1));// * (Math.PI / 180 * 5);
		}

		return new Coordinate(angleX, angleY, angleZ);
	}
}
