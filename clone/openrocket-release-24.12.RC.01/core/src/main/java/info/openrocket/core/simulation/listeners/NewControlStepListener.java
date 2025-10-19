package info.openrocket.core.simulation.listeners;

import info.openrocket.core.rocketcomponent.FinSet;
import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.rocketcomponent.TabControlledTrapezoidFinSet;
import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.simulation.exception.SimulationException;
import info.openrocket.core.util.ArrayList;
import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.Quaternion;

import java.util.Iterator;

import static info.openrocket.core.simulation.listeners.ControllerState.*;
import static java.lang.Math.*;

/**
 * Simulation listener that launches a rocket from a specific altitude.
 * <p>
 * The altitude is read from the system property "openrocket.airstart.altitude"
 * if defined, otherwise a default altitude of 1000 meters is used.
 */
public class NewControlStepListener extends AbstractSimulationListener {

    public static ControllerState currentState = HALTED;
    public static double targetAngle;
    public static double movingStartTime;
    public static double lastIterTime;
    public static double ctrlOut;
    public static double iniAngle;
    public static double degPerSec = 300; // really 300 deg/sec

    public static SimulationStatus initialStat = null;
    public static SimulationStatus latestStatus = null;
    public static double latestTimeStep = -1;
    public static double IdecayFactor = 1;
    public static double servoStepCount = 4097.0; // number of discrete steps the servo can make
    public static final double servoRangeAngleDeg = 120.0; // total range of motion of the servo
    public static double SERVO_REFRESH_TIME = 2e-10; // seconds
    public static double invVelSqCoeff = 1;
    public static double iniVel = 10.0;


    public static boolean simulateUsingTabs = true;

    public static FinSet theFinsToModify = null;

    public static Flag datIsReadyToCollect;
    public static Flag readyToProceed;

    public static ArrayList<Double> pastOmegaZ;
    public static ArrayList<Double> pastThetaZ;
    public static ArrayList<Double> finCantLog;
    public static ArrayList<Double> finTabAngleLog;
    public static ArrayList<Double> desiredFinTabAngleLog;
    public static ArrayList<Double> rocketVelMagnitudeLog;

    public static Rocket theRocket;


    public static SimulationStatus lastStat = null;

    public static double totErrVel = 0;
    public static double totErrAng = 0;


    // THESE WILL BE MODIFIED FROM PYTHON
    public static boolean flagPrintDebugMsg = false;
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


    public static double WIND_EVENT_1_GUST = 10; // delta Z in mrad/sec
    public static double WIND_EVENT_2_GUST = 20; // delta Z in mrad/sec
    public static double WIND_EVENT_3_GUST = -40; // delta Z in mrad/sec
    public static double WIND_EVENT_1_TIMESTAMP = 1.5; // seconds after launch when wind gust hits
    public static double WIND_EVENT_2_TIMESTAMP = 4; // seconds after launch when wind gust hits
    public static double WIND_EVENT_3_TIMESTAMP = 7; // seconds after launch when wind gust hits
    private static boolean didFireWindEvent1 = false;
    private static boolean didFireWindEvent2 = false;
    private static boolean didFireWindEvent3 = false;

    public static double lastServoCommandTimestamp = 0;


    public static boolean velocityPIDon = true;
    public static boolean positionPIDon = true;

    private static double secondToLastTabAngle = 0;
    private static double lastTabAngle = 0;



    public static final double refVelSq = 1e4;

    public static final double B0 = 7.73e3;
    public static final double B1 = -94.34;
    public static final double A0 = 7.73e3;
    public static final double A1 = 180.4;
    public static final double A2 = 2.068;


    public NewControlStepListener() {
        super();
        pastOmegaZ = new ArrayList<>();
        pastThetaZ = new ArrayList<>();
        finCantLog = new ArrayList<>();
        finTabAngleLog = new ArrayList<>();
        desiredFinTabAngleLog = new ArrayList<>();
        rocketVelMagnitudeLog = new ArrayList<>();
        datIsReadyToCollect = new Flag();
        readyToProceed = new Flag();
        currentState = HALTED;
    }


    public static double initial = -1;
    @Override
    public void startSimulation(SimulationStatus status) throws SimulationException {
        status.copySimStatParameters(initialStat);
        super.startSimulation(status);
        lastStat = status.clone();
        currentState = READY;
        System.out.println("[JAVA] Confirmed reception of PID Coeffs ANG {" + kP_ANG + " " + kI_ANG + " " + kD_ANG + "} ");

    }

    @Override
    public boolean preStep(SimulationStatus status) throws SimulationException {
        initial = status.getSimulationTime();
        lastStat = status.clone();
        return super.preStep(status);
    }

    public void postStepAction(SimulationStatus status) {
        double currentTime = status.getSimulationTime();
        if(WIND_EVENT_1_TIMESTAMP == ((int) (currentTime*10)/10.0)) {
            if (!didFireWindEvent1) {
                windEvent(status,WIND_EVENT_1_GUST);
                didFireWindEvent1 = true;
            }
            if (flagPrintDebugMsg) {
                System.out.println("WIND EVENT 1 at t=" + currentTime);
            }
        }
        if(WIND_EVENT_2_TIMESTAMP == ((int) (currentTime*10)/10.0)) {
            if (!didFireWindEvent2) {
                windEvent(status,WIND_EVENT_2_GUST);
                didFireWindEvent2 = true;
            }
            if (flagPrintDebugMsg) {
                System.out.println("WIND EVENT 2 at t=" + currentTime);
            }
        }
        if(WIND_EVENT_3_TIMESTAMP == ((int) (currentTime*10)/10.0)) {
            if (!didFireWindEvent3) {
                windEvent(status,WIND_EVENT_3_GUST);
                didFireWindEvent3 = true;
            }
            if (flagPrintDebugMsg) {
                System.out.println("WIND EVENT 3 at t=" + currentTime);
            }
        }
        if (simulateUsingTabs) {
            switch (currentState) {
                case HALTED:
                    if (status.getRocketVelocity().length() > velMinThresh) {
                        currentState = READY;
                        System.out.println("Exiting HALTED state");
                    }
                    setFinTabAngleDumb(0);
                    secondToLastTabAngle = lastTabAngle;
                    lastTabAngle = getFinTabAngle();
                    break;
                case READY:
                    theFinsToModify = getTheFinsToModifyTabs(status);
                    secondToLastTabAngle = lastTabAngle;
                    lastTabAngle = getFinTabAngle();
                    if (status.getRocketVelocity().length() > velMinThresh) {
                        iniAngle = getFinTabAngle();
                        ctrlOut = finCantController_Tabs(status);


                        if (latestStatus.getSimulationTime() - lastServoCommandTimestamp < SERVO_REFRESH_TIME) {
                            System.out.println("too early !! try again in " + (SERVO_REFRESH_TIME - (latestStatus.getSimulationTime() - lastServoCommandTimestamp)) + " seconds");
                            return; // no command allowed.
                        }
                        if (Math.abs(iniAngle-ctrlOut) > 1.5*servoRangeAngleDeg/servoStepCount) {
                            targetAngle = ctrlOut;
                            movingStartTime = status.getSimulationTime();
                            currentState = MOVING;
                            lastServoCommandTimestamp = movingStartTime;
                            lastIterTime = status.getSimulationTime();
                            System.out.println("STARTING MOVING PROC: current angle " + iniAngle + ", target angle " + targetAngle + ", time " + movingStartTime);
                        }
                    }
                    else {
                        currentState = HALTED;
                        setFinTabAngleDumb(0);
                        System.out.println("Entering HALTED state");
                    }
                    break;
                case MOVING:
                    int factor = targetAngle > iniAngle ?  1 : -1;



                    double angleToSet = iniAngle + factor*degPerSec*(status.getSimulationTime() - movingStartTime);

                    // use second order approx from dynamics.
                    double dt = latestTimeStep;// status.getSimulationTime() - lastIterTime;
                    System.out.println("linear output: " + angleToSet + " dt " + dt);
                    angleToSet = (B0*angleToSet +A1/dt*lastTabAngle - A2/dt/dt*(-2*lastTabAngle+secondToLastTabAngle))/(A2/dt/dt+A1/dt+A0);
                    System.out.println("Proposed angle: " + angleToSet);


                    if (iniAngle < targetAngle) {
                        /*if (angleToSet < getFinTabAngle()) {
                            angleToSet *= -1;
                            System.out.println("flipped angle to " + angleToSet);
                            //angleToSet = getFinTabAngle()*1.01; // tf bro
                        }*/
                        if (angleToSet >= targetAngle) {
                            angleToSet = targetAngle;
                            System.out.println("Reached target " + targetAngle + " delta t " + (status.getSimulationTime() - movingStartTime));
                            currentState = READY;
                        }
                    }
                    else if (iniAngle > targetAngle) {
                        /*if (angleToSet > getFinTabAngle()) {
                            angleToSet *= -1;
                            System.out.println("flipped angle to " + angleToSet);
                            //angleToSet = getFinTabAngle(); // tf bro
                        }*/
                        if (angleToSet <= targetAngle) {
                            angleToSet = targetAngle;
                            System.out.println("Reached target " + targetAngle + " delta t " + (status.getSimulationTime() - movingStartTime));
                            currentState = READY;
                        }
                    }
                    if (status.getSimulationTime() - movingStartTime > SERVO_REFRESH_TIME) {
                        System.out.println("TIMEOUT REACHED");
                        currentState = READY;
                        return;
                    }

                    // low-level
                    System.out.println("[JAVA] current time " + status.getSimulationTime() + "\nSetting angle " + angleToSet);
                    setFinTabAngleDumb(angleToSet);
                    secondToLastTabAngle = lastTabAngle;
                    lastTabAngle = getFinTabAngle();
                    lastIterTime = status.getSimulationTime();
                    break;
            }
        }
    }


    @Override
    public void postStep(SimulationStatus status) throws SimulationException {
        latestStatus = status.clone();
        double finTimeStep = status.getSimulationTime();
        latestTimeStep = finTimeStep - initial;

        //System.out.println("Controller Engaged");

        if(!simulateUsingTabs) {
            theFinsToModify = getTheFinsToModify(status);
            if (status.getRocketVelocity().length() > velMinThresh) {
                setCantOfFinDeg(finCantController(status));
            } else {
                setCantOfFinDeg(0);
            }
            pastOmegaZ.add(status.getRocketRotationVelocity().z);
            pastThetaZ.add(toDegrees(toEulerAngles(status.getRocketOrientationQuaternion()).z));
            finCantLog.add(getCantOfFinDeg());
        }

        else { // using tabs

            postStepAction(status);


            pastOmegaZ.add(status.getRocketRotationVelocity().z);
            pastThetaZ.add(toDegrees(toEulerAngles(status.getRocketOrientationQuaternion()).z));
            finTabAngleLog.add(getFinTabAngle());
            desiredFinTabAngleLog.add(finCantController_Tabs(status,true));
            rocketVelMagnitudeLog.add(status.getRocketVelocity().length());
        }



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


    public static void windEvent(SimulationStatus status,double deltaZ) {
        status.setRocketRotationVelocity(new Coordinate(status.getRocketRotationVelocity().x,status.getRocketRotationVelocity().y,status.getRocketRotationVelocity().z+deltaZ));
    }

    public static double finCantController_Tabs(SimulationStatus currentStat, boolean flagOk) {
        double currentSpeed = currentStat.getRocketVelocity().length();
        if(currentSpeed < velMinThresh) {
            if(!flagOk) {
                System.out.println("SHOULD NEVER GET HERE");
            }
            return 0;
        }
        double previousAngle = getFinTabAngle();
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

        double velToUseForGainSq = currentSpeed*currentSpeed;
        if (currentSpeed < velMinThresh) {
            velToUseForGainSq = velMinThresh*velMinThresh;
        }

        if (velocityPIDon) {
            totErrVel = errVel + totErrVel * IdecayFactor;

            thrusting += errVel * kP_VEL*refVelSq/velToUseForGainSq;
            thrusting += (errVel - lastErrVel) * kD_VEL*refVelSq/velToUseForGainSq;
            thrusting += totErrVel * kI_VEL*refVelSq/velToUseForGainSq;
        }
        if (positionPIDon) {
            totErrAng = errAng + totErrAng * IdecayFactor;

            thrusting += errAng * kP_ANG*refVelSq/velToUseForGainSq;
            thrusting += (errAng - lastErrAng) * kD_ANG*refVelSq/velToUseForGainSq;
            thrusting += totErrAng * kI_ANG*refVelSq/velToUseForGainSq;
        }
        return thrusting;
    }
    public static double finCantController_Tabs(SimulationStatus currentStat) {
        return finCantController_Tabs(currentStat,false);
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

    // don't worry about it
    public static FinSet getTheFinsToModifyTabs(SimulationStatus status) {
        ArrayList<FinSet> finSets = new ArrayList<>();
        Rocket rocket = status.getConfiguration().getRocket();
        for (Iterator<RocketComponent> it = rocket.iterator(true); it.hasNext(); ) {
            RocketComponent component = it.next();

            if(component instanceof TabControlledTrapezoidFinSet) {
                finSets.add((FinSet) component);
            }


        }
        return finSets.get(0);
    }




    public static void setFinTabAngle(double newAngle) {
        double stepSize = servoRangeAngleDeg/servoStepCount;
        double numStepsFromZero = (int) (newAngle/stepSize);
        if (latestStatus.getSimulationTime() - lastServoCommandTimestamp < SERVO_REFRESH_TIME) {
            return; // no command allowed.
        }
        ((TabControlledTrapezoidFinSet) theFinsToModify).setTabAngle(Math.PI/180*numStepsFromZero*stepSize);
        lastServoCommandTimestamp = latestStatus.getSimulationTime();
        if (flagPrintDebugMsg) {
            System.out.println("[JAVA] Actuated a servo change to " + newAngle + " degrees, which is " + numStepsFromZero + " steps.");
        }
    }

    public static void setFinTabAngleDumb(double newAngle) {
        if (newAngle > 10) {
            newAngle = 10;
        }
        if (newAngle < -10) {
            newAngle = -10;
        }
        ((TabControlledTrapezoidFinSet) theFinsToModify).setTabAngle(Math.PI/180*newAngle);
    }
    public static double getFinTabAngle() {
        //double stepSize = servoRangeAngleDeg/servoStepCount;
        //double numStepsFromZero = (int) (newAngle/stepSize);
        return ((TabControlledTrapezoidFinSet) theFinsToModify).getTabAngle()*180/PI;
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
