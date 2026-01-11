package info.openrocket.core.simulation.listeners;

import info.openrocket.core.rocketcomponent.AirbrakeSet;
import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.simulation.exception.SimulationException;


import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import static info.openrocket.core.simulation.listeners.AirbrakesControllerState.*;
import static info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener.*;
import static info.openrocket.core.simulation.listeners.NewControlStepListener.getRocketCD;
import static info.openrocket.core.simulation.listeners.NewControlStepListener.getRocketMass;
import static java.lang.Math.toDegrees;

public class AirbrakesControllerListener extends AbstractSimulationListener{
    public static SimulationStatus currentStatus;
    public static SimulationStatus initialStatus;
    public static AirbrakeSet theAirbrakes;
    public static Rocket theRocket;


    public static double targetAlt = 5500.0;
    public static double latestTimeStep = -1;
    public static double predictedAlt;
    public static ArrayList<Double> pastOmegaZ;
    public static ArrayList<Double> pastThetaZ;
    public static ArrayList<Double> finTabAngleLog;
    public static ArrayList<Double> rktVelMagLog;
    public static ArrayList<Double> rktAltLog;
    public static ArrayList<Double> CldLog;
    public static ArrayList<Double> Qlog;
    public static ArrayList<Double> CldArefDLog;
    public static ArrayList<AirbrakesAccelerationMeasurement> accelMeasurements;
    public static AirbrakesControllerState state = AirbrakesControllerState.DISABLED;
    public static boolean shouldPrep = false;
    public static AirbrakesAccelerationMeasurement accelDat;

    public static double mass = 53.48;
    public static double g = 9.81;
    public static double rho = 0.736;
    public static double airbrakesCd = AirbrakeSet.CD_perp;//1.28;
    public static double rocketCd = 0.5859;
    public static double aRef = 0.01929;
    public static double alpha = 0.2463;
    public static double t_apog = 31.89; //time of apogee
    // maximum airbrakes area
    public static double a_max = 0.0066; // TODO get from AirbrakesSet
    public static int counter = 0;
    public static int roundToHowMuch = 100; // desired altitude correction
    public static double desiredDeltaX = 0;

    public static double airbrakesCtrlStartTime = 1e10;
    public static double A0_req = 0;

    public AirbrakesControllerListener(){
        super();
        pastOmegaZ = new ArrayList<>();
        pastThetaZ = new ArrayList<>();
        finTabAngleLog = new ArrayList<>();
        rktVelMagLog = new ArrayList<>();
        rktAltLog = new ArrayList<>();
        CldLog = new ArrayList<>();
        CldArefDLog = new ArrayList<>();
        Qlog = new ArrayList<>();
        accelMeasurements = new ArrayList<>();
        accelDat = new AirbrakesAccelerationMeasurement(0.0, 0.0);

    }


    @Override
    public void startSimulation(SimulationStatus status) throws SimulationException {
        status.copySimStatParameters(initialStatus);
        theRocket = status.getConfiguration().getRocket();

        super.startSimulation(status);
        currentStatus = status.clone();

        theAirbrakes = getAirbrakes(status);
    }

    @Override
    public void postStep(SimulationStatus status) throws SimulationException {
        status.clone();
        double airbrakesTimeStep = status.getSimulationTime();
        latestTimeStep = airbrakesTimeStep - loopStart;

        double realVelocity = status.getRocketVelocity().length();

        pastOmegaZ.add(status.getRocketRotationVelocity().z);
        pastThetaZ.add(toDegrees(toEulerAngles_rocketCoord(status.getRocketOrientationQuaternion()).z));
        finTabAngleLog.add(getFinTabAngleDeg());
        rktVelMagLog.add(realVelocity);

        List<Double> CldFlightBranch = status.getFlightDataBranch().get(FlightDataType.TYPE_ROLL_DAMPING_COEFF);
        CldLog.add(CldFlightBranch.get(CldFlightBranch.toArray().length-1));
        CldArefDLog.add(status.getFlightDataBranch().get(FlightDataType.TYPE_ROLL_DAMPING_COEFF).get(CldFlightBranch.toArray().length-1)*status.getFlightConfiguration().getReferenceLength()*status.getFlightConfiguration().getReferenceArea());
        Qlog.add(status.getSimulationConditions().getAtmosphericModel().getConditions(status.getRocketWorldPosition().getAltitude()).getDensity()*realVelocity*realVelocity/2.0);


        double currentTime = status.getSimulationTime();
        System.out.println("[JAVA] current time " + currentTime + "                     \r");

        mass = getRocketMass(theRocket);
        rocketCd = getRocketCD(theRocket);
        aRef = status.getFlightConfiguration().getReferenceArea();
        rho = status.getSimulationConditions().getAtmosphericModel().getConditions(status.getRocketWorldPosition().getAltitude()).getDensity();




        super.postStep(status);

        int everyHowMany = 10;
        int nMeasurements = 5;

        // if state is disabled, check if should start prep
        if (state == DISABLED){
            state =  (shouldStartAirbrakesControlPrep(status)) ? PREP : DISABLED;
        }
        else if (state == PREP){
            if (counter % everyHowMany == 0) {
                List<Double> accelZ = status.getFlightDataBranch().get(FlightDataType.TYPE_ACCELERATION_Z);
                accelDat.setData(currentTime, accelZ.get(accelZ.size() - 1));
                accelMeasurements.add(accelDat);
            }
            counter++;
            if (counter > nMeasurements*everyHowMany) {
                state = PREPROCESS;
                counter = 0;
            }
        }
        else if (state == PREPROCESS){
            // calculate alpha and t_apog from linear fit to accel measurements
            // TODO implement preprocessing to estimate alpha and t_apog

            // calculate apogee height.
            double x0 = status.getRocketWorldPosition().getAltitude();
            double v0 = status.getRocketVelocity().z;
            predictedAlt = alpha*Math.pow(t_apog,3)/6.0 -
                    t_apog*t_apog/2*(g + alpha*t_apog) + v0*t_apog + x0;
            System.out.println("[JAVA] predicted apogee: " + predictedAlt + " m");
            double desiredAlt = Math.floor(predictedAlt/roundToHowMuch)*roundToHowMuch;
            System.out.println("[JAVA] desired apogee: " + desiredAlt + " m");
            desiredDeltaX = predictedAlt - desiredAlt;

            A0_req = reqDeployedAreaAirbrakes(currentTime+1, desiredDeltaX);
            airbrakesCtrlStartTime = currentTime + 1;
            if (A0_req < 0.2) {
                A0_req = reqDeployedAreaAirbrakes(currentTime + 5, desiredDeltaX);
                if (A0_req > 1.0) {
                    A0_req = reqDeployedAreaAirbrakes(currentTime+1, desiredDeltaX);
                }
                else {
                    airbrakesCtrlStartTime = currentTime + 5;
                }

            }
            else if (A0_req < 0.5) {
                A0_req = reqDeployedAreaAirbrakes(currentTime + 2, desiredDeltaX);
                if (A0_req > 1.0) {
                    A0_req = reqDeployedAreaAirbrakes(currentTime+1, desiredDeltaX);
                }
                else {
                    airbrakesCtrlStartTime = currentTime + 2;
                }
            }


            if (A0_req > 1.0) {
                System.out.println("[JAVA] Cannot reach desired apogee with airbrakes.");
                state = INFEASIBLE;
            }

            else {
                System.out.println("[JAVA] Reaching desired apogee is feasible.");
                System.out.println("[JAVA] Required Airbrakes Deployed Fraction: " + A0_req);
                System.out.println("[JAVA] Desired deploy time: " + airbrakesCtrlStartTime + " s");
                state = WAIT_FOR_START;
            }
        }

        else if (state == WAIT_FOR_START){
            if (status.getSimulationTime() >= airbrakesCtrlStartTime){
                state = CONTROLLING_RAMP;
            }
        }
        else if (state == CONTROLLING_RAMP){
            // implement airbrakes control ramp
            // A(t) = 2A_0(t-t_0) for t_0 <= t < t_0 + 0.5s
            if (status.getSimulationTime() >= airbrakesCtrlStartTime) {
                double deployedFraction = 2 * A0_req * (status.getSimulationTime() - airbrakesCtrlStartTime);
                theAirbrakes.setFracExposed(deployedFraction);
            }
            if (status.getSimulationTime() >= airbrakesCtrlStartTime + 0.5){
                state = CONTROLLING_PLATEAU;
            }
        }
        else if (state == CONTROLLING_PLATEAU){
            // implement airbrakes control plateau
            // A(t) = A_0 for t >= t_0 + 0.5s
            theAirbrakes.setFracExposed(A0_req);
            // check if apogee reached
            if (status.getRocketVelocity().z <= 0 || status.apogeeReached){
                state = DONE;
            }
        }






    }

    public static AirbrakeSet getAirbrakes(SimulationStatus status){
        ArrayList<AirbrakeSet> sets = new ArrayList<>();
        Rocket rocket = status.getConfiguration().getRocket();
        for(Iterator<RocketComponent> it = rocket.iterator(true); it.hasNext(); ){
            RocketComponent component = it.next();

            if(component instanceof AirbrakeSet){
                sets.add((AirbrakeSet) component);
            }
        }
        return sets.get(0);
    }

    /* computes A_0/A_max fraction given deploy time and desired delta x*/
    public static double reqDeployedAreaAirbrakes(double t_0, double deltaX){
        double B = (aRef * rho * rocketCd) / (2*mass);
        double eta = (2*mass*g*deltaX*(Math.pow(B, 1.5)))/(airbrakesCd*rho*Math.pow(alpha,1.5));
        double denom1 = (4/5)*(t_apog -t_0)*(Math.pow(t_apog -t_0, 2.5) - Math.pow(t_apog - t_0 - 0.5, 2.5));
        double denom2 = (4/7)*(Math.pow(t_apog -t_0, 3.5) - Math.pow(t_apog - t_0 - 0.5, 3.5));
        double denom3 = (2/5)*Math.pow(t_apog - t_0 - 0.5, 2.5);

        double a_0 = eta/(denom1 - denom2 + denom3);
        return a_0/a_max;
    }


    public static boolean shouldStartAirbrakesControlPrep(SimulationStatus status) {
        if (status.getSimulationTime() > 15.0 && !status.apogeeReached) {
            return true;
        }
        else return false;
    }


}
