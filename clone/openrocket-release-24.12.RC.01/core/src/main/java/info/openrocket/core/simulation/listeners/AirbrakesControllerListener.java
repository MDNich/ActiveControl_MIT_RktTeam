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

import static info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener.*;
import static java.lang.Math.toDegrees;

public class AirbrakesControllerListener extends AbstractSimulationListener{
    public static SimulationStatus currentStatus;
    public static SimulationStatus initialStatus;
    public static AirbrakeSet theAirbrakes;
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
    public static ArrayList<AirbrakesVelocityMeasurement> velocityMeasurements;
    public static AirbrakesControllerState state = AirbrakesControllerState.DISABLED;
    public static boolean shouldPrep = false;
    public static AirbrakesVelocityMeasurement velDat;

    public static double mass = 53.48;
    public static double g = 9.81;
    public static double rho = 0.736;
    public static double airbrakesCd = 1.28;
    public static double rocketCd = 0.5859;
    public static double aRef = 0.01929;
    public static double alpha = 11.12;
    public static double B = (aRef * rho * rocketCd) / (2*mass);
    public static double t_ap = 31.89; //time of apogee
    public static double a_max = 0.0066; //maximum airbrakes area


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
        velocityMeasurements = new ArrayList<>();
        velDat = new AirbrakesVelocityMeasurement(0.0, 0.0);

    }


    @Override
    public void startSimulation(SimulationStatus status) throws SimulationException {
        status.copySimStatParameters(initialStatus);

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

        super.postStep(status);

        // if state is disabled, check if should start prep
        if (state == AirbrakesControllerState.DISABLED){
            state =  (shouldStartAirbrakesControlPrep(status)) ? AirbrakesControllerState.PREP : AirbrakesControllerState.DISABLED;
        }
        else if (state == AirbrakesControllerState.PREP){
             velDat.setData(currentTime, status.getRocketVelocity().length());
             velocityMeasurements.add(velDat);
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
    public static double  percentArea(double t_0, double deltaX){
        double eta = (2*mass*g*deltaX*(Math.pow(B, 1.5)))/(airbrakesCd*rho*Math.pow(alpha,1.5));
        double denom1 = (4/5)*(t_ap-t_0)*(Math.pow(t_ap-t_0, 2.5) - Math.pow(t_ap - t_0 - 0.5, 2.5));
        double denom2 = (4/7)*(Math.pow(t_ap-t_0, 3.5) - Math.pow(t_ap - t_0 - 0.5, 3.5));
        double denom3 = (2/5)*Math.pow(t_ap - t_0 - 0.5, 2.5);

        double a_0 = eta/(denom1 - denom2 + denom3);
        return a_0/a_max;
    }


    public static boolean shouldStartAirbrakesControl(SimulationStatus status) {
        return false;
    }

    public static boolean shouldStartAirbrakesControlPrep(SimulationStatus status) {
        return false;
    }


}
