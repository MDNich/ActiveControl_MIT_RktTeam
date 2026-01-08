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
            shouldPrep = shouldStartAirbrakesControlPrep(status);
            if (shouldPrep) {
                state = AirbrakesControllerState.PREP;
            }
        }
        else if (state == AirbrakesControllerState.PREP){
             velDat.setData(currentTime, status.getRocketVelocity().length());
             velocityMeasurements.add(velDat);
        }
        //

        //status.getRocketVelocity().length();



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


    public static boolean shouldStartAirbrakesControl(SimulationStatus status) {
        return false;
    }

    public static boolean shouldStartAirbrakesControlPrep(SimulationStatus status) {
        return false;
    }


}
