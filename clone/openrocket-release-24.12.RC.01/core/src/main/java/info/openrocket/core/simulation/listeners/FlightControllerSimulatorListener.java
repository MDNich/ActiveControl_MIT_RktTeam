package info.openrocket.core.simulation.listeners;

import edu.mit.rocket_team.zephyrus.FC.RTFC;
import info.openrocket.core.models.atmosphere.AtmosphericConditions;
import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.rocketcomponent.TabControlledTrapezoidFinSet;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.MotorClusterState;
import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.simulation.exception.SimulationException;
import info.openrocket.core.util.*;

import java.util.Iterator;
import java.util.List;

import static edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.convertToImuAngles;
import static java.lang.Math.*;

/**
 * Simulation listener that emulates flight computer control.
 */
public class FlightControllerSimulatorListener extends AbstractSimulationListener {


    public static SimulationStatus initialStatus = null;
    public static SimulationStatus latestStatus = null;
    public static double latestTimeStep = -1;

    public static TabControlledTrapezoidFinSet theFinsToModify = null;

    public static double overrideCNA = 0;
    public static double TIME_DELAY_MOTOR = 0;


    public static ArrayList<Double> pastOmegaZ;
    public static ArrayList<Double> pastThetaZ;
    public static ArrayList<Double> finTabAngleLog;
    public static ArrayList<Double> rktVelMagLog;
    public static ArrayList<Double> rktAltLog;
    public static ArrayList<Double> CldLog;
    public static ArrayList<Double> Qlog;
    public static ArrayList<Double> CldArefDLog;

    public static SimulationStatus lastStat = null;



    // Randomness controls
    public static ArrayList<Double> altitudeMeasuredList = null;
    public static double amplitude_randomness_size = 5;
    public static boolean ABORT_AT_APOGEE = false;



    // Simulation loop timing
    public static double loopStart = 0;


    // Simulation of servo lag
    public static double servoStepCount = 4097.0; // number of discrete steps the servo can make
    public static final double servoRangeAngleDeg = 120.0; // total range of motion of the servo
    public static double SERVO_REFRESH_TIME = 2e-10; // seconds
    public static double lastServoCommandTimestamp = 0;
    public static boolean flagPrintDebugMsg = false;
    public static boolean roundToNearest5 = true;


    public FlightControllerSimulatorListener() {
        super();
        pastOmegaZ = new ArrayList<>();
        pastThetaZ = new ArrayList<>();
        finTabAngleLog = new ArrayList<>();
        rktVelMagLog = new ArrayList<>();
        rktAltLog = new ArrayList<>();
        altitudeMeasuredList = new ArrayList<>();
        CldLog = new ArrayList<>();
        CldArefDLog = new ArrayList<>();
        Qlog = new ArrayList<>();
    }

    @Override
    public void startSimulation(SimulationStatus status) throws SimulationException {
        status.copySimStatParameters(initialStatus);
        if (TIME_DELAY_MOTOR > 0) {
            ((MotorClusterState) status.getActiveMotors().toArray()[0]).ignite(status.getSimulationTime()-TIME_DELAY_MOTOR); // 3.5 seconds before
        }
        if (TIME_DELAY_MOTOR > 100) {
            ((MotorClusterState) status.getActiveMotors().toArray()[0]).burnOut(status.getSimulationTime()-TIME_DELAY_MOTOR); // 3.5 seconds before
        }

        super.startSimulation(status);
        lastStat = status.clone();

        theFinsToModify = getTheFinsToModifyTabs(status);
        if (overrideCNA > 0) {
            theFinsToModify.setCNALPHA(overrideCNA);
        }

        RTFC.init();
    }

    @Override
    public boolean preStep(SimulationStatus status) throws SimulationException {
        loopStart = status.getSimulationTime();
        lastStat = status.clone();
        return super.preStep(status); // true
    }

    @Override
    public void postStep(SimulationStatus status) throws SimulationException {
        latestStatus = status.clone();
        double finTimeStep = status.getSimulationTime();
        latestTimeStep = finTimeStep - loopStart;

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


        // fudging
        SimulationStatus fudgedStatus = status.clone();
        fudgeSimulationStatus(fudgedStatus);

        RTFC.pre_loop(fudgedStatus.clone());
        RTFC.loop();

        if (ABORT_AT_APOGEE) {
            if (status.apogeeReached) {
                throw new SimulationException("Apogee => done");
            }
        }
    }


    // don't worry about it
    public static TabControlledTrapezoidFinSet getTheFinsToModifyTabs(SimulationStatus status) {
        ArrayList<TabControlledTrapezoidFinSet> finSets = new ArrayList<>();
        Rocket rocket = status.getConfiguration().getRocket();
        for (Iterator<RocketComponent> it = rocket.iterator(true); it.hasNext(); ) {
            RocketComponent component = it.next();

            if (component instanceof TabControlledTrapezoidFinSet) {
                finSets.add((TabControlledTrapezoidFinSet) component);
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
        theFinsToModify.setTabAngle(Math.PI/180*numStepsFromZero*stepSize);
        lastServoCommandTimestamp = latestStatus.getSimulationTime();
        if (flagPrintDebugMsg) {
            System.out.println("[JAVA] Actuated a servo change to " + newAngle + " degrees, which is " + numStepsFromZero + " steps.");
        }
    }

    public static void setFinTabAngleLowlevel(double newAngle) {
        if (newAngle > 10) {
            newAngle = 10;
        }
        if (newAngle < -10) {
            newAngle = -10;
        }
        theFinsToModify.setTabAngle(Math.PI/180*newAngle);
    }
    public static double getFinTabAngleDeg() {
        return theFinsToModify.getTabAngle()*180/PI ;
    }


    // Master Fudger

    public static void fudgeSimulationStatus(SimulationStatus fudgedStatus) {
        // accel
        List<Double> accelZ = fudgedStatus.getFlightDataBranch().get(FlightDataType.TYPE_ACCELERATION_Z);
        List<Double> accelXY = fudgedStatus.getFlightDataBranch().get(FlightDataType.TYPE_ACCELERATION_XY);

        Coordinate realAccel = new Coordinate(
                accelXY.get(accelXY.size() - 1),
                accelXY.get(accelXY.size() - 1),
                accelZ.get(accelZ.size() - 1));
        Coordinate fudgedAccel = fudgeAccel(realAccel);
        fudgedStatus.putExtraData("fudged_accel", fudgedAccel);

        // baro

        double alt = fudgedStatus.getRocketWorldPosition().getAltitude();
        AtmosphericConditions atmos = fudgedStatus.getSimulationConditions().
                getAtmosphericModel().
                getConditions(alt);

        double rP0 = atmos.getPressure();
        double rT0 = atmos.getTemperature();
        double t0 = atmos.getTemperature();
        double a0 = alt;
        double p0 = atmos.getPressure();

        double rP1 = fudgeRawPressure(rP0);
        double rT1 = fudgeRawTemperature(rT0);
        double t1 = fudgeTemperature(t0);
        double a1 = fudgeAltitude(alt);
        double p1 = fudgePressure(p0);

        fudgedStatus.putExtraData("fudged_rawPressure", rP1);
        fudgedStatus.putExtraData("fudged_rawTemperature", rT1);
        fudgedStatus.putExtraData("fudged_temperature", t1);
        fudgedStatus.putExtraData("fudged_altitude", a1);
        fudgedStatus.putExtraData("fudged_pressure", p1);

        // GPS
        WorldCoordinate realLocation = fudgedStatus.getRocketWorldPosition();
        WorldCoordinate fudgedLocation = fudgeGPS(realLocation);
        boolean gps_has_fix = true; // possibly fudge this too.
        fudgedStatus.putExtraData("fudged_gps_has_fix", gps_has_fix);
        fudgedStatus.putExtraData("fudged_gps_position", fudgedLocation);

        // Fudge the orientation

        List<Double> rollAngles = fudgedStatus.getFlightDataBranch().get(FlightDataType.TYPE_ORIENTATION_PHI);
        List<Double> pitchAngles = fudgedStatus.getFlightDataBranch().get(FlightDataType.TYPE_ORIENTATION_THETA);

        double rollAngle = rollAngles.get(rollAngles.size() - 1);
        double pitchAngle = pitchAngles.get(pitchAngles.size() - 1);
        Coordinate worldAngle = convertToImuAngles(rollAngle, pitchAngle);
        Coordinate fudgedWorldAngle = fudgeWorldAngle(worldAngle);
        fudgedStatus.putExtraData("fudged_world_angle", fudgedWorldAngle);

        List<Double> rollRates = fudgedStatus.getFlightDataBranch().get(FlightDataType.TYPE_ROLL_RATE);
        List<Double> pitchRates = fudgedStatus.getFlightDataBranch().get(FlightDataType.TYPE_PITCH_RATE);

        double rollRate = rollRates.get(rollRates.size() - 1);
        double pitchRate = pitchRates.get(pitchRates.size() - 1);
        Coordinate worldAngRate = convertToImuAngles(rollRate, pitchRate);
        Coordinate fudgedWorldAngleRate = fudgeWorldAngleRate(worldAngRate);
        fudgedStatus.putExtraData("fudged_world_angle_rate", fudgedWorldAngleRate);

        // no return, its a shared reference
    }





    // Fudgers per sensor
    // Accel
    public static Coordinate fudgeAccel(Coordinate accel) {
        // for the moment no-op.
        return accel;
    }

    // Baro
    public static double fudgeAltitude(double altitude) {
        // fudge with ± amplitude_randomness_size meters
        return altitude + (0.5-random())*2*amplitude_randomness_size;
    }
    public static double fudgeRawPressure(double rP0) {
        // for the moment no-op.
        return rP0;
    }
    public static double fudgeRawTemperature(double rT0) {
        // for the moment no-op.
        return rT0;
    }
    public static double fudgeTemperature(double t0) {
        // for the moment no-op.
        return t0;
    }
    public static double fudgePressure(double p0) {
        // for the moment no-op.
        return p0;
    }

    // GPS
    public static WorldCoordinate fudgeGPS(WorldCoordinate location) {
        // for the moment no-op.
        return new WorldCoordinate(
                location.getLatitudeRad(),
                location.getLongitudeRad(),
                location.getAltitude() // GPS-specific altitude fudging, not shared with baro.
        );
    }

    // Mag / IMU
    public static Coordinate fudgeWorldAngle(Coordinate worldAngle) {
        // for the moment no-op.
        return worldAngle;
    }

    // Gyro
    public static Coordinate fudgeWorldAngleRate(Coordinate worldAngleRate) {
        // for the moment no-op.
        return worldAngleRate;
    }






    public static Coordinate toEulerAngles_rocketCoord(Quaternion q) {

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
