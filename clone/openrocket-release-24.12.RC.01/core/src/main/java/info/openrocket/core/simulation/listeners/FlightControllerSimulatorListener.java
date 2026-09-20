package info.openrocket.core.simulation.listeners;
import edu.mit.rocket_team.zephyrus.FC.RTFC;
import edu.mit.rocket_team.zephyrus.util.RTSimulationCommunicator;
import edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.Trace;
import edu.mit.rocket_team.zephyrus.util.data.*;
import info.openrocket.core.models.atmosphere.AtmosphericConditions;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.exception.SimulationException;
import info.openrocket.core.util.*;
import java.util.Iterator;
import java.util.List;
import java.nio.file.Path;
import java.util.function.Consumer;
import static edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.convertToImuAngles;
import static java.lang.Math.*;

/** Existing FC listener, now driving the translated FC at a deterministic 100 Hz. */
public class FlightControllerSimulatorListener extends AbstractSimulationListener {
    // Compatibility-only fields for historical callers. Not used by the FC path below.
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




    private static final long WARMUP_US=1_000_000;
    private final Consumer<String> console;
    private final double maxPhysicsStep;
    private final boolean holdAirbrakesClosed;
    private Run run;
    /** Shared only across framework copies belonging to this flight, never across new starts. */
    private static final class Run {
        final RTFC fc;
        final RTSimulationCommunicator communicator;
        long nextTickUs=10_000,lastPublishedUs=-1,lastGpsUs=-100_000;
        double stepStart;
        RTFC.Inputs candidate,latest;
        boolean intervalOpen,closed;
        Run(Consumer<String> console) { fc=new RTFC(new Trace(console)); communicator=new RTSimulationCommunicator(fc.trace); }
    }
    public FlightControllerSimulatorListener() { this(line -> System.out.println(line),0.0025,false); }
    public FlightControllerSimulatorListener(Consumer<String> console,double maxPhysicsStep,boolean holdAirbrakesClosed) {
        if(!(maxPhysicsStep>0 && maxPhysicsStep<=0.0025)) throw new IllegalArgumentException("FC physics step must be in (0, 0.0025] seconds");
        this.console=console; this.maxPhysicsStep=maxPhysicsStep; this.holdAirbrakesClosed=holdAirbrakesClosed;
    }
    public RTFC getFlightComputer() { return run==null?null:run.fc; }
    public double getMaximumPhysicsStep() { return maxPhysicsStep; }
    @Override public void startSimulation(SimulationStatus status) throws SimulationException {
        run=new Run(console);
        if(status.getConfiguration().getActiveStageCount()!=1) throw new SimulationException("Zephyrus Java FC currently supports one active stage");
        if(status.getSimulationTime()!=0) throw new SimulationException("Zephyrus FC requires pad startup; mid-flight checkpoints are not implemented");
        run.communicator.bind(status);
        run.fc.trace.log("simulation.start", "rocket="+status.getConfiguration().getRocket().getName()+" max_step_s="+maxPhysicsStep+" mounting=X:bodyZ,Y:bodyX,Z:bodyY noise=off recovery=OpenRocket pyro=recorded_only roll_physics=off hold_closed="+holdAirbrakesClosed);
        run.fc.init();
        run.fc.telemetry.open(Path.of(System.getProperty("openrocket.fc.telemetryDir","fc-telemetry")));
        try {
            for(long us=0;us<WARMUP_US;us+=RTFC.LOOP_US) {
                if(us==500_000) run.fc.enqueueCommand(RTFC.stateCommand(edu.mit.rocket_team.zephyrus.util.RTRocketState.PRE_FLIGHT));
                RTFC.Inputs pad=sample(status,Coordinate.ZERO,us);
                tick(us,pad);
            }
            run.latest=sample(status,Coordinate.ZERO,WARMUP_US);
            tick(WARMUP_US,run.latest);
            run.communicator.apply(run.fc.getOutput(),holdAirbrakesClosed);
            run.fc.trace.log("simulation.release", "simulation_s=0 boot_us="+WARMUP_US);
        } catch(RuntimeException e) { finish(); throw e; }
    }
    private void tick(long bootUs,RTFC.Inputs input) {
        boolean gpsDue=bootUs-run.lastGpsUs>=100_000;
        if(gpsDue) run.lastGpsUs=bootUs;
        run.fc.pre_loop(bootUs,new RTFC.Inputs(input.acquisitionUs(),input.accel(),input.baro(),gpsDue?input.gps():null,input.gyro()));
        run.fc.loop();
        if(bootUs%RTFC.PWM_US==0) run.fc.latchPwm();
    }
    @Override public boolean preStep(SimulationStatus status) throws SimulationException {
        run.stepStart=status.getSimulationTime(); run.candidate=null; run.intervalOpen=true;
        run.communicator.bind(status);
        run.fc.trace.log("physics.begin", "simulation_s="+run.stepStart);
        return true;
    }
    @Override public AccelerationData postAccelerationCalculation(SimulationStatus status,AccelerationData acceleration) {
        if(run!=null && run.intervalOpen && run.candidate==null && abs(status.getSimulationTime()-run.stepStart)<1e-10) {
            run.candidate=sample(status,acceleration.getLinearAccelerationWC(),WARMUP_US+Math.round(status.getSimulationTime()*1_000_000));
        }
        return null; // Observe; never override the derivative.
    }
    @Override public void postStep(SimulationStatus status) throws SimulationException {
        run.intervalOpen=false;
        double time=status.getSimulationTime();
        if(time<=run.stepStart) { run.fc.trace.log("physics.no_advance", "simulation_s="+time); return; }
        long us=Math.round(time*1_000_000);
        if(us==run.lastPublishedUs) return;
        run.lastPublishedUs=us;
        if(run.candidate==null) {
            if(!status.isLanded()) throw new SimulationException("Missing coherent FC acceleration sample at "+run.stepStart);
            run.candidate=sample(status,Coordinate.ZERO,WARMUP_US+us);
        }
        run.latest=run.candidate;
        run.fc.trace.log("physics.accept", "simulation_s="+time+" sample_us="+run.latest.acquisitionUs()+" truth_altitude_m="+status.getRocketWorldPosition().getAltitude()+" truth_velocity_mps="+status.getRocketVelocity().z);
        if(us>run.nextTickUs) throw new SimulationException("FC deadline skipped: expected "+run.nextTickUs+" us, got "+us);
        if(us==run.nextTickUs) {
            tick(WARMUP_US+us,run.latest);
            run.nextTickUs+=RTFC.LOOP_US;
            run.communicator.apply(run.fc.getOutput(),holdAirbrakesClosed);
        }
    }
    /** Coherent engineering-unit observation; simulation body Z is the longitudinal axis. */
    public static RTFC.Inputs sample(SimulationStatus status,Coordinate accelerationWC,long acquisitionUs) {
        double gravity=status.getSimulationConditions().getGravityModel().getGravity(status.getRocketWorldPosition());
        Coordinate coriolis=status.getSimulationConditions().getGeodeticComputation().getCoriolisAcceleration(status.getRocketWorldPosition(),status.getRocketVelocity());
        Coordinate body=status.getRocketOrientationQuaternion().invRotate(accelerationWC.sub(coriolis).add(0,0,gravity));
        Coordinate omega=status.getRocketOrientationQuaternion().invRotate(status.getRocketRotationVelocity());
        AtmosphericConditions atmosphere=status.getSimulationConditions().getAtmosphericModel().getConditions(status.getRocketWorldPosition().getAltitude());
        WorldCoordinate location=status.getRocketWorldPosition();
        return new RTFC.Inputs(acquisitionUs,new RTAccelData(body.z,body.x,body.y),
            new RTBaroData(Float.NaN,Float.NaN,(float)(atmosphere.getTemperature()-273.15),Float.NaN,(float)(atmosphere.getPressure()/100.0)),
            new RTGPSData(location.getLatitudeDeg(),location.getLongitudeDeg(),location.getAltitude(),0.0,0.0,0.0,true),
            new RTGyroData(toDegrees(omega.z),toDegrees(omega.x),toDegrees(omega.y)));
    }
    public static FlightControllerSimulatorListener active(SimulationStatus status) {
        FlightControllerSimulatorListener found=null;
        for(SimulationListener l:status.getSimulationConditions().getSimulationListenerList()) if(l instanceof FlightControllerSimulatorListener fc) {
            if(found!=null) throw new IllegalArgumentException("Register only one FC listener per simulation"); found=fc;
        }
        return found;
    }
    /** Used by the existing engine/steppers only when this listener is active. */
    public double limitStep(SimulationStatus status,double eventLimit) {
        double untilTick=run.nextTickUs/1_000_000.0-status.getSimulationTime();
        if(!(untilTick>0)) throw new IllegalStateException("FC tick must execute before another physics interval");
        return Math.min(Math.min(eventLimit,maxPhysicsStep),untilTick);
    }
    @Override public void endSimulation(SimulationStatus status,SimulationException exception) { finish(); }
    public void finish() {
        if(run!=null && !run.closed) { run.closed=true; run.fc.trace.log("simulation.end", "loops="+run.fc.getLoopCount()); run.fc.telemetry.close(); }
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


    public static AirbrakeSet getAirbrakes(SimulationStatus status){
        java.util.ArrayList<AirbrakeSet> sets = new java.util.ArrayList<>();
        Rocket rocket = status.getConfiguration().getRocket();
        for(Iterator<RocketComponent> it = rocket.iterator(true); it.hasNext(); ){
            RocketComponent component = it.next();

            if(component instanceof AirbrakeSet){
                sets.add((AirbrakeSet) component);
            }
        }
        return sets.get(0);
    }
}
