package edu.mit.rocket_team.zephyrus.FC;

import edu.mit.rocket_team.zephyrus.control.airbrakes.RTAirbrakesController;
import edu.mit.rocket_team.zephyrus.control.RTPyroController;
import edu.mit.rocket_team.zephyrus.control.RTRollController;
import edu.mit.rocket_team.zephyrus.instrument.*;
import edu.mit.rocket_team.zephyrus.internal.RTFlashDriver;
import edu.mit.rocket_team.zephyrus.internal.RTPowerBoard;
import edu.mit.rocket_team.zephyrus.util.RTController;
import edu.mit.rocket_team.zephyrus.util.RTInstrument;
import edu.mit.rocket_team.zephyrus.util.data.*;
import info.openrocket.core.models.atmosphere.AtmosphericConditions;
import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.WorldCoordinate;

// Flight computer code goes here.
public class RTFC {

    public static SimulationStatus currentFCsimStat;

    static RTAccel accel;
    static RTBaro baro;
    static RTGPS gps;
    static RTGyro gyro;
    static RTMag mag;

    static RTRollController rollController;
    static RTAirbrakesController airbrakesController;
    static RTPyroController pyroController;
    static RTController[] controllers;

    static RTPowerBoard powerBoard;
    static RTFlashDriver flashDriver;


    public static void init() {
        // FC code from C.
        accel = new RTAccel();
        baro = new RTBaro();
        gps = new RTGPS();
        gyro = new RTGyro();
        mag = new RTMag();

        powerBoard = new RTPowerBoard();

        rollController = new RTRollController();
        airbrakesController = new RTAirbrakesController();
        pyroController = new RTPyroController();
        controllers = new RTController[]{rollController, airbrakesController, pyroController};


        RTInstrument[] sensors = {accel, baro, gps, gyro, mag};
        for (RTInstrument sensor: sensors) {
            sensor.setup();
        }
        // other tasks from FC.
    }

    public static void pre_loop(SimulationStatus fudgedStat) {
        currentFCsimStat = fudgedStat.clone();

        // accel
        Coordinate measuredAccel = (Coordinate) currentFCsimStat.getExtraData("fudged_accel");
        // mag
        Coordinate worldAngle = (Coordinate) currentFCsimStat.getExtraData("fudged_world_angle");
        // gyro
        Coordinate worldAngleRate = (Coordinate) currentFCsimStat.getExtraData("fudged_world_angle_rate");

        // baro
        double alt = currentFCsimStat.getRocketWorldPosition().getAltitude();
        AtmosphericConditions atmos = currentFCsimStat.getSimulationConditions().
                getAtmosphericModel().
                getConditions(alt);

        // GPS
        WorldCoordinate pos = (WorldCoordinate) currentFCsimStat.getExtraData("fudged_gps_position");
        boolean hasFix = (Boolean) currentFCsimStat.getExtraData("fudged_gps_has_fix");

        RTAccelData accelDat = new RTAccelData(
                measuredAccel.x,
                measuredAccel.y,
                measuredAccel.z
        );
        RTBaroData baroDat = new RTBaroData(
                (Double) currentFCsimStat.getExtraData("fudged_rawPressure"),
                (Double) currentFCsimStat.getExtraData("fudged_rawTemperature"),
                (Double) currentFCsimStat.getExtraData("fudged_temperature"),
                (Double) currentFCsimStat.getExtraData("fudged_altitude"),
                (Double) currentFCsimStat.getExtraData("fudged_pressure")
        );
        RTGPSData gpsDat = new RTGPSData(
                pos.getLatitudeDeg(),
                pos.getLongitudeDeg(),
                pos.getAltitude(),
                1.0, 1.0, 1.0,
                hasFix
        );
        RTGyroData gyroDat = new RTGyroData(
                worldAngleRate.x,
                worldAngleRate.y,
                worldAngleRate.z
        );
        RTMagData magDat = new RTMagData(
                0.00001, // mimic small magnetic field values
                Math.tan(worldAngle.z) * 0.00001,
                0.00001,
                worldAngle.z
        );

        RTInstrument[] sensors = {accel, baro, gps, gyro, mag};
        RTFudgedData[] data = {accelDat, baroDat, gpsDat, gyroDat, magDat};
        for (int i = 0; i < sensors.length; i++) {
            sensors[i].backdoorFudge(data[i]);
        }


        RTFudgedAirbrakesData fudgedAirbrakesData = new RTFudgedAirbrakesData(
                alt,
                currentFCsimStat.getRocketVelocity().z,
                measuredAccel.z,
                currentFCsimStat.apogeeReached
        );

        RTFudgedData[] controllerData = {fudgedAirbrakesData};
        RTController[] controllers = {airbrakesController};
        for (int i = 0; i < sensors.length; i++) {
            controllers[i].backdoorFudge(controllerData[i]);
        }
    }

    public static void loop() {
        // FC code from C

        for (RTController controller: controllers) {
            controller.performLoopAction();
        }
    }
}
