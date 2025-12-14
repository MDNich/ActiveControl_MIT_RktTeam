package edu.mit.rocket_team.zephyrus.FC;

import edu.mit.rocket_team.zephyrus.instrument.*;
import edu.mit.rocket_team.zephyrus.util.RTInstrument;
import edu.mit.rocket_team.zephyrus.util.data.*;
import info.openrocket.core.models.atmosphere.AtmosphericConditions;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.util.Coordinate;

import java.util.List;

import static edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.convertToImuAngles;

// Flight computer code goes here.
public class RTFC {

    public static SimulationStatus currentFCsimStat;

    static RTAccel accel;
    static RTBaro baro;
    static RTGPS gps;
    static RTGyro gyro;
    static RTMag mag;

    public static void init() {
        // FC code from C.
        accel = new RTAccel();
        baro = new RTBaro();
        gps = new RTGPS();
        gyro = new RTGyro();
        mag = new RTMag();

        RTInstrument[] sensors = {accel, baro, gps, gyro, mag};
        for (RTInstrument sensor: sensors) {
            sensor.setup();
        }
        // other tasks from FC.
    }

    public static void pre_loop(SimulationStatus fudgedStat) {
        currentFCsimStat = fudgedStat.clone();

        List<Double> accelZ = currentFCsimStat.getFlightDataBranch().get(FlightDataType.TYPE_ACCELERATION_Z);
        List<Double> accelXY = currentFCsimStat.getFlightDataBranch().get(FlightDataType.TYPE_ACCELERATION_XY);

        // mag
        Coordinate worldAngle = (Coordinate) currentFCsimStat.getExtraData("fudged_world_angle");
        // gyro
        Coordinate worldAngleRate = (Coordinate) currentFCsimStat.getExtraData("fudged_world_angle_rate");

        // pressure
        double alt = currentFCsimStat.getRocketWorldPosition().getAltitude();
        AtmosphericConditions atmos = currentFCsimStat.getSimulationConditions().
                getAtmosphericModel().
                getConditions(alt);

        RTAccelData accelDat = new RTAccelData(
                accelXY.get(accelXY.size() - 1),
                accelXY.get(accelXY.size() - 1),
                accelZ.get(accelZ.size() - 1));
        RTBaroData baroDat = new RTBaroData(
                atmos.getPressure(),
                atmos.getTemperature(),
                atmos.getTemperature(),
                alt, // fudged already, intended for barometer
                atmos.getPressure());
        RTGPSData gpsDat = new RTGPSData(
                currentFCsimStat.getRocketWorldPosition().getLatitudeDeg(),
                currentFCsimStat.getRocketWorldPosition().getLongitudeDeg(),
                (Double) currentFCsimStat.getExtraData("fudged_gps_altitude"),
                1.0,1.0,1.0,(Boolean) currentFCsimStat.getExtraData("fudged_gps_has_fix"));
        RTGyroData gyroDat = new RTGyroData(
                worldAngleRate.x,
                worldAngleRate.y,
                worldAngleRate.z
        );
        RTMagData magDat = new RTMagData(
                worldAngle.x,
                worldAngle.y,
                worldAngle.z
        );

        RTInstrument[] sensors = {accel, baro, gps, gyro, mag};
        RTFudgedData[] data = {accelDat, baroDat, gpsDat, gyroDat, magDat};
        for (int i = 0; i < sensors.length; i++) {
            sensors[i].backdoorFudge(data[i]);
        }
    }

    public static void loop() {
        // FC code from C
    }
}
