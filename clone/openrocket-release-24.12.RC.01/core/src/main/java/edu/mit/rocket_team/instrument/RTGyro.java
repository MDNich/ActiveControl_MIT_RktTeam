package edu.mit.rocket_team.instrument;

import edu.mit.rocket_team.util.*;
import edu.mit.rocket_team.util.data.*;
import static edu.mit.rocket_team.util.RTUtilLibrary.elapsed32;

/** gyro.cpp: source negative-X roll and micros-based integration. */
public class RTGyro extends RTInstrument {
    private float x, y, z, roll, pitch, yaw, angleFromVertical;
    private long lastUpdate;

    /** Creates a RTGyro with the supplied dependencies and per-instance state. */
    public RTGyro() {
        super();
    }

    /**
     * Creates a RTGyro with the supplied dependencies and per-instance state.
     *
     * @param trace run-local virtual clock and action-log sink
     */
    public RTGyro(RTUtilLibrary.Trace trace) {
        super(trace);
    }

    /** Records the degrees-per-second input convention and negative-X roll sign. */
    @Override
    public void setup() {
        trace.log("gyro.setup", "rates=dps roll_sign=negative_X");
    }

    /**
     * Loads a held three-axis angular-rate sample in degrees per second.
     *
     * @param data input data for this operation
     */
    @Override
    public void backdoorFudge(RTFudgedData data) {
        RTGyroData d = (RTGyroData) data;
        x = d.getGyroX();
        y = d.getGyroY();
        z = d.getGyroZ();
    }

    /**
     * Integrates held angular rates with elapsed virtual microseconds and computes inclination;
     * roll uses negative sensor X.
     */
    public void update() {
        long now = trace.micros(), dt = elapsed32(now, lastUpdate);
        roll += x * -1.0 * dt / 1000000.0;
        pitch += y * dt / 1000000.0;
        yaw += z * dt / 1000000.0;
        angleFromVertical =
                (float)
                        (Math.acos(
                                        Math.cos(pitch * Math.PI / 180.0)
                                                * Math.cos(yaw * Math.PI / 180))
                                * 180.0
                                / Math.PI);
        lastUpdate = now;
        trace.log(
                "gyro.update",
                "x_dps="
                        + x
                        + " y_dps="
                        + y
                        + " z_dps="
                        + z
                        + " roll_deg="
                        + roll
                        + " pitch_deg="
                        + pitch
                        + " yaw_deg="
                        + yaw
                        + " dt_us="
                        + dt);
    }

    /** Zeros the three integrated angles without changing held rates. */
    public void zeroRollPitchYaw() {
        roll = pitch = yaw = 0;
        trace.log("gyro.zero", "roll_pitch_yaw=0");
    }

    /**
     * Returns the roll in degrees.
     *
     * @return the roll in degrees
     */
    public float getRoll() {
        return roll;
    }

    /**
     * Returns the pitch in degrees.
     *
     * @return the pitch in degrees
     */
    public float getPitch() {
        return pitch;
    }

    /**
     * Returns the yaw in degrees.
     *
     * @return the yaw in degrees
     */
    public float getYaw() {
        return yaw;
    }

    /**
     * Returns negative sensor-X angular rate in degrees per second.
     *
     * @return negative sensor-X angular rate in degrees per second
     */
    public float getRollRate() {
        return -x;
    }

    /**
     * Returns the inclination inferred from integrated pitch and yaw, in degrees.
     *
     * @return the inclination inferred from integrated pitch and yaw, in degrees
     */
    public float getAngleFromVertical() {
        return angleFromVertical;
    }

    /**
     * Returns the gyro x in degrees per second.
     *
     * @return the gyro x in degrees per second
     */
    public float getGyroX() {
        return x;
    }

    /**
     * Returns the gyro y in degrees per second.
     *
     * @return the gyro y in degrees per second
     */
    public float getGyroY() {
        return y;
    }

    /**
     * Returns the gyro z in degrees per second.
     *
     * @return the gyro z in degrees per second
     */
    public float getGyroZ() {
        return z;
    }

    /**
     * Applies the firmware raw-count scale and fixed gyro offsets to produce degrees per second.
     *
     * @param x X component or horizontal coordinate
     * @param y Y component or vertical coordinate
     * @param z Z component
     * @return the resulting RTGyroData
     */
    public static RTGyroData fromRaw(short x, short y, short z) {
        return new RTGyroData(
                (float) (x * 0.03051757812 - 0.2525f),
                (float) (y * 0.03051757812 - 0.2441f),
                (float) (z * 0.03051757812 - 0.4376f));
    }
}
