package edu.mit.rocket_team.instrument;

import edu.mit.rocket_team.util.*;
import edu.mit.rocket_team.util.data.*;
import static edu.mit.rocket_team.util.RTUtilLibrary.elapsed32;

/** ADXL357.cpp: calibrated acquisition boundary; source X integration is intentional. */
public class RTAccel extends RTInstrument {
    private float x, y, z, velocity, vertical;
    private long lastUpdate;
    private boolean ready;
    private RTAccelData pending;

    /** Creates a RTAccel with the supplied dependencies and per-instance state. */
    public RTAccel() {
        super();
    }

    /**
     * Creates a RTAccel with the supplied dependencies and per-instance state.
     *
     * @param trace run-local virtual clock and action-log sink
     */
    public RTAccel(RTUtilLibrary.Trace trace) {
        super(trace);
    }

    /** Records the accelerometer units and longitudinal sensor-X convention. */
    @Override
    public void setup() {
        trace.log("accel.setup", "units=mps2 sensor_X=longitudinal");
    }

    /**
     * Stages a fresh engineering-unit accelerometer sample for the next update.
     *
     * @param data input data for this operation
     */
    @Override
    public void backdoorFudge(RTFudgedData data) {
        pending = (RTAccelData) data;
        ready = true;
    }

    /**
     * Consumes at most one fresh sample and integrates sensor-X acceleration minus gravity using
     * elapsed virtual microseconds; preflight integration follows the firmware threshold.
     *
     * @param state current flight phase or state
     */
    public void update(RTRocketState state) {
        if (!ready) {
            trace.log("accel.update", "new_data=false");
            return;
        }
        x = pending.getAccelX();
        y = pending.getAccelY();
        z = pending.getAccelZ();
        ready = false;
        vertical = (float) (x - 9.8065);
        long now = trace.micros(), dt = elapsed32(now, lastUpdate);
        if (state != RTRocketState.PRE_FLIGHT || vertical > 10)
            velocity += vertical * dt / 1000000.0;
        lastUpdate = now;
        trace.log(
                "accel.update",
                "new_data=true x="
                        + x
                        + " y="
                        + y
                        + " z="
                        + z
                        + " vertical="
                        + vertical
                        + " velocity="
                        + velocity
                        + " dt_us="
                        + dt);
    }

    /**
     * Consumes at most one fresh sample and integrates sensor-X acceleration minus gravity using
     * elapsed virtual microseconds; preflight integration follows the firmware threshold.
     */
    public void update() {
        update(RTRocketState.GROUND_TESTING);
    }

    /**
     * Returns the accel x in metres per second squared.
     *
     * @return the accel x in metres per second squared
     */
    public float getAccelX() {
        return x;
    }

    /**
     * Returns the accel y in metres per second squared.
     *
     * @return the accel y in metres per second squared
     */
    public float getAccelY() {
        return y;
    }

    /**
     * Returns the accel z in metres per second squared.
     *
     * @return the accel z in metres per second squared
     */
    public float getAccelZ() {
        return z;
    }

    /**
     * Returns sensor-X specific force minus 9.8065 m/s², as used by the firmware launch detector.
     *
     * @return sensor-X specific force minus 9.8065 m/s², as used by the firmware launch detector
     */
    public float getVerticalAccelMinusGravity() {
        return vertical;
    }

    /**
     * Returns the firmware longitudinal velocity estimate in metres per second, integrated from
     * measured acceleration.
     *
     * @return the firmware longitudinal velocity estimate in metres per second, integrated from
     *     measured acceleration
     */
    public float getIntegratedVelo() {
        return velocity;
    }

    /** Resets integrated longitudinal velocity to zero. */
    public void zeroIntegratedVelo() {
        velocity = 0;
        trace.log("accel.zero", "velocity=0");
    }

    /**
     * Sign-extends a 20-bit ADXL357 sample and applies the firmware scale and X-axis calibration to
     * obtain m/s².
     *
     * @param raw raw
     * @param xAxis x axis
     * @return computed numeric value
     */
    public static float calibratedRaw(int raw, boolean xAxis) {
        int signed = (raw << 12) >> 12;
        return (float) (signed / 12800.0 * 9.80665 * (xAxis ? 1.060 : 1.0));
    }
}
