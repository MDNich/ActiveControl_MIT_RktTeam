package edu.mit.rocket_team.control.airbrakes;

/**
 * Mutable acceleration/time sample used by the firmware coast fit. Acceleration is m/s² and the
 * supplied flight timestamp is seconds.
 */
public class RTAirbrakesAccelerationMeasurement {
    public float accelMeasurement;
    public float timeStamp;

    /**
     * Creates a RTAirbrakesAccelerationMeasurement with the supplied dependencies and per-instance
     * state.
     *
     * @param ts initial time stamp
     * @param am initial accel measurement
     */
    RTAirbrakesAccelerationMeasurement(float ts, float am) {
        accelMeasurement = am;
        timeStamp = ts;
    }

    /**
     * Returns the stored acceleration measurement in metres per second squared.
     *
     * @return the accel measurement
     */
    public float getAccelMeasurement() {
        return accelMeasurement;
    }

    /**
     * Returns the sample time supplied by the airbrake predictor.
     *
     * @return the sample time supplied by the airbrake predictor
     */
    public float getTimeStamp() {
        return timeStamp;
    }

    /**
     * Replaces the acceleration in metres per second squared and associated predictor timestamp.
     *
     * @param ts time stamp value
     * @param am accel measurement value
     */
    public void setData(float ts, float am) {
        timeStamp = ts;
        accelMeasurement = am;
    }
}
