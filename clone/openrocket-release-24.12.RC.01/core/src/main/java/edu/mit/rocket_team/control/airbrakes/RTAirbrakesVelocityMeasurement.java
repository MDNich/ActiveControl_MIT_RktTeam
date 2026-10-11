package edu.mit.rocket_team.control.airbrakes;

/**
 * Mutable velocity/time sample used by the firmware coast fit. Velocity is m/s and the supplied
 * flight timestamp is seconds.
 */
public class RTAirbrakesVelocityMeasurement {
    public float velocityMeasurement;
    public float timeStamp;

    /**
     * Creates a RTAirbrakesVelocityMeasurement with the supplied dependencies and per-instance
     * state.
     *
     * @param ts initial time stamp
     * @param vm initial velocity measurement
     */
    RTAirbrakesVelocityMeasurement(float ts, float vm) {
        velocityMeasurement = vm;
        timeStamp = ts;
    }

    /**
     * Returns the stored velocity in metres per second.
     *
     * @return the stored velocity in metres per second
     */
    public float getVelocityMeasurement() {
        return velocityMeasurement;
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
     * Replaces the velocity in metres per second and associated predictor timestamp.
     *
     * @param ts time stamp value
     * @param vm velocity measurement value
     */
    public void setData(float ts, float vm) {
        timeStamp = ts;
        velocityMeasurement = vm;
    }
}
