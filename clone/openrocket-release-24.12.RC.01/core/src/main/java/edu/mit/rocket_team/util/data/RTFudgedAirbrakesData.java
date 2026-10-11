package edu.mit.rocket_team.util.data;

/**
 * Legacy predictor input bundle: altitude in metres, velocity in m/s, acceleration in m/s² and
 * apogee state.
 */
public class RTFudgedAirbrakesData extends RTFudgedData {
    public float altitude;
    public float vel_z;
    public float accel_z;
    public boolean apogeeReached;

    /**
     * Creates a RTFudgedAirbrakesData with the supplied dependencies and per-instance state.
     *
     * @param altitude initial altitude
     * @param vel_z initial vel z
     * @param accel_z initial accel z
     * @param apogeeReached initial apogee reached
     */
    public RTFudgedAirbrakesData(
            double altitude, double vel_z, double accel_z, boolean apogeeReached) {
        super();
        this.altitude = (float) altitude;
        this.vel_z = (float) vel_z;
        this.accel_z = (float) accel_z;
        this.apogeeReached = apogeeReached;
    }

    /**
     * Returns the altitude in metres.
     *
     * @return the altitude in metres
     */
    public float getAltitude() {
        return altitude;
    }

    /**
     * Returns the predictor input velocity in metres per second.
     *
     * @return the vel z
     */
    public float getVel_z() {
        return vel_z;
    }

    /**
     * Returns the predictor input acceleration in metres per second squared.
     *
     * @return the accel z
     */
    public float getAccel_z() {
        return accel_z;
    }

    /**
     * Reports whether apogee reached.
     *
     * @return true when the condition described above holds
     */
    public boolean isApogeeReached() {
        return apogeeReached;
    }

    /**
     * Replaces the apogee reached stored by this instance.
     *
     * @param apogeeReached apogee reached value
     */
    public void setApogeeReached(boolean apogeeReached) {
        this.apogeeReached = apogeeReached;
    }

    /**
     * Replaces the altitude stored by this instance.
     *
     * @param altitude altitude value
     */
    public void setAltitude(double altitude) {
        this.altitude = (float) altitude;
    }

    /**
     * Replaces the vel z stored by this instance.
     *
     * @param vel_z vel z value
     */
    public void setVel_z(double vel_z) {
        this.vel_z = (float) vel_z;
    }

    /**
     * Replaces the accel z stored by this instance.
     *
     * @param accel_z accel z value
     */
    public void setAccel_z(double accel_z) {
        this.accel_z = (float) accel_z;
    }
}
