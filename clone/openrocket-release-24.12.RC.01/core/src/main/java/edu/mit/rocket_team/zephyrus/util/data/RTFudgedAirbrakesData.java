package edu.mit.rocket_team.zephyrus.util.data;

public class RTFudgedAirbrakesData extends RTFudgedData {
    public float altitude;
    public float vel_z;
    public float accel_z;
    public boolean apogeeReached;


    public RTFudgedAirbrakesData(double altitude, double vel_z, double accel_z, boolean apogeeReached) {
        super();
        this.altitude = (float) altitude;
        this.vel_z = (float) vel_z;
        this.accel_z = (float) accel_z;
        this.apogeeReached = apogeeReached;
    }

    public float getAltitude() {
        return altitude;
    }
    public float getVel_z() {
        return vel_z;
    }
    public float getAccel_z() {
        return accel_z;
    }
    public boolean isApogeeReached() {
        return apogeeReached;
    }

    public void setApogeeReached(boolean apogeeReached) {
        this.apogeeReached = apogeeReached;
    }
    public void setAltitude(double altitude) {
        this.altitude = (float) altitude;
    }
    public void setVel_z(double vel_z) {
        this.vel_z = (float) vel_z;
    }
    public void setAccel_z(double accel_z) {
        this.accel_z = (float) accel_z;
    }


}
