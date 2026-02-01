package edu.mit.rocket_team.zephyrus.util.data;

public class RTFudgedAirbrakesData extends RTFudgedData {
    public double altitude;
    public double vel_z;
    public double accel_z;
    public boolean apogeeReached;


    public RTFudgedAirbrakesData(double altitude, double vel_z, double accel_z, boolean apogeeReached) {
        super();
        this.altitude = altitude;
        this.vel_z = vel_z;
        this.accel_z = accel_z;
        this.apogeeReached = apogeeReached;
    }

    public double getAltitude() {
        return altitude;
    }
    public double getVel_z() {
        return vel_z;
    }
    public double getAccel_z() {
        return accel_z;
    }
    public boolean isApogeeReached() {
        return apogeeReached;
    }

    public void setApogeeReached(boolean apogeeReached) {
        this.apogeeReached = apogeeReached;
    }
    public void setAltitude(double altitude) {
        this.altitude = altitude;
    }
    public void setVel_z(double vel_z) {
        this.vel_z = vel_z;
    }
    public void setAccel_z(double accel_z) {
        this.accel_z = accel_z;
    }


}
