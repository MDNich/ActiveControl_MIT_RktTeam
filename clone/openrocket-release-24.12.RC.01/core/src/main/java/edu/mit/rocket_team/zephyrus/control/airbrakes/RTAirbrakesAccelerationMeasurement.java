package edu.mit.rocket_team.zephyrus.control.airbrakes;

public class RTAirbrakesAccelerationMeasurement {
    public double accelMeasurement;
    public double timeStamp;

    RTAirbrakesAccelerationMeasurement(double ts, double am){
        accelMeasurement = am;
        timeStamp = ts;
    }

    public double getAccelMeasurement(){
        return accelMeasurement;
    }

    public double getTimeStamp(){
        return timeStamp;
    }

    public void setData(double ts, double am){
        timeStamp = ts;
        accelMeasurement = am;
    }
}
