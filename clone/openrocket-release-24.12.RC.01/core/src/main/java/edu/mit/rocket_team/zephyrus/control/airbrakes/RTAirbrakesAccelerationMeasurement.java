package edu.mit.rocket_team.zephyrus.control.airbrakes;

public class RTAirbrakesAccelerationMeasurement {
    public float accelMeasurement;
    public float timeStamp;

    RTAirbrakesAccelerationMeasurement(float ts, float am){
        accelMeasurement = am;
        timeStamp = ts;
    }

    public float getAccelMeasurement(){
        return accelMeasurement;
    }

    public float getTimeStamp(){
        return timeStamp;
    }

    public void setData(float ts, float am){
        timeStamp = ts;
        accelMeasurement = am;
    }
}
