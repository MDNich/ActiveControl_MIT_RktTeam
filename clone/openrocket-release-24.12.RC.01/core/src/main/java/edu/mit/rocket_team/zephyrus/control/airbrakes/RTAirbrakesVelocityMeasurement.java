package edu.mit.rocket_team.zephyrus.control.airbrakes;

public class RTAirbrakesVelocityMeasurement {
    public float velocityMeasurement;
    public float timeStamp;

    RTAirbrakesVelocityMeasurement(float ts, float vm){
        velocityMeasurement = vm;
        timeStamp = ts;
    }

    public float getVelocityMeasurement(){
        return velocityMeasurement;
    }

    public float getTimeStamp(){
        return timeStamp;
    }

    public void setData(float ts, float vm){
        timeStamp = ts;
        velocityMeasurement = vm;
    }
}
