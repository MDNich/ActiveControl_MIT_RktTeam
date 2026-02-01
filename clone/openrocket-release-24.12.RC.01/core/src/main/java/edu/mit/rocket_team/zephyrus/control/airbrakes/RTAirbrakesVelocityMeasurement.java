package edu.mit.rocket_team.zephyrus.control.airbrakes;

public class RTAirbrakesVelocityMeasurement {
    public double velocityMeasurement;
    public double timeStamp;

    RTAirbrakesVelocityMeasurement(double ts, double vm){
        velocityMeasurement = vm;
        timeStamp = ts;
    }

    public double getVelocityMeasurement(){
        return velocityMeasurement;
    }

    public double getTimeStamp(){
        return timeStamp;
    }

    public void setData(double ts, double vm){
        timeStamp = ts;
        velocityMeasurement = vm;
    }
}
