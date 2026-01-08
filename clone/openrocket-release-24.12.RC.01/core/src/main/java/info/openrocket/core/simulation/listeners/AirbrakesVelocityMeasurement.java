package info.openrocket.core.simulation.listeners;

public class AirbrakesVelocityMeasurement {
    public double velocityMeasurement;
    public double timeStamp;

    AirbrakesVelocityMeasurement(double ts, double vm){
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
