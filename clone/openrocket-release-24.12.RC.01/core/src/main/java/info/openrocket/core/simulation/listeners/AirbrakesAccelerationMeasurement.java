package info.openrocket.core.simulation.listeners;

public class AirbrakesAccelerationMeasurement {
    public double accelMeasurement;
    public double timeStamp;

    AirbrakesAccelerationMeasurement(double ts, double am){
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
