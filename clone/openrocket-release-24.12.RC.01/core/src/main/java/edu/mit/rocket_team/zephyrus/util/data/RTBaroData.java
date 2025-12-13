package edu.mit.rocket_team.zephyrus.util.data;

public class RTBaroData extends RTFudgedData {
    private float rawPressure;
    private float rawTemperature;
    private float temperature;
    private float altitude;
    private float pressure;

    public RTBaroData(float rP, float rT, float t, float a, float p) {
        this.rawPressure = rP;
        this.rawTemperature = rT;
        this.temperature = t;
        this.altitude = a;
        this.pressure = p;
    }

    public float getRawPressure() {
        return rawPressure;
    }
    public float getRawTemperature() {
        return rawTemperature;
    }
    public float getTemperature() {
        return temperature;
    }
    public float getAltitude() {
        return altitude;
    }
    public float getPressure() {
        return pressure;
    }

    public void setRawPressure(float rawPressure) {
        this.rawPressure = rawPressure;
    }
    public void setRawTemperature(float rawTemperature) {
        this.rawTemperature = rawTemperature;
    }
    public void setTemperature(float temperature) {
        this.temperature = temperature;
    }
    public void setAltitude(float altitude) {
        this.altitude = altitude;
    }
    public void setPressure(float pressure) {
        this.pressure = pressure;
    }
}