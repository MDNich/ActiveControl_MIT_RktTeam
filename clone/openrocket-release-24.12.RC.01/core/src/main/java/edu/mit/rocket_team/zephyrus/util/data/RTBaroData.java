package edu.mit.rocket_team.zephyrus.util.data;

public class RTBaroData extends RTFudgedData {
    private float rawPressure;
    private float rawTemperature;
    private float normalTemperature;
    private float getRawAltitude;
    private float getProcessedAltitude;

    public RTBaroData(float rP, float rT, float nT, float rA, float pA) {
        this.rawPressure = rP;
        this.rawTemperature = rT;
        this.normalTemperature = nT;
        this.getRawAltitude = rA;
        this.getProcessedAltitude = pA;
    }

    public float getRawPressure() {
        return rawPressure;
    }
    public float getRawTemperature() {
        return rawTemperature;
    }
    public float getNormalTemperature() {
        return normalTemperature;
    }
    public float getRawAltitude() {
        return getRawAltitude;
    }
    public float getProcessedAltitude() {
        return getProcessedAltitude;
    }

    public void setRawPressure(float rawPressure) {
        this.rawPressure = rawPressure;
    }
    public void setRawTemperature(float rawTemperature) {
        this.rawTemperature = rawTemperature;
    }
    public void setNormalTemperature(float normalTemperature) {
        this.normalTemperature = normalTemperature;
    }
    public void setRawAltitude(float rawAltitude) {
        this.getRawAltitude = rawAltitude;
    }
    public void setProcessedAltitude(float processedAltitude) {
        this.getProcessedAltitude = processedAltitude;
    }
}
