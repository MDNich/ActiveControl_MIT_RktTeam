package edu.mit.rocket_team.zephyrus.util.data;

public class RTMagData extends RTFudgedData {
    private float magX;
    private float magY;
    private float magZ;
    private float rollAngle;

    public RTMagData(float x, float y, float z, float rollAngle) {
        this.magX = x;
        this.magY = y;
        this.magZ = z;
        this.rollAngle = rollAngle;
    }

    public RTMagData(Double x, Double y, Double z, Double rollAngle) {
        this.magX      = x.floatValue();
        this.magY      = y.floatValue();
        this.magZ      = z.floatValue();
        this.rollAngle = rollAngle.floatValue();
    }

    public float getMagX() {
        return magX;
    }

    public float getMagY() {
        return magY;
    }

    public float getMagZ() {
        return magZ;
    }

    public float getRollAngle() {
        return rollAngle;
    }

    public void setMagX(float magX) {
        this.magX = magX;
    }
    public void setMagY(float magY) {
        this.magY = magY;
    }
    public void setMagZ(float magZ) {
        this.magZ = magZ;
    }
    public void setRollAngle(float rollAngle) {
        this.rollAngle = rollAngle;
    }
}
