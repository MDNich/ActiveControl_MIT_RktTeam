package edu.mit.rocket_team.zephyrus.util.data;

public class RTMagData extends RTFudgedData {
    private float magX;
    private float magY;
    private float magZ;

    public RTMagData(float x, float y, float z) {
        this.magX = x;
        this.magY = y;
        this.magZ = z;
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

    public void setMagX(float magX) {
        this.magX = magX;
    }
    public void setMagY(float magY) {
        this.magY = magY;
    }
    public void setMagZ(float magZ) {
        this.magZ = magZ;
    }
}
