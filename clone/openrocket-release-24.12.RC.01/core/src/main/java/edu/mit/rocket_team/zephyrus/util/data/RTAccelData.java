package edu.mit.rocket_team.zephyrus.util.data;

public class RTAccelData extends RTFudgedData {
    private float accelX;
    private float accelY;
    private float accelZ;

    public RTAccelData(float x, float y, float z) {
        this.accelX = x;
        this.accelY = y;
        this.accelZ = z;
    }

    public float getAccelX() {
        return accelX;
    }

    public float getAccelY() {
        return accelY;
    }

    public float getAccelZ() {
        return accelZ;
    }
    public void setAccelX(float accelX) {
        this.accelX = accelX;
    }
    public void setAccelY(float accelY) {
        this.accelY = accelY;
    }
    public void setAccelZ(float accelZ) {
        this.accelZ = accelZ;
    }
}
