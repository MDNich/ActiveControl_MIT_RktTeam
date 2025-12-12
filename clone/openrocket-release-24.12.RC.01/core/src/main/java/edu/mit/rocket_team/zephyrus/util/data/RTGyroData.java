package edu.mit.rocket_team.zephyrus.util.data;

public class RTGyroData extends RTFudgedData {
    private float gyroX;
    private float gyroY;
    private float gyroZ;

    public RTGyroData(float x, float y, float z) {
        this.gyroX = x;
        this.gyroY = y;
        this.gyroZ = z;
    }

    public float getGyroX() {
        return gyroX;
    }

    public float getGyroY() {
        return gyroY;
    }

    public float getGyroZ() {
        return gyroZ;
    }

    public void setGyroX(float gyroX) {
        this.gyroX = gyroX;
    }
    public void setGyroY(float gyroY) {
        this.gyroY = gyroY;
    }
    public void setGyroZ(float gyroZ) {
        this.gyroZ = gyroZ;
    }
}
