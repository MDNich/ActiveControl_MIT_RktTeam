package edu.mit.rocket_team.util.data;

/** Mutable sensor-axis angular rates in degrees per second. */
public class RTGyroData extends RTFudgedData {
    private float gyroX;
    private float gyroY;
    private float gyroZ;

    /**
     * Creates a RTGyroData with the supplied dependencies and per-instance state.
     *
     * @param x X-axis angular rate in degrees per second
     * @param y Y-axis angular rate in degrees per second
     * @param z Z-axis angular rate in degrees per second
     */
    public RTGyroData(float x, float y, float z) {
        this.gyroX = x;
        this.gyroY = y;
        this.gyroZ = z;
    }

    /**
     * Creates a RTGyroData with the supplied dependencies and per-instance state.
     *
     * @param x X-axis angular rate in degrees per second
     * @param y Y-axis angular rate in degrees per second
     * @param z Z-axis angular rate in degrees per second
     */
    public RTGyroData(Double x, Double y, Double z) {
        this.gyroX = x.floatValue();
        this.gyroY = y.floatValue();
        this.gyroZ = z.floatValue();
    }

    /**
     * Returns the gyro x in degrees per second.
     *
     * @return the gyro x in degrees per second
     */
    public float getGyroX() {
        return gyroX;
    }

    /**
     * Returns the gyro y in degrees per second.
     *
     * @return the gyro y in degrees per second
     */
    public float getGyroY() {
        return gyroY;
    }

    /**
     * Returns the gyro z in degrees per second.
     *
     * @return the gyro z in degrees per second
     */
    public float getGyroZ() {
        return gyroZ;
    }

    /**
     * Replaces the gyro x stored by this instance.
     *
     * @param gyroX gyro x value
     */
    public void setGyroX(float gyroX) {
        this.gyroX = gyroX;
    }

    /**
     * Replaces the gyro y stored by this instance.
     *
     * @param gyroY gyro y value
     */
    public void setGyroY(float gyroY) {
        this.gyroY = gyroY;
    }

    /**
     * Replaces the gyro z stored by this instance.
     *
     * @param gyroZ gyro z value
     */
    public void setGyroZ(float gyroZ) {
        this.gyroZ = gyroZ;
    }
}
