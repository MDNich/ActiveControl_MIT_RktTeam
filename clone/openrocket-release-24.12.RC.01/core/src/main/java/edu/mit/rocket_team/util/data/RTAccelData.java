package edu.mit.rocket_team.util.data;

/** Mutable calibrated sensor-axis specific force in m/s²; sensor X is longitudinal. */
public class RTAccelData extends RTFudgedData {
    private float accelX;
    private float accelY;
    private float accelZ;

    /**
     * Creates a RTAccelData with the supplied dependencies and per-instance state.
     *
     * @param x X-axis specific force in m/s²
     * @param y Y-axis specific force in m/s²
     * @param z Z-axis specific force in m/s²
     */
    public RTAccelData(float x, float y, float z) {
        this.accelX = x;
        this.accelY = y;
        this.accelZ = z;
    }

    /**
     * Creates a RTAccelData with the supplied dependencies and per-instance state.
     *
     * @param x X-axis specific force in m/s²
     * @param y Y-axis specific force in m/s²
     * @param z Z-axis specific force in m/s²
     */
    public RTAccelData(Double x, Double y, Double z) {
        this.accelX = x.floatValue();
        this.accelY = y.floatValue();
        this.accelZ = z.floatValue();
    }

    /**
     * Returns the accel x in metres per second squared.
     *
     * @return the accel x in metres per second squared
     */
    public float getAccelX() {
        return accelX;
    }

    /**
     * Returns the accel y in metres per second squared.
     *
     * @return the accel y in metres per second squared
     */
    public float getAccelY() {
        return accelY;
    }

    /**
     * Returns the accel z in metres per second squared.
     *
     * @return the accel z in metres per second squared
     */
    public float getAccelZ() {
        return accelZ;
    }

    /**
     * Replaces the accel x stored by this instance.
     *
     * @param accelX accel x value
     */
    public void setAccelX(float accelX) {
        this.accelX = accelX;
    }

    /**
     * Replaces the accel y stored by this instance.
     *
     * @param accelY accel y value
     */
    public void setAccelY(float accelY) {
        this.accelY = accelY;
    }

    /**
     * Replaces the accel z stored by this instance.
     *
     * @param accelZ accel z value
     */
    public void setAccelZ(float accelZ) {
        this.accelZ = accelZ;
    }
}
