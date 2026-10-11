package edu.mit.rocket_team.util.data;

/**
 * Mutable legacy magnetometer readings and externally supplied roll angle; no calibrated sensor
 * model is implied.
 */
public class RTMagData extends RTFudgedData {
    private float magX;
    private float magY;
    private float magZ;
    private float rollAngle;

    /**
     * Creates a RTMagData with the supplied dependencies and per-instance state.
     *
     * @param x X component or horizontal coordinate
     * @param y Y component or vertical coordinate
     * @param z Z component
     * @param rollAngle initial roll angle
     */
    public RTMagData(float x, float y, float z, float rollAngle) {
        this.magX = x;
        this.magY = y;
        this.magZ = z;
        this.rollAngle = rollAngle;
    }

    /**
     * Creates a RTMagData with the supplied dependencies and per-instance state.
     *
     * @param x X component or horizontal coordinate
     * @param y Y component or vertical coordinate
     * @param z Z component
     * @param rollAngle initial roll angle
     */
    public RTMagData(Double x, Double y, Double z, Double rollAngle) {
        this.magX = x.floatValue();
        this.magY = y.floatValue();
        this.magZ = z.floatValue();
        this.rollAngle = rollAngle.floatValue();
    }

    /**
     * Returns the mag x.
     *
     * @return the mag x
     */
    public float getMagX() {
        return magX;
    }

    /**
     * Returns the mag y.
     *
     * @return the mag y
     */
    public float getMagY() {
        return magY;
    }

    /**
     * Returns the mag z.
     *
     * @return the mag z
     */
    public float getMagZ() {
        return magZ;
    }

    /**
     * Returns the roll angle.
     *
     * @return the roll angle
     */
    public float getRollAngle() {
        return rollAngle;
    }

    /**
     * Replaces the mag x stored by this instance.
     *
     * @param magX mag x value
     */
    public void setMagX(float magX) {
        this.magX = magX;
    }

    /**
     * Replaces the mag y stored by this instance.
     *
     * @param magY mag y value
     */
    public void setMagY(float magY) {
        this.magY = magY;
    }

    /**
     * Replaces the mag z stored by this instance.
     *
     * @param magZ mag z value
     */
    public void setMagZ(float magZ) {
        this.magZ = magZ;
    }

    /**
     * Replaces the roll angle stored by this instance.
     *
     * @param rollAngle roll angle value
     */
    public void setRollAngle(float rollAngle) {
        this.rollAngle = rollAngle;
    }
}
