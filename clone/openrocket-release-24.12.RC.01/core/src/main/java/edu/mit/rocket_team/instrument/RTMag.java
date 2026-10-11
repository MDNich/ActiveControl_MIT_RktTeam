package edu.mit.rocket_team.instrument;

import edu.mit.rocket_team.util.RTInstrument;
import edu.mit.rocket_team.util.data.RTFudgedData;
import edu.mit.rocket_team.util.data.RTMagData;

/** Legacy magnetometer injection adapter; the active FC loop does not acquire this device. */
public class RTMag extends RTInstrument {

    float magX;
    float magY;
    float magZ;
    float rollAngle;

    /** Creates a RTMag instance with its default state. */
    public RTMag() {}

    /**
     * Retains the magnetometer setup hook; this adapter is a placeholder outside the active FC
     * sensor loop.
     */
    @Override
    public void setup() {}

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
     * Returns the roll in degrees.
     *
     * @return the roll in degrees
     */
    public float getRoll() {
        // return (float) Math.atan(magY / magX); // real calculation
        return this.rollAngle; // real calculation
    }

    /**
     * Copies an explicitly supplied magnetometer/roll bundle into the placeholder adapter.
     *
     * @param fudged fudged
     * @throws IllegalArgumentException when rejected: Invalid fudged data type for RTMag
     */
    @Override
    public void backdoorFudge(RTFudgedData fudged) {
        if (fudged instanceof RTMagData magFudged) {
            this.magX = magFudged.getMagX();
            this.magY = magFudged.getMagY();
            this.magZ = magFudged.getMagZ();
            this.rollAngle = magFudged.getRollAngle();
        } else {
            throw new IllegalArgumentException("Invalid fudged data type for RTMag");
        }
    }
}
