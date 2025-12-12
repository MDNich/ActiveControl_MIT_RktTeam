package edu.mit.rocket_team.zephyrus.instrument;

import edu.mit.rocket_team.zephyrus.util.RTInstrument;
import edu.mit.rocket_team.zephyrus.util.data.RTFudgedData;
import edu.mit.rocket_team.zephyrus.util.data.RTMagData;

public class RTMag extends RTInstrument {


    float magX;
    float magY;
    float magZ;

    public RTMag() {

    }

    @Override
    public void setup() {

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

    @Override
    public void backdoorFudge(RTFudgedData fudged) {
        if (fudged instanceof RTMagData magFudged) {
            this.magX = magFudged.getMagX();
            this.magY = magFudged.getMagY();
            this.magZ = magFudged.getMagZ();
        }
        else {
            throw new IllegalArgumentException("Invalid fudged data type for RTMag");
        }
    }
}
    