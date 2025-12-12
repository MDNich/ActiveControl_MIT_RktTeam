package edu.mit.rocket_team.zephyrus.instrument;

import edu.mit.rocket_team.zephyrus.util.RTInstrument;

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


}
