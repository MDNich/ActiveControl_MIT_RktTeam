package edu.mit.rocket_team.zephyrus.instrument;

import edu.mit.rocket_team.zephyrus.util.Instrument;

public class Mag extends Instrument {


    float magX;
    float magY;
    float magZ;

    public Mag() {

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
