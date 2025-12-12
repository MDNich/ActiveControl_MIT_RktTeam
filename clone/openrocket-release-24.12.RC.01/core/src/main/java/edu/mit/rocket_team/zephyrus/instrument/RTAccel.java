package edu.mit.rocket_team.zephyrus.instrument;

import edu.mit.rocket_team.zephyrus.util.RTInstrument;
import info.openrocket.core.util.Coordinate;

public class RTAccel extends RTInstrument {

    private Coordinate currentAccel;

    public RTAccel() {

    }

    @Override
    public void setup() {
        currentAccel = new Coordinate(0,0,9.81);
    }

    public void update() {

    }

    public float getAccelX() {
        return (float) currentAccel.x;
    }

    public float getAccelY() {
        return (float) currentAccel.y;
    }

    public float getAccelZ() {
        return (float) currentAccel.z;
    }


    public void backdoorCoordinate(Coordinate fudged) {
        this.currentAccel = fudged.clone();
    }
}
