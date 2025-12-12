package edu.mit.rocket_team.zephyrus.instrument;

import edu.mit.rocket_team.zephyrus.util.data.RTAccelData;
import edu.mit.rocket_team.zephyrus.util.data.RTFudgedData;
import edu.mit.rocket_team.zephyrus.util.RTInstrument;

public class RTAccel extends RTInstrument {

    float accelX;
    float accelY;
    float accelZ;

    public RTAccel() {

    }

    @Override
    public void setup() {
        accelX = 0;
        accelY = 0;
        accelZ = (float) 9.81;
    }

    public void update() {

    }

    public float getAccelX() {
        return this.accelX;
    }

    public float getAccelY() {
        return this.accelY;
    }

    public float getAccelZ() {
        return this.accelZ;
    }

    public void backdoorFudge(RTFudgedData fudged) {
        if (fudged instanceof RTAccelData accelFudged) {
            this.accelX = accelFudged.getAccelX();
            this.accelY = accelFudged.getAccelY();
            this.accelZ = accelFudged.getAccelZ();
        }
        else {
            throw new IllegalArgumentException("Invalid fudged data type for RTAccel");
        }
    }
}
