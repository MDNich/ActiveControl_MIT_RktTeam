package edu.mit.rocket_team.zephyrus.instrument;

import edu.mit.rocket_team.zephyrus.util.Instrument;

public class Baro extends Instrument {

    public Baro() {

    }

    @Override
    public void setup() {

    }

    public float rawPressure() {
        return 0;
    }

    public float rawTemperature() {
        return 0;
    }

    public float normalTemperature() {
        return 0;
    }

    public float getRawAltitude() {
        return 0;
    }

    public float getProcessedAltitude() {
        return 0;
    }
}
