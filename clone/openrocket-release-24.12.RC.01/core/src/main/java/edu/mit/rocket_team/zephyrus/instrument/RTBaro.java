package edu.mit.rocket_team.zephyrus.instrument;

import edu.mit.rocket_team.zephyrus.util.RTInstrument;

public class RTBaro extends RTInstrument {

    public RTBaro() {

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
