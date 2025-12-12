package edu.mit.rocket_team.zephyrus.instrument;

import edu.mit.rocket_team.zephyrus.util.RTInstrument;

public class RTGyro extends RTInstrument {

    float gyroX;
    float gyroY;
    float gyroZ;

    public RTGyro() {

    }

    @Override
    public void setup() {
        begin();
        configure();
    }

    private void begin() {

    }

    private void configure() {

    }

    public float getGyroX() {
        return gyroX;
    }
    public float getGyroY() {
        return gyroY;
    }
    public float getGyroZ() {
        return gyroZ;
    }

}
