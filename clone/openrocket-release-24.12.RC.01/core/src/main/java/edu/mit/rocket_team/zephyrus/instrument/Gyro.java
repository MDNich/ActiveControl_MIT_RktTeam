package edu.mit.rocket_team.zephyrus.instrument;

import edu.mit.rocket_team.zephyrus.util.Instrument;

public class Gyro extends Instrument {

    float gyroX;
    float gyroY;
    float gyroZ;

    public Gyro() {

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
