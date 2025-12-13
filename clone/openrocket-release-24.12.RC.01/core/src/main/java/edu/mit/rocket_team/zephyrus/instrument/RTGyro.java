package edu.mit.rocket_team.zephyrus.instrument;

import edu.mit.rocket_team.zephyrus.util.RTInstrument;
import edu.mit.rocket_team.zephyrus.util.data.RTFudgedData;
import edu.mit.rocket_team.zephyrus.util.data.RTGyroData;

public class RTGyro extends RTInstrument {

    float gyroX;
    float gyroY;
    float gyroZ;

    public RTGyro() {

    }

    @Override
    public void setup() {
        begin();
        config();
    }

    private void begin() {

    }

    private void config() {

    }

    public float gyro_x() {
        return gyroX;
    }
    public float gyro_y() {
        return gyroY;
    }
    public float gyro_z() {
        return gyroZ;
    }

    @Override
    public void backdoorFudge(RTFudgedData fudged) {
        if (fudged instanceof RTGyroData gyroFudged) {
            this.gyroX = gyroFudged.getGyroX();
            this.gyroY = gyroFudged.getGyroY();
            this.gyroZ = gyroFudged.getGyroZ();
        }
        else {
            throw new IllegalArgumentException("Invalid fudged data type for RTGyro");
        }
    }
}
