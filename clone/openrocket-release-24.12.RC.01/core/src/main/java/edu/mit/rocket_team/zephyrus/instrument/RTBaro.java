package edu.mit.rocket_team.zephyrus.instrument;

import edu.mit.rocket_team.zephyrus.util.data.RTBaroData;
import edu.mit.rocket_team.zephyrus.util.data.RTFudgedData;
import edu.mit.rocket_team.zephyrus.util.RTInstrument;

public class RTBaro extends RTInstrument {

    private float rawPressure;
    private float rawTemperature;
    private float normalTemperature;
    private float rawAltitude;
    private float processedAltitude;


    public RTBaro() {

    }

    @Override
    public void setup() {

    }

    public float rawPressure() {
        return rawPressure;
    }

    public float rawTemperature() {
        return rawTemperature;
    }

    public float normalTemperature() {
        return normalTemperature;
    }

    public float getRawAltitude() {
        return rawAltitude;
    }

    public float getProcessedAltitude() {
        return processedAltitude;
    }

    public void backdoorFudge(RTFudgedData fudged) {
        if (fudged instanceof RTBaroData baroFudged) {
            this.rawPressure = baroFudged.getRawPressure();
            this.rawTemperature = baroFudged.getRawTemperature();
            this.normalTemperature = baroFudged.getNormalTemperature();
            this.rawAltitude = baroFudged.getRawAltitude();
            this.processedAltitude = baroFudged.getProcessedAltitude();
        }
        else {
            throw new IllegalArgumentException("Invalid fudged data type for RTBaro");
        }
    }
}
