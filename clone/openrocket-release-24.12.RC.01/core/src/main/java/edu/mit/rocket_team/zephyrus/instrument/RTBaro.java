package edu.mit.rocket_team.zephyrus.instrument;

import edu.mit.rocket_team.zephyrus.util.data.RTBaroData;
import edu.mit.rocket_team.zephyrus.util.data.RTFudgedData;
import edu.mit.rocket_team.zephyrus.util.RTInstrument;

public class RTBaro extends RTInstrument {

    private float rawPressure;
    private float rawTemperature;
    private float temperature;
    private float altitude;
    private float pressure;
    private float movingAvg;

    public RTBaro() {

    }

    @Override
    public void setup() {

    }

    /* jbolte @ dec 13
    Essentially, you have to tell it to start a conversion,
    then wait, then get the result of the conversion.
    The timeframe for the OSR (over sampling rate) we used for xanthus was 3ms,
    so 3ms for temp and 3ms for pressure. I think for now we will split the
    begin_conversion and then read, and think more about the logic.
     */

    public void startPressureConversion() {
        // no-op for the moment
    }

    public void startTemperatureConversion() {
        // no-op for the moment
    }

    // uint32_t in arduino.
    public float getRawPressure() {
        return rawPressure;
    }

    // uint32_t in arduino.
    public float getRawTemperature() {
        return rawTemperature;
    }

    public float pressure() {
        return pressure;
    }

    public float altitude() {
        return altitude;
    }

    public float altitudeMavg() {
        // todo: calculate moving average according to spec
        movingAvg = altitude; // placeholder
        return movingAvg;
    }

    public float temperature() {
        return temperature;
    }

    @Override
    public void backdoorFudge(RTFudgedData fudged) {
        if (fudged instanceof RTBaroData baroFudged) {
            this.rawPressure = baroFudged.getRawPressure();
            this.rawTemperature = baroFudged.getRawTemperature();
            this.temperature = baroFudged.getTemperature();
            this.altitude = baroFudged.getAltitude();
            this.pressure = baroFudged.getPressure();
        }
        else {
            throw new IllegalArgumentException("Invalid fudged data type for RTBaro");
        }
    }
}
