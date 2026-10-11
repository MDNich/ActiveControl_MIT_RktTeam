package edu.mit.rocket_team.instrument;

import edu.mit.rocket_team.util.*;
import edu.mit.rocket_team.util.data.*;

/** baro.cpp: engineering-unit acquisition precedes the unchanged altitude/filter math. */
public class RTBaro extends RTInstrument {
    private float pressure = 1013.25f,
            temperature = 20,
            rawPressure = Float.NaN,
            rawTemperature = Float.NaN;
    private float[] samples = new float[20];

    /**
     * Replaces the moving-average buffer with 1–1000 zero-initialized samples.
     *
     * @param count number of entries or samples
     * @throws IllegalArgumentException when rejected: Filter length
     */
    public void setFilterSamples(int count) {
        if (count < 1 || count > 1000) throw new IllegalArgumentException("Filter length");
        samples = new float[count];
    }

    private float filtered, offset, maxAlt;

    /** Creates a RTBaro with the supplied dependencies and per-instance state. */
    public RTBaro() {
        super();
    }

    /**
     * Creates a RTBaro with the supplied dependencies and per-instance state.
     *
     * @param trace run-local virtual clock and action-log sink
     */
    public RTBaro(RTUtilLibrary.Trace trace) {
        super(trace);
    }

    /** Records the filter length and engineering-unit barometer interface. */
    @Override
    public void setup() {
        trace.log(
                "baro.setup",
                "samples=" + samples.length + " pressure=hPa temperature=C raw=unavailable");
    }

    /**
     * Loads pressure in hPa and temperature in Celsius, rejecting nonpositive pressure or
     * non-finite temperature.
     *
     * @param data input data for this operation
     * @throws IllegalArgumentException when rejected: Invalid barometer observation
     */
    @Override
    public void backdoorFudge(RTFudgedData data) {
        RTBaroData d = (RTBaroData) data;
        pressure = d.getPressure();
        temperature = d.getTemperature();
        rawPressure = d.getRawPressure();
        rawTemperature = d.getRawTemperature();
        if (!(pressure > 0) || !Float.isFinite(temperature))
            throw new IllegalArgumentException("Invalid barometer observation");
    }

    /**
     * Adds one pressure-derived altitude to the moving average and updates the maximum relative
     * altitude.
     */
    public void updateAll() {
        System.arraycopy(samples, 0, samples, 1, samples.length - 1);
        samples[0] = getAltitude();
        filtered = 0;
        for (float s : samples) filtered += s;
        filtered /= samples.length;
        if (getFilteredAltitude() > maxAlt) maxAlt = getFilteredAltitude();
        trace.log(
                "baro.update",
                "pressure_hPa="
                        + pressure
                        + " temperature_C="
                        + temperature
                        + " altitude_m="
                        + samples[0]
                        + " filtered_m="
                        + getFilteredAltitude()
                        + " maximum_m="
                        + maxAlt);
    }

    /**
     * Computes unfiltered pressure altitude in metres using the firmware atmosphere formula.
     *
     * @return computed numeric value
     */
    public float getAltitude() {
        float p = 1013.25f / pressure;
        return (float) (153.84615 * (Math.pow(p, 0.19) - 1) * (temperature + 273.15));
    }

    /**
     * Returns moving-average altitude minus the current zero reference, in metres.
     *
     * @return moving-average altitude minus the current zero reference, in metres
     */
    public float getFilteredAltitude() {
        return filtered - offset;
    }

    /**
     * Applies the firmware zero-reference operation and clears maximum altitude; repeated calls
     * retain the source behavior.
     */
    public void zeroAlt() {
        offset = getFilteredAltitude();
        maxAlt = 0;
        trace.log("baro.zero", "offset_m=" + offset);
    }

    /** Clears the maximum recorded relative altitude. */
    public void resetMaxAlt() {
        maxAlt = 0;
        trace.log("baro.reset_max", "maximum_m=0");
    }

    /**
     * Returns the largest filtered relative altitude observed since the last reset, in metres.
     *
     * @return the largest filtered relative altitude observed since the last reset, in metres
     */
    public float getMaxAlt() {
        return maxAlt;
    }

    /**
     * Returns the pressure in hectopascals.
     *
     * @return the pressure in hectopascals
     */
    public float getPressure() {
        return pressure;
    }

    /**
     * Returns the temperature in degrees Celsius.
     *
     * @return the temperature in degrees Celsius
     */
    public float getTemperature() {
        return temperature;
    }

    /**
     * Returns the raw pressure count, or NaN when only an engineering-unit observation is
     * available.
     *
     * @return the raw pressure
     */
    public float getRawPressure() {
        return rawPressure;
    }

    /**
     * Returns the raw temperature count, or NaN when only an engineering-unit observation is
     * available.
     *
     * @return the raw temperature
     */
    public float getRawTemperature() {
        return rawTemperature;
    }

    /** Traces a pressure-conversion request without adding hardware conversion latency. */
    public void startPressureConversion() {
        trace.log("baro.conversion", "kind=pressure modeled_delay=0");
    }

    /** Traces a temperature-conversion request without adding hardware conversion latency. */
    public void startTemperatureConversion() {
        trace.log("baro.conversion", "kind=temperature modeled_delay=0");
    }

    /** baro.cpp compensation constants, for independent raw replay inputs. */
    public static RTBaroData fromRaw(int rawP, int rawT) {
        float dT = (float) rawT - (float) 0x8405 * 256;
        float temp = (float) (2000.0 + dT * (float) 0x6D91 / (float) (1L << 23));
        long off = (long) (((long) 0x953A) * (1L << 17) + dT * (float) 0x6305 / (1 << 6));
        long sens = (long) ((float) 0xA579 * (1L << 16) + dT * (float) 0x68AC / (1 << 7));
        float pa = (float) rawP / (1L << 15),
                pb = sens / (float) (1L << 21),
                pc = pa * pb,
                pd = off / (float) (1L << 15);
        return new RTBaroData((float) rawP, (float) rawT, temp / 100, Float.NaN, (pc - pd) / 100);
    }
}
