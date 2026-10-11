package edu.mit.rocket_team.util.data;

/**
 * Mutable barometer acquisition: pressure in hPa, temperature in Celsius and optional raw ADC
 * counts. Raw values may be NaN at the simulator boundary.
 */
public class RTBaroData extends RTFudgedData {
    private float rawPressure;
    private float rawTemperature;
    private float temperature;
    private float altitude;
    private float pressure;

    /**
     * Creates a RTBaroData with the supplied dependencies and per-instance state.
     *
     * @param rP raw pressure count, or NaN when unavailable
     * @param rT raw temperature count, or NaN when unavailable
     * @param t temperature in degrees Celsius
     * @param a altitude in metres
     * @param p pressure in hPa
     */
    public RTBaroData(float rP, float rT, float t, float a, float p) {
        this.rawPressure = rP;
        this.rawTemperature = rT;
        this.temperature = t;
        this.altitude = a;
        this.pressure = p;
    }

    /**
     * Creates a RTBaroData with the supplied dependencies and per-instance state.
     *
     * @param rP raw pressure count, or NaN when unavailable
     * @param rT raw temperature count, or NaN when unavailable
     * @param t temperature in degrees Celsius
     * @param a altitude in metres
     * @param p pressure in hPa
     */
    public RTBaroData(Double rP, Double rT, Double t, Double a, Double p) {
        this.rawPressure = rP.floatValue();
        this.rawTemperature = rT.floatValue();
        this.temperature = t.floatValue();
        this.altitude = a.floatValue();
        this.pressure = p.floatValue();
    }

    /**
     * Returns the raw pressure.
     *
     * @return the raw pressure
     */
    public float getRawPressure() {
        return rawPressure;
    }

    /**
     * Returns the raw temperature.
     *
     * @return the raw temperature
     */
    public float getRawTemperature() {
        return rawTemperature;
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
     * Returns the altitude in metres.
     *
     * @return the altitude in metres
     */
    public float getAltitude() {
        return altitude;
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
     * Replaces the raw pressure stored by this instance.
     *
     * @param rawPressure raw pressure value
     */
    public void setRawPressure(float rawPressure) {
        this.rawPressure = rawPressure;
    }

    /**
     * Replaces the raw temperature stored by this instance.
     *
     * @param rawTemperature raw temperature value
     */
    public void setRawTemperature(float rawTemperature) {
        this.rawTemperature = rawTemperature;
    }

    /**
     * Replaces the temperature stored by this instance.
     *
     * @param temperature temperature value
     */
    public void setTemperature(float temperature) {
        this.temperature = temperature;
    }

    /**
     * Replaces the altitude stored by this instance.
     *
     * @param altitude altitude value
     */
    public void setAltitude(float altitude) {
        this.altitude = altitude;
    }

    /**
     * Replaces the pressure stored by this instance.
     *
     * @param pressure pressure value
     */
    public void setPressure(float pressure) {
        this.pressure = pressure;
    }
}
