package edu.mit.rocket_team.util.data;

/**
 * Mutable GPS acquisition with decimal-degree coordinates, MSL altitude in metres and dimensionless
 * dilution-of-precision fields.
 */
public class RTGPSData extends RTFudgedData {
    private double latitude;
    private double longitude;
    private double altitude;
    private float PDOP;
    private float VDOP;
    private float HDOP;
    private boolean hasFix;
    private int fixType;

    /**
     * Returns the fix type.
     *
     * @return the fix type
     */
    public int getFixType() {
        return fixType;
    }

    /**
     * Replaces the fix type stored by this instance.
     *
     * @param type fix type value
     * @throws IllegalArgumentException when rejected: Invalid GPS fix type
     */
    public void setFixType(int type) {
        if (type < 0 || type > 5) throw new IllegalArgumentException("Invalid GPS fix type");
        fixType = type;
        hasFix = type == 3;
    }

    /**
     * Creates a RTGPSData with the supplied dependencies and per-instance state.
     *
     * @param lat latitude in decimal degrees
     * @param lon longitude in decimal degrees
     * @param alt GPS altitude in metres above mean sea level
     * @param p position dilution of precision
     * @param v vertical dilution of precision
     * @param h horizontal dilution of precision
     * @param hF whether a 3D fix is available
     */
    public RTGPSData(float lat, float lon, float alt, float p, float v, float h, boolean hF) {
        this.latitude = lat;
        this.longitude = lon;
        this.altitude = alt;
        this.PDOP = p;
        this.VDOP = v;
        this.HDOP = h;
        this.hasFix = hF;
        this.fixType = hF ? 3 : 0;
    }

    /**
     * Creates a RTGPSData with the supplied dependencies and per-instance state.
     *
     * @param lat latitude in decimal degrees
     * @param lon longitude in decimal degrees
     * @param alt GPS altitude in metres above mean sea level
     * @param p position dilution of precision
     * @param v vertical dilution of precision
     * @param h horizontal dilution of precision
     * @param hF whether a 3D fix is available
     */
    public RTGPSData(Double lat, Double lon, Double alt, Double p, Double v, Double h, boolean hF) {
        this.latitude = lat.doubleValue();
        this.longitude = lon.doubleValue();
        this.altitude = alt.doubleValue();
        this.PDOP = p.floatValue();
        this.VDOP = v.floatValue();
        this.HDOP = h.floatValue();
        this.hasFix = hF;
        this.fixType = hF ? 3 : 0;
    }

    /**
     * Returns the latitude in decimal degrees.
     *
     * @return the latitude in decimal degrees
     */
    public double getLatitude() {
        return latitude;
    }

    /**
     * Returns the longitude in decimal degrees.
     *
     * @return the longitude in decimal degrees
     */
    public double getLongitude() {
        return longitude;
    }

    /**
     * Returns the altitude in metres.
     *
     * @return the altitude in metres
     */
    public double getAltitude() {
        return altitude;
    }

    /**
     * Returns the pdop as a dimensionless dilution of precision.
     *
     * @return the pdop as a dimensionless dilution of precision
     */
    public float getPDOP() {
        return PDOP;
    }

    /**
     * Returns the vdop as a dimensionless dilution of precision.
     *
     * @return the vdop as a dimensionless dilution of precision
     */
    public float getVDOP() {
        return VDOP;
    }

    /**
     * Returns the hdop as a dimensionless dilution of precision.
     *
     * @return the hdop as a dimensionless dilution of precision
     */
    public float getHDOP() {
        return HDOP;
    }

    /**
     * Returns the has fix.
     *
     * @return the has fix
     */
    public boolean getHasFix() {
        return hasFix;
    }

    /**
     * Replaces the latitude stored by this instance.
     *
     * @param lat latitude in decimal degrees
     */
    public void setLatitude(double lat) {
        this.latitude = lat;
    }

    /**
     * Replaces the longitude stored by this instance.
     *
     * @param lon longitude in decimal degrees
     */
    public void setLongitude(double lon) {
        this.longitude = lon;
    }

    /**
     * Replaces the altitude stored by this instance.
     *
     * @param alt GPS altitude in metres above mean sea level
     */
    public void setAltitude(double alt) {
        this.altitude = alt;
    }

    /**
     * Replaces the pdop stored by this instance.
     *
     * @param p position dilution of precision
     */
    public void setPDOP(float p) {
        this.PDOP = p;
    }

    /**
     * Replaces the vdop stored by this instance.
     *
     * @param v vertical dilution of precision
     */
    public void setVDOP(float v) {
        this.VDOP = v;
    }

    /**
     * Replaces the hdop stored by this instance.
     *
     * @param h horizontal dilution of precision
     */
    public void setHDOP(float h) {
        this.HDOP = h;
    }

    /**
     * Replaces the has fix stored by this instance.
     *
     * @param hasFix has fix value
     */
    public void setHasFix(boolean hasFix) {
        this.hasFix = hasFix;
        this.fixType = hasFix ? 3 : 0;
    }
}
