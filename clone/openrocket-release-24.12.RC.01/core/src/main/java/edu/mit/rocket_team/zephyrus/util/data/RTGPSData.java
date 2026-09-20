package edu.mit.rocket_team.zephyrus.util.data;

public class RTGPSData extends RTFudgedData {
    private float latitude;
    private float longitude;
    private float altitude;
    private float PDOP;
    private float VDOP;
    private float HDOP;
    private boolean hasFix;
    private int fixType;
    public int getFixType() { return fixType; }
    public void setFixType(int type) { if(type<0 || type>5) throw new IllegalArgumentException("Invalid GPS fix type"); fixType=type; hasFix=type==3; }

    public RTGPSData(float lat, float lon, float alt,
                     float p, float v, float h, boolean hF) {
        this.latitude  = lat;
        this.longitude = lon;
        this.altitude  = alt;
        this.PDOP      = p;
        this.VDOP      = v;
        this.HDOP      = h;
        this.hasFix    = hF;
        this.fixType = hF ? 3 : 0;
    }

    public RTGPSData(Double lat, Double lon, Double alt,
                     Double p, Double v, Double h, boolean hF) {
        this.latitude  = lat.floatValue();
        this.longitude = lon.floatValue();
        this.altitude  = alt.floatValue();
        this.PDOP      = p.floatValue();
        this.VDOP      = v.floatValue();
        this.HDOP      = h.floatValue();
        this.hasFix    = hF;
        this.fixType = hF ? 3 : 0;
    }

    public float getLatitude() {
        return latitude;
    }

    public float getLongitude() {
        return longitude;
    }

    public float getAltitude() {
        return altitude;
    }

    public float getPDOP() {
        return PDOP;
    }

    public float getVDOP() {
        return VDOP;
    }

    public float getHDOP() {
        return HDOP;
    }

    public boolean getHasFix() {
        return hasFix;
    }

    public void setLatitude(float lat) {
        this.latitude = lat;
    }
    public void setLongitude(float lon) {
        this.longitude = lon;
    }
    public void setAltitude(float alt) {
        this.altitude = alt;
    }
    public void setPDOP(float p) {
        this.PDOP = p;
    }
    public void setVDOP(float v) {
        this.VDOP = v;
    }
    public void setHDOP(float h) {
        this.HDOP = h;
    }
    public void setHasFix(boolean hasFix) {
        this.hasFix = hasFix;
        this.fixType = hasFix ? 3 : 0;
    }
}
