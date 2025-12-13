package edu.mit.rocket_team.zephyrus.instrument;

import edu.mit.rocket_team.zephyrus.util.data.RTFudgedData;
import edu.mit.rocket_team.zephyrus.util.data.RTGPSData;
import edu.mit.rocket_team.zephyrus.util.RTInstrument;

public class RTGPS extends RTInstrument {

    private float latitude;
    private float longitude;
    private float altitude;

    private float PDOP;
    private float VDOP;
    private float HDOP;

    private boolean hasFix;

    public RTGPS() {

    }


    @Override
    public void setup() {

    }

    public void update() {
        // update the class variables.
    }

    public void parse() {
        // update the class variables.
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

    public boolean getFix() {
        // return if the GPS has a fix
        return hasFix;
    }

    public void backdoorFudge(RTFudgedData fudged) {
        if  (fudged instanceof RTGPSData gpsData) {
            this.latitude = gpsData.getLatitude();
            this.longitude = gpsData.getLongitude();
            this.altitude = gpsData.getAltitude();
            this.PDOP = gpsData.getPDOP();
            this.VDOP = gpsData.getVDOP();
            this.HDOP = gpsData.getHDOP();
            this.hasFix = gpsData.getHasFix();
        }
        else {
            throw new IllegalArgumentException("Invalid fudged data type for RTGPS");
        }
    }



}
