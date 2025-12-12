package edu.mit.rocket_team.zephyrus.instrument;

import edu.mit.rocket_team.zephyrus.util.Instrument;

public class GPS extends Instrument {

    private float latitude;
    private float longitude;
    private float altitude;

    private float PDOP;
    private float VDOP;
    private float HDOP;

    public GPS() {

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
        return false;
    }



}
