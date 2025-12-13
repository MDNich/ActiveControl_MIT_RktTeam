package edu.mit.rocket_team.zephyrus.instrument;

import edu.mit.rocket_team.zephyrus.util.data.RTFudgedData;
import edu.mit.rocket_team.zephyrus.util.data.RTGPSData;
import edu.mit.rocket_team.zephyrus.util.RTInstrument;
import edu.mit.rocket_team.zephyrus.util.hardware.RTHardwareSerial;

public class RTGPS extends RTInstrument {

    private float latitude;
    private float longitude;
    private float altitude;

    private float PDOP;
    private float VDOP;
    private float HDOP;

    private boolean fix;

    private char[] gpsData;

    private float rawGPS;
    private float gpsAltOffset;
    private float maxGpsAlt;

    private RTHardwareSerial gpsSer;

    public RTGPS() {
        gpsData = new char[100];
    }


    @Override
    public void setup() {

    }

    public void updateAndParse() {
        // update and parse from the physical sensor; no-op in simulation, perhaps a wait.
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
        return fix;
    }

    public void backdoorFudge(RTFudgedData fudged) {
        if  (fudged instanceof RTGPSData gpsData) {
            this.latitude = gpsData.getLatitude();
            this.longitude = gpsData.getLongitude();
            this.altitude = gpsData.getAltitude();
            this.PDOP = gpsData.getPDOP();
            this.VDOP = gpsData.getVDOP();
            this.HDOP = gpsData.getHDOP();
            this.fix = gpsData.getHasFix();
        }
        else {
            throw new IllegalArgumentException("Invalid fudged data type for RTGPS");
        }
    }


    // Private methods to parse GPS data would go here
    private boolean gpsUpdateData(int length) {
        return true;
    }

    private void gpsParse(char[] data, int length) {
        // no-op
    }

    private float convertToDecimalFast(float raw, char hemi) {
        int deg = (int) raw / 100;
        float minutes = raw - deg * 100;
        float decimal = (float) (deg + minutes / 60.0);

        if (hemi == 'S' || hemi == 'W')
            decimal = -decimal;

        return decimal;
    }

    private int indexOf(char[] arr, char target, int start) {
        for (int i = start; arr[i] != '\0'; i++) {
            if (arr[i] == target) {
                return i;   // found
            }
        }
        return -1;
    }


}
