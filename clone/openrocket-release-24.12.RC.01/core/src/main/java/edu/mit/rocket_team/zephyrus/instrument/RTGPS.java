package edu.mit.rocket_team.zephyrus.instrument;
import edu.mit.rocket_team.zephyrus.util.*;
import edu.mit.rocket_team.zephyrus.util.data.*;
/** GPS.cpp decoded NAV-PVT boundary, height in millimeters before source zeroing. */
public class RTGPS extends RTInstrument {
    private RTGPSData pending;
    private float latitude,longitude,heightOffset,maxAlt,pdop,vdop,hdop;
    private int heightMm,fixType;
    public RTGPS() { super(); }
    public RTGPS(RTUtilLibrary.Trace trace) { super(trace); }
    @Override public void setup() { trace.log("gps.setup", "rate_hz=10 datum=simulator_absolute_height"); }
    @Override public void backdoorFudge(RTFudgedData data) { pending=(RTGPSData)data; }
    public void updateAndParse() {
        if(pending!=null) {
            latitude=pending.getLatitude(); longitude=pending.getLongitude(); heightMm=Math.round(pending.getAltitude()*1000);
            fixType=pending.getFixType(); pdop=pending.getPDOP(); vdop=pending.getVDOP(); hdop=pending.getHDOP(); pending=null;
            if(fixType==3 && getHeight()>maxAlt) maxAlt=getHeight();
            trace.log("gps.update", "new_data=true lat_deg="+latitude+" lon_deg="+longitude+" height_mm="+heightMm+" relative_m="+getHeight()+" maximum_m="+maxAlt+" fix_type="+fixType);
        } else trace.log("gps.update", "new_data=false fix_type="+fixType);
    }
    public void zeroAlt() { heightOffset=heightMm; maxAlt=0; trace.log("gps.zero", "offset_mm="+heightOffset); }
    public float getHeight() { return (float)((heightMm-heightOffset)/1000.0); }
    public float getAltitude() { return getHeight(); }
    public float getMaxAlt() { return maxAlt; }
    public int getFixType() { return fixType; }
    public boolean getFix() { return fixType==3; }
    public float getLatitude() { return latitude; }
    public float getLongitude() { return longitude; }
    public float getPDOP() { return pdop; }
    public float getVDOP() { return vdop; }
    public float getHDOP() { return hdop; }
}
