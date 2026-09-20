package edu.mit.rocket_team.zephyrus.instrument;
import edu.mit.rocket_team.zephyrus.util.*;
import edu.mit.rocket_team.zephyrus.util.data.*;
/** GPS.cpp decoded NAV-PVT boundary, height in millimeters before source zeroing. */
public class RTGPS extends RTInstrument {
    private RTGPSData pending;
    private float heightOffset,maxAlt,pdop,vdop,hdop;
    private int latitudeE7,longitudeE7;
    private int heightMm,fixType;
    public RTGPS() { super(); }
    public RTGPS(RTUtilLibrary.Trace trace) { super(trace); }
    @Override public void setup() { trace.log("gps.setup", "rate_hz=10 datum=simulator_absolute_height"); }
    @Override public void backdoorFudge(RTFudgedData data) { pending=(RTGPSData)data; }
    public void updateAndParse() {
        if(pending!=null) {
            latitudeE7=(int)Math.round(pending.getLatitude()*1e7); longitudeE7=(int)Math.round(pending.getLongitude()*1e7); heightMm=(int)Math.round(pending.getAltitude()*1000);
            fixType=pending.getFixType(); pdop=pending.getPDOP(); vdop=pending.getVDOP(); hdop=pending.getHDOP(); pending=null;
            if(fixType==3 && getHeight()>maxAlt) maxAlt=getHeight();
            trace.log("gps.update", "new_data=true lat_deg="+getLatitude()+" lon_deg="+getLongitude()+" height_mm="+heightMm+" relative_m="+getHeight()+" maximum_m="+maxAlt+" fix_type="+fixType);
        } else trace.log("gps.update", "new_data=false fix_type="+fixType);
    }
    public void zeroAlt() { heightOffset=heightMm; maxAlt=0; trace.log("gps.zero", "offset_mm="+heightOffset); }
    public float getHeight() { return (float)((heightMm-heightOffset)/1000.0); }
    public float getAltitude() { return getHeight(); }
    public float getMaxAlt() { return maxAlt; }
    public int getFixType() { return fixType; }
    public boolean getFix() { return fixType==3; }
    public double getLatitude() { return latitudeE7*1e-7; }
    public int getLatE7() { return latitudeE7; }
    public double getLongitude() { return longitudeE7*1e-7; }
    public int getLonE7() { return longitudeE7; }
    public float getPDOP() { return pdop; }
    public float getVDOP() { return vdop; }
    public float getHDOP() { return hdop; }
}
