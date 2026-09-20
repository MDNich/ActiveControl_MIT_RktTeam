package edu.mit.rocket_team.zephyrus.instrument;
import edu.mit.rocket_team.zephyrus.util.*;
import edu.mit.rocket_team.zephyrus.util.data.*;
/** baro.cpp: engineering-unit acquisition precedes the unchanged altitude/filter math. */
public class RTBaro extends RTInstrument {
    private float pressure=1013.25f,temperature=20,rawPressure=Float.NaN,rawTemperature=Float.NaN;
    private final float[] samples=new float[20];
    private float filtered,offset,maxAlt;
    public RTBaro() { super(); }
    public RTBaro(RTUtilLibrary.Trace trace) { super(trace); }
    @Override public void setup() { trace.log("baro.setup", "samples=20 pressure=hPa temperature=C raw=unavailable"); }
    @Override public void backdoorFudge(RTFudgedData data) {
        RTBaroData d=(RTBaroData)data;
        pressure=d.getPressure(); temperature=d.getTemperature();
        rawPressure=d.getRawPressure(); rawTemperature=d.getRawTemperature();
        if (!(pressure>0) || !Float.isFinite(temperature)) throw new IllegalArgumentException("Invalid barometer observation");
    }
    public void updateAll() {
        System.arraycopy(samples,0,samples,1,19); samples[0]=getAltitude(); filtered=0;
        for(float s:samples) filtered+=s;
        filtered/=20;
        if(getFilteredAltitude()>maxAlt) maxAlt=getFilteredAltitude();
        trace.log("baro.update", "pressure_hPa="+pressure+" temperature_C="+temperature+" altitude_m="+samples[0]+" filtered_m="+getFilteredAltitude()+" maximum_m="+maxAlt);
    }
    public float getAltitude() { float p=1013.25f/pressure; return (float)(153.84615*(Math.pow(p,0.19)-1)*(temperature+273.15)); }
    public float getFilteredAltitude() { return filtered-offset; }
    public void zeroAlt() { offset=getFilteredAltitude(); maxAlt=0; trace.log("baro.zero", "offset_m="+offset); }
    public void resetMaxAlt() { maxAlt=0; trace.log("baro.reset_max", "maximum_m=0"); }
    public float getMaxAlt() { return maxAlt; }
    public float getPressure() { return pressure; }
    public float getTemperature() { return temperature; }
    public float getRawPressure() { return rawPressure; }
    public float getRawTemperature() { return rawTemperature; }
    public void startPressureConversion() { trace.log("baro.conversion", "kind=pressure modeled_delay=0"); }
    public void startTemperatureConversion() { trace.log("baro.conversion", "kind=temperature modeled_delay=0"); }
    /** baro.cpp compensation constants, for independent raw replay inputs. */
    public static RTBaroData fromRaw(int rawP,int rawT) {
        float dT=(float)rawT-(float)0x8405*256;
        float temp=(float)(2000.0+dT*(float)0x6D91/(float)(1L<<23));
        long off=(long)(((long)0x953A)*(1L<<17)+dT*(float)0x6305/(1<<6));
        long sens=(long)((float)0xA579*(1L<<16)+dT*(float)0x68AC/(1<<7));
        float pa=(float)rawP/(1L<<15),pb=sens/(float)(1L<<21),pc=pa*pb,pd=off/(float)(1L<<15);
        return new RTBaroData((float)rawP,(float)rawT,temp/100,Float.NaN,(pc-pd)/100);
    }
}
