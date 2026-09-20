package edu.mit.rocket_team.zephyrus.instrument;
import edu.mit.rocket_team.zephyrus.util.*;
import edu.mit.rocket_team.zephyrus.util.data.*;
import static edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.elapsed32;
/** ADXL357.cpp: calibrated acquisition boundary; source X integration is intentional. */
public class RTAccel extends RTInstrument {
    private float x,y,z,velocity,vertical;
    private long lastUpdate;
    private boolean ready;
    private RTAccelData pending;
    public RTAccel() { super(); }
    public RTAccel(RTUtilLibrary.Trace trace) { super(trace); }
    @Override public void setup() { trace.log("accel.setup", "units=mps2 sensor_X=longitudinal"); }
    @Override public void backdoorFudge(RTFudgedData data) { pending=(RTAccelData)data; ready=true; }
    public void update(RTRocketState state) {
        if (!ready) { trace.log("accel.update", "new_data=false"); return; }
        x=pending.getAccelX(); y=pending.getAccelY(); z=pending.getAccelZ(); ready=false;
        vertical=(float)(x-9.8065);
        long now=trace.micros(), dt=elapsed32(now,lastUpdate);
        if (state!=RTRocketState.PRE_FLIGHT || vertical>10)
            velocity += vertical * dt / 1000000.0;
        lastUpdate=now;
        trace.log("accel.update", "new_data=true x="+x+" y="+y+" z="+z+" vertical="+vertical+" velocity="+velocity+" dt_us="+dt);
    }
    public void update() { update(RTRocketState.GROUND_TESTING); }
    public float getAccelX() { return x; }
    public float getAccelY() { return y; }
    public float getAccelZ() { return z; }
    public float getVerticalAccelMinusGravity() { return vertical; }
    public float getIntegratedVelo() { return velocity; }
    public void zeroIntegratedVelo() { velocity=0; trace.log("accel.zero", "velocity=0"); }
    public static float calibratedRaw(int raw, boolean xAxis) {
        int signed=(raw<<12)>>12;
        return (float)(signed/12800.0*9.80665*(xAxis?1.060:1.0));
    }
}
