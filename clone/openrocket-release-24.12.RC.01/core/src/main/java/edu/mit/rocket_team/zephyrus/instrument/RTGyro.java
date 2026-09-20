package edu.mit.rocket_team.zephyrus.instrument;
import edu.mit.rocket_team.zephyrus.util.*;
import edu.mit.rocket_team.zephyrus.util.data.*;
import static edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.elapsed32;
/** gyro.cpp: source negative-X roll and micros-based integration. */
public class RTGyro extends RTInstrument {
    private float x,y,z,roll,pitch,yaw,angleFromVertical;
    private long lastUpdate;
    public RTGyro() { super(); }
    public RTGyro(RTUtilLibrary.Trace trace) { super(trace); }
    @Override public void setup() { trace.log("gyro.setup", "rates=dps roll_sign=negative_X"); }
    @Override public void backdoorFudge(RTFudgedData data) {
        RTGyroData d=(RTGyroData)data; x=d.getGyroX(); y=d.getGyroY(); z=d.getGyroZ();
    }
    public void update() {
        long now=trace.micros(),dt=elapsed32(now,lastUpdate);
        roll+=x*-1.0*dt/1000000.0; pitch+=y*dt/1000000.0; yaw+=z*dt/1000000.0;
        angleFromVertical=(float)(Math.acos(Math.cos(pitch*Math.PI/180.0)*Math.cos(yaw*Math.PI/180))*180.0/Math.PI);
        lastUpdate=now;
        trace.log("gyro.update", "x_dps="+x+" y_dps="+y+" z_dps="+z+" roll_deg="+roll+" pitch_deg="+pitch+" yaw_deg="+yaw+" dt_us="+dt);
    }
    public void zeroRollPitchYaw() { roll=pitch=yaw=0; trace.log("gyro.zero", "roll_pitch_yaw=0"); }
    public float getRoll() { return roll; }
    public float getPitch() { return pitch; }
    public float getYaw() { return yaw; }
    public float getRollRate() { return -x; }
    public float getAngleFromVertical() { return angleFromVertical; }
    public float getGyroX() { return x; }
    public float getGyroY() { return y; }
    public float getGyroZ() { return z; }
    public static RTGyroData fromRaw(short x,short y,short z) {
        return new RTGyroData((float)(x*0.03051757812-0.2525f),(float)(y*0.03051757812-0.2441f),(float)(z*0.03051757812-0.4376f));
    }
}
