package edu.mit.rocket_team.zephyrus.AV;
import edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.Trace;
public class RTCamera {
    private final Trace trace;
    public RTCamera() { this(new Trace()); }
    public RTCamera(Trace trace) { this.trace=trace; }
    public void setup() { trace.log("rtcamera.setup", "mode=simulated"); }
    
}
