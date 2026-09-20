package edu.mit.rocket_team.zephyrus.AV;
import edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.Trace;
public class RTVTX {
    private final Trace trace;
    public RTVTX() { this(new Trace()); }
    public RTVTX(Trace trace) { this.trace=trace; }
    public void setup() { trace.log("rtvtx.setup", "mode=simulated"); }
    private int power;
    public void setPower(int value) { power=value; trace.log("vtx.power", "level="+power); }
}
