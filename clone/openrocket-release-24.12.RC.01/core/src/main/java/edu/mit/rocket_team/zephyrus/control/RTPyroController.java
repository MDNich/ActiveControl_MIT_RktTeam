package edu.mit.rocket_team.zephyrus.control;
import java.util.Arrays;
import edu.mit.rocket_team.zephyrus.util.*;
import edu.mit.rocket_team.zephyrus.util.data.*;
import static edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.elapsed32;
/** pyro.cpp: logical outputs only; no recovery-device actuation. */
public class RTPyroController extends RTController {
    public static final int NUM_PYROS=6, FIRE_DURATION_MS=250;
    private final RTUtilLibrary.Trace trace;
    private final boolean[] armed=new boolean[6],fired=new boolean[6],connected=new boolean[6];
    private final long[] firedTimes=new long[6];
    private final RTPyroStatus[] status=new RTPyroStatus[6];
    public RTPyroController() { this(new RTUtilLibrary.Trace()); }
    public RTPyroController(RTUtilLibrary.Trace trace) { this.trace=trace; Arrays.fill(connected,true); Arrays.fill(status,RTPyroStatus.PYRO_FAILURE); }
    private void check(int c) { if(c<0 || c>=6) throw new IllegalArgumentException("Pyro channel must be 0..5: "+c); }
    @Override public void setup() { trace.log("pyro.setup", "channels=6 recovery=recorded_only continuity=connected"); }
    public void armPyro(int c) { check(c); armed[c]=true; trace.log("pyro.arm", "channel="+c); }
    public void disarmPyro(int c) { check(c); armed[c]=false; trace.log("pyro.disarm", "channel="+c); }
    public void firePyro(int c) {
        check(c); if(!armed[c]) { trace.log("pyro.fire_ignored", "channel="+c+" reason=unarmed"); return; }
        firedTimes[c]=trace.millis(); fired[c]=true; armed[c]=false;
        trace.log("pyro.fire", "channel="+c+" duration_ms="+FIRE_DURATION_MS);
    }
    public void off(int c) { check(c); fired[c]=false; trace.log("pyro.off", "channel="+c); }
    public void setConnected(int c,boolean value) { check(c); connected[c]=value; trace.log("pyro.continuity", "channel="+c+" connected="+value); }
    public boolean isArmed(int c) { check(c); return armed[c]; }
    public boolean isFired(int c) { check(c); return fired[c]; }
    public RTPyroStatus getPyroStatus(int c) { check(c); return status[c]; }
    public int getPyrosStatus() { int bits=0; for(int i=0;i<6;i++) bits|=status[i].ID<<(2*i); return bits; }
    public void pyroMonitor(RTRocketState state) {
        for(int i=0;i<6;i++) {
            RTPyroStatus old=status[i];
            if(fired[i] && elapsed32(trace.millis(),firedTimes[i])>FIRE_DURATION_MS) {
                off(i); status[i]=!connected[i] && status[i]!=RTPyroStatus.PYRO_FAILURE ? RTPyroStatus.PYRO_SUCCESS : RTPyroStatus.PYRO_FAILURE;
            }
            if(state==RTRocketState.GROUND_TESTING) status[i]=connected[i]?RTPyroStatus.PYRO_CONNECTED:RTPyroStatus.PYRO_UNCONNECTED;
            else if(!connected[i] && status[i]!=RTPyroStatus.PYRO_SUCCESS && !fired[i]) status[i]=RTPyroStatus.PYRO_FAILURE;
            if(old!=status[i]) trace.log("pyro.status", "channel="+i+" from="+old+" to="+status[i]);
        }
        trace.log("pyro.update", "status_bits="+getPyrosStatus());
    }
    @Override public void performLoopAction() { throw new IllegalStateException("Use pyroMonitor with FC state"); }
    @Override public void backdoorFudge(RTFudgedData data) { throw new UnsupportedOperationException("Use setConnected"); }
}
