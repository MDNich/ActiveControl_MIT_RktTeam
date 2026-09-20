package edu.mit.rocket_team.zephyrus;
import edu.mit.rocket_team.zephyrus.FC.RTFC;
import edu.mit.rocket_team.zephyrus.util.data.*;
import edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.Trace;
import static edu.mit.rocket_team.zephyrus.util.RTRocketState.*;
final class RTTestInputs {
    static RTFC.Inputs pad(long us,float ax,float altitude,int fix) {
        RTGPSData gps=new RTGPSData(42f,-77f,altitude,0,0,0,true); gps.setFixType(fix);
        return new RTFC.Inputs(us,new RTAccelData(ax,0,0),new RTBaroData(Float.NaN,Float.NaN,20,Float.NaN,1013.25f),gps,new RTGyroData(0,0,0));
    }
    static void tick(RTFC c,long ms,float ax,float altitude,int fix) { c.pre_loop(ms*1000,pad(ms*1000,ax,altitude,fix)); c.loop(); }
    static RTFC fresh() { RTFC c=new RTFC(new Trace(s->{})); c.init(); return c; }
    static RTFC flying() {
        RTFC c=fresh(); c.enqueueCommand(RTFC.stateCommand(PRE_FLIGHT)); tick(c,0,9.8065f,0,3); tick(c,10,9.8065f,0,3);
        c.enqueueCommand(RTFC.stateCommand(FLIGHT)); tick(c,20,9.8065f,0,3); tick(c,30,9.8065f,0,3); return c;
    }
}
