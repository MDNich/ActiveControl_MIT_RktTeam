package edu.mit.rocket_team.zephyrus.util;
import edu.mit.rocket_team.zephyrus.FC.RTFC;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.simulation.exception.SimulationException;
/** Physical output of the existing FC, with ideal motion after PWM latching. */
public class RTSimulationCommunicator {
    private AirbrakeSet airbrakes;
    private final RTUtilLibrary.Trace trace;
    public RTSimulationCommunicator(RTUtilLibrary.Trace trace) { this.trace=trace; }
    public void bind(SimulationStatus status) throws SimulationException {
        AirbrakeSet selected=null;
        for(RocketComponent component:status.getConfiguration().getActiveComponents()) if(component instanceof AirbrakeSet set) {
            if(selected!=null) throw new SimulationException("FC requires exactly one active AirbrakeSet"); selected=set;
        }
        if(selected==null) throw new SimulationException("FC requires an active AirbrakeSet");
        if(airbrakes!=selected) { airbrakes=selected; trace.log("component.bind", "airbrakes="+selected.getName()); }
    }
    public void apply(RTFC.Output output,boolean holdClosed) {
        double fraction=holdClosed?0:output.exposedFraction();
        airbrakes.setFracExposed(fraction);
        trace.log("airbrakes.actuate", "exposed="+fraction+" pulse_us="+output.airbrakePulseUs()+" ideal_linkage=true hold_closed="+holdClosed);
    }
}
