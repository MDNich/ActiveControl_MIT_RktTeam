package edu.mit.rocket_team.zephyrus.internal;
import java.util.Arrays;
import edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.Trace;
/** FC pwrCommands: source converter order, deterministic nominal readings. */
public class RTPowerBoard {
    private final Trace trace;
    private final boolean[] enabled={true,true,true,true,true,true};
    private final double[] volts={3,3.3,5,7.4,8.4,28};
    private boolean protections=true,screwSwitch=true;
    public RTPowerBoard() { this(new Trace()); }
    public RTPowerBoard(Trace trace) { this.trace=trace; }
    public void setup() { trace.log("power.setup", "mode=nominal_simulated converters=3,3.3,5,7.4,8.4,28V"); }
    public void update() { trace.log("power.update", "status=nominal"); }
    public void setConverter(int i,boolean value) { enabled[i]=value; trace.log("power.converter", "index="+i+" enabled="+value); }
    public void enableAll() { for(int i=0;i<6;i++) setConverter(i,true); }
    public boolean isEnabled(int i) { return enabled[i]; }
    public void setProtections(boolean value) { protections=screwSwitch=value; trace.log("power.protection", "protections="+value+" screw_switch="+value); }
    public boolean protectionsEnabled() { return protections; }
    public void sendCommand() { trace.log("power.command", "converters="+Arrays.toString(enabled)+" protections="+protections+" screw_switch="+screwSwitch); }
    public double getVoltageByRail(int i) { return enabled[i]?volts[i]:0; }
    public double getCurrentByRail(int i) { return 0; }
    public double getCellVoltage(int i) { return 3.7; }
    public double getTotalBatteryVoltage() { return 11.1; }
}
