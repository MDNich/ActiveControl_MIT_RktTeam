package edu.mit.rocket_team.internal;

import java.util.Arrays;
import edu.mit.rocket_team.util.RTUtilLibrary.Trace;

/** FC pwrCommands: source converter order, deterministic nominal readings. */
public class RTPowerBoard {
    private final Trace trace;
    private final boolean[] enabled = {true, true, true, true, true, true};
    private final double[] volts = {3, 3.3, 5, 7.4, 8.4, 28};
    private boolean protections = true, screwSwitch = true;

    /** Creates a RTPowerBoard using the supplied settings and defaults for omitted options. */
    public RTPowerBoard() {
        this(new Trace());
    }

    /**
     * Creates a RTPowerBoard with the supplied dependencies and per-instance state.
     *
     * @param trace run-local virtual clock and action-log sink
     */
    public RTPowerBoard(Trace trace) {
        this.trace = trace;
    }

    /** Records initialization of the logical power-board model. */
    public void setup() {
        trace.log("power.setup", "mode=nominal_simulated converters=3,3.3,5,7.4,8.4,28V");
    }

    /** Runs the modeled power-board polling hook without electrical dynamics. */
    public void update() {
        trace.log("power.update", "status=nominal");
    }

    /**
     * Changes the enabled state of a simulated converter rail.
     *
     * @param i zero-based rail or cell index
     * @param value replacement or input value
     */
    public void setConverter(int i, boolean value) {
        enabled[i] = value;
        trace.log("power.converter", "index=" + i + " enabled=" + value);
    }

    /** Enables every modeled converter rail. */
    public void enableAll() {
        for (int i = 0; i < 6; i++) setConverter(i, true);
    }

    /**
     * Returns whether the selected simulated converter rail is enabled.
     *
     * @param i zero-based rail or cell index
     * @return whether the selected simulated converter rail is enabled
     */
    public boolean isEnabled(int i) {
        return enabled[i];
    }

    /**
     * Changes the modeled battery-protection enable flag.
     *
     * @param value replacement or input value
     */
    public void setProtections(boolean value) {
        protections = screwSwitch = value;
        trace.log("power.protection", "protections=" + value + " screw_switch=" + value);
    }

    /**
     * Returns the modeled battery-protection enable flag.
     *
     * @return the modeled battery-protection enable flag
     */
    public boolean protectionsEnabled() {
        return protections;
    }

    /** Traces the current power-board command state. */
    public void sendCommand() {
        trace.log(
                "power.command",
                "converters="
                        + Arrays.toString(enabled)
                        + " protections="
                        + protections
                        + " screw_switch="
                        + screwSwitch);
    }

    /**
     * Returns the model's fixed voltage for the selected rail.
     *
     * @param i zero-based rail or cell index
     * @return the model's fixed voltage for the selected rail
     */
    public double getVoltageByRail(int i) {
        return enabled[i] ? volts[i] : 0;
    }

    /**
     * Returns the model's fixed current for the selected rail.
     *
     * @param i zero-based rail or cell index
     * @return the model's fixed current for the selected rail
     */
    public double getCurrentByRail(int i) {
        return 0;
    }

    /**
     * Returns the model's fixed cell voltage.
     *
     * @param i zero-based rail or cell index
     * @return the model's fixed cell voltage
     */
    public double getCellVoltage(int i) {
        return 3.7;
    }

    /**
     * Returns the model's fixed total battery voltage.
     *
     * @return the model's fixed total battery voltage
     */
    public double getTotalBatteryVoltage() {
        return 11.1;
    }
}
