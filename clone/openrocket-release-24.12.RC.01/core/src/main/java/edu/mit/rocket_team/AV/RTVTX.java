package edu.mit.rocket_team.AV;

import edu.mit.rocket_team.util.RTUtilLibrary.Trace;

/** Logical video-transmitter power adapter; RF propagation is not modeled. */
public class RTVTX {
    private final Trace trace;

    /** Creates a RTVTX using the supplied settings and defaults for omitted options. */
    public RTVTX() {
        this(new Trace());
    }

    /**
     * Creates a RTVTX with the supplied dependencies and per-instance state.
     *
     * @param trace run-local virtual clock and action-log sink
     */
    public RTVTX(Trace trace) {
        this.trace = trace;
    }

    /** Records setup of the logical audiovisual device without modeling video hardware. */
    public void setup() {
        trace.log("rtvtx.setup", "mode=simulated");
    }

    private int power;

    /**
     * Stores and traces the requested logical video-transmitter power level.
     *
     * @param value replacement or input value
     */
    public void setPower(int value) {
        power = value;
        trace.log("vtx.power", "level=" + power);
    }
}
