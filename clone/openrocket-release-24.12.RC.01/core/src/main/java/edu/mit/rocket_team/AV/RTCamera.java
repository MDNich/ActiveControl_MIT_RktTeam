package edu.mit.rocket_team.AV;

import edu.mit.rocket_team.util.RTUtilLibrary.Trace;

/** Logical camera adapter that records setup without capturing video. */
public class RTCamera {
    private final Trace trace;

    /** Creates a RTCamera using the supplied settings and defaults for omitted options. */
    public RTCamera() {
        this(new Trace());
    }

    /**
     * Creates a RTCamera with the supplied dependencies and per-instance state.
     *
     * @param trace run-local virtual clock and action-log sink
     */
    public RTCamera(Trace trace) {
        this.trace = trace;
    }

    /** Records setup of the logical audiovisual device without modeling video hardware. */
    public void setup() {
        trace.log("rtcamera.setup", "mode=simulated");
    }
}
