package edu.mit.rocket_team.util;

import edu.mit.rocket_team.util.data.RTFudgedData;

/** Base for a simulated instrument sharing the run-local virtual clock and trace sink. */
public abstract class RTInstrument {
    protected final RTUtilLibrary.Trace trace;

    /** Creates a RTInstrument using the supplied settings and defaults for omitted options. */
    protected RTInstrument() {
        this(new RTUtilLibrary.Trace());
    }

    /**
     * Creates a RTInstrument with the supplied dependencies and per-instance state.
     *
     * @param trace run-local virtual clock and action-log sink
     */
    protected RTInstrument(RTUtilLibrary.Trace trace) {
        this.trace = trace;
    }

    /** Initializes the simulated device for its run-local configuration. */
    public abstract void setup();

    /**
     * Injects an engineering-unit sensor/control payload at the simulation boundary.
     *
     * @param fudged fudged
     */
    public abstract void backdoorFudge(RTFudgedData fudged);
}
