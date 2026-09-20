package edu.mit.rocket_team.zephyrus.util;

import edu.mit.rocket_team.zephyrus.util.data.RTFudgedData;

public abstract class RTInstrument {
    protected final RTUtilLibrary.Trace trace;
    protected RTInstrument() { this(new RTUtilLibrary.Trace()); }
    protected RTInstrument(RTUtilLibrary.Trace trace) { this.trace = trace; }
    public abstract void setup();

    public abstract void backdoorFudge(RTFudgedData fudged);
}
