package edu.mit.rocket_team.zephyrus.util;

import edu.mit.rocket_team.zephyrus.util.data.RTFudgedData;

public abstract class RTInstrument {
    public abstract void setup();

    public abstract void backdoorFudge(RTFudgedData fudged);
}
