package edu.mit.rocket_team.zephyrus.util;

import edu.mit.rocket_team.zephyrus.util.data.RTFudgedAirbrakesData;
import edu.mit.rocket_team.zephyrus.util.data.RTFudgedData;

public abstract class RTController {
    public abstract void setup();
    public abstract void performLoopAction();
    public abstract void backdoorFudge(RTFudgedData data);
}
