package edu.mit.rocket_team.util;

import edu.mit.rocket_team.util.data.RTFudgedAirbrakesData;
import edu.mit.rocket_team.util.data.RTFudgedData;

/**
 * Common setup, loop and input-injection contract for translated firmware controllers. Typed
 * controllers may reject generic hooks when they require explicit timing or input arguments.
 */
public abstract class RTController {
    /** Initializes the simulated device for its run-local configuration. */
    public abstract void setup();

    /** Performs the device's work for the current FC loop. */
    public abstract void performLoopAction();

    /**
     * Injects an engineering-unit sensor/control payload at the simulation boundary.
     *
     * @param data input data for this operation
     */
    public abstract void backdoorFudge(RTFudgedData data);
}
