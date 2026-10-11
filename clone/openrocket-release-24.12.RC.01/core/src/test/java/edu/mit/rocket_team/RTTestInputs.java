package edu.mit.rocket_team;

import edu.mit.rocket_team.FC.RTFC;
import edu.mit.rocket_team.util.data.*;
import edu.mit.rocket_team.util.RTUtilLibrary.Trace;
import static edu.mit.rocket_team.util.RTRocketState.*;

final class RTTestInputs {
    /**
     * Creates a stationary launch-pad sensor acquisition.
     *
     * @param us in microseconds
     * @param ax ax
     * @param altitude altitude
     * @param fix fix
     * @return the resulting Inputs
     */
    static RTFC.Inputs pad(long us, float ax, float altitude, int fix) {
        RTGPSData gps = new RTGPSData(42f, -77f, altitude, 0, 0, 0, true);
        gps.setFixType(fix);
        return new RTFC.Inputs(
                us,
                new RTAccelData(ax, 0, 0),
                new RTBaroData(Float.NaN, Float.NaN, 20, Float.NaN, 1013.25f),
                gps,
                new RTGyroData(0, 0, 0));
    }

    /**
     * Runs one FC iteration with the supplied virtual time and sensor data.
     *
     * @param c selected channel or controller
     * @param ms in milliseconds
     * @param ax ax
     * @param altitude altitude
     * @param fix fix
     */
    static void tick(RTFC c, long ms, float ax, float altitude, int fix) {
        c.pre_loop(ms * 1000, pad(ms * 1000, ax, altitude, fix));
        c.loop();
    }

    /**
     * Creates a fresh FC instance with test inputs and an isolated trace.
     *
     * @return the resulting RTFC
     */
    static RTFC fresh() {
        RTFC c = new RTFC(new Trace(s -> {}));
        c.init();
        return c;
    }

    /**
     * Creates measured sensor inputs representing the test flight segment.
     *
     * @return the resulting RTFC
     */
    static RTFC flying() {
        RTFC c = fresh();
        c.enqueueCommand(RTFC.stateCommand(PRE_FLIGHT));
        tick(c, 0, 9.8065f, 0, 3);
        tick(c, 10, 9.8065f, 0, 3);
        c.enqueueCommand(RTFC.stateCommand(FLIGHT));
        tick(c, 20, 9.8065f, 0, 3);
        tick(c, 30, 9.8065f, 0, 3);
        return c;
    }
}
