package info.openrocket.swing.gui.dialogs.motor.thrustcurve;

import info.openrocket.core.database.motor.ThrustCurveMotorSet;
import info.openrocket.core.motor.ThrustCurveMotor;
import info.openrocket.core.preferences.ApplicationPreferences;
import info.openrocket.core.startup.Application;
import info.openrocket.swing.gui.util.SwingPreferences;
import java.util.Comparator;

/**
 * One policy for the normal motor picker and batch simulation. Invoke while capturing UI inputs.
 */
public final class PreferredMotorCurve {
    /** Prevents instantiation of this utility class. */
    private PreferredMotorCurve() {}

    /**
     * Chooses the preferred thrust curve among equivalent motor entries.
     *
     * @param set set
     * @return the resulting ThrustCurveMotor
     */
    public static ThrustCurveMotor select(ThrustCurveMotorSet set) {
        var prefs =
                ((SwingPreferences) Application.getPreferences())
                        .getNode(ApplicationPreferences.PREFERRED_THRUST_CURVE_MOTOR_NODE);
        return set.getMotors().stream()
                .min(
                        Comparator.comparing(
                                        (ThrustCurveMotor m) ->
                                                !prefs.getBoolean(m.getDigest(), false))
                                .thenComparing(new ThrustCurveMotorComparator())
                                .thenComparing(ThrustCurveMotor::getDigest))
                .orElseThrow(() -> new IllegalArgumentException("Empty motor set"));
    }
}
