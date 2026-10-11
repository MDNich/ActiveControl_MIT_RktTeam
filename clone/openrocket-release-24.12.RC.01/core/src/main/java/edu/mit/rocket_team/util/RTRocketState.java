package edu.mit.rocket_team.util;

/** Firmware phase codes retained in command packets, telemetry and phase-dependent logic. */
public enum RTRocketState {
    GROUND_TESTING(0),
    PRE_FLIGHT(1),
    FLIGHT(2),
    APOGEE(3),
    MAIN(4),
    END(5);

    public final int ID;

    /**
     * Creates a RTRocketState with the supplied dependencies and per-instance state.
     *
     * @param ID initial id
     */
    RTRocketState(int ID) {
        this.ID = ID;
    }
}
