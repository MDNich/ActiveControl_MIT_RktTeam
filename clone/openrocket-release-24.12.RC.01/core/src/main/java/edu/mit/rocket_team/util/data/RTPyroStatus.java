package edu.mit.rocket_team.util.data;

/** Two-bit firmware continuity/firing status values used in telemetry. */
public enum RTPyroStatus {
    PYRO_FAILURE(0),
    PYRO_UNCONNECTED(1),
    PYRO_CONNECTED(2),
    PYRO_SUCCESS(3);

    public int ID = 0;

    /**
     * Creates a RTPyroStatus with the supplied dependencies and per-instance state.
     *
     * @param ID initial id
     */
    RTPyroStatus(int ID) {
        this.ID = ID;
    }
}
