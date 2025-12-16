package edu.mit.rocket_team.zephyrus.util.data;

public enum RTPyroStatus {
    PYRO_FAILURE(0),
    PYRO_UNCONNECTED(1),
    PYRO_CONNECTED(2),
    PYRO_SUCCESS(3);

    public int ID = 0;

    RTPyroStatus(int ID) {
        this.ID = ID;
    }
}
