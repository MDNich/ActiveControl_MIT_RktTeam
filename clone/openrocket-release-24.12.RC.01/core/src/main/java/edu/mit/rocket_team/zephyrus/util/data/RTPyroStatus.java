package edu.mit.rocket_team.zephyrus.util.data;

public enum RTPyroStatus {
    PYRO_FAILURE(1),
    PYRO_UNCONNECTED(2),
    PYRO_CONNECTED(3),
    PYRO_ARMED(4),
    PYRO_SUCCESS(5),
    PYRO_FIRING(6);

    public int ID = -1;

    RTPyroStatus(int ID) {
        this.ID = ID;
    }
}
