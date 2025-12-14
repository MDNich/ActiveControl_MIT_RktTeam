package edu.mit.rocket_team.zephyrus.util.data;

public enum RTPyroConnectStatus {
    PYRO_FAILURE(1),
    PYRO_UNCONNECTED(2),
    PYRO_CONNECTED(3),
    PYRO_SUCCESS(4);

    public int ID = -1;

    RTPyroConnectStatus(int ID) {
        this.ID = ID;
    }
}
