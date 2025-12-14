package edu.mit.rocket_team.zephyrus.util.data;

public enum RTPyroFireStatus {
    PYRO_READY(1),
    PYRO_ARMED(2),
    PYRO_FIRED(3),
    PYRO_UNAVAIL(4);

    public int ID = -1;

    RTPyroFireStatus(int ID) {
        this.ID = ID;
    }
}
