package edu.mit.rocket_team.zephyrus.util;

public enum RTRocketState {
    GROUND_TESTING(0),
    PRE_FLIGHT(1),
    FLIGHT(2),
    APOGEE(3),
    DISREEF(4),
    END(5);

    public int ID = 0;

    RTRocketState(int ID) {
        this.ID = ID;
    }
}
