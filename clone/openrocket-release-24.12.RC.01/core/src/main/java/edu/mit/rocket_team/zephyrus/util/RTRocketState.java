package edu.mit.rocket_team.zephyrus.util;

public enum RTRocketState {
    GROUND_TESTING(0),
    PRE_FLIGHT(1),
    FLIGHT(2),
    APOGEE(3),
    MAIN(4),
    END(5);

    public final int ID;

    RTRocketState(int ID) {
        this.ID = ID;
    }
}
