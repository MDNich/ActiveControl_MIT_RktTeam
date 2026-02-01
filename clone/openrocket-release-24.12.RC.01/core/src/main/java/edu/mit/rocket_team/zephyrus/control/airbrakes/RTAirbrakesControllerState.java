package edu.mit.rocket_team.zephyrus.control.airbrakes;

public enum RTAirbrakesControllerState {
    DISABLED,
    PREP,
    PREPROCESS,
    WAIT_FOR_START,
    CONTROLLING_RAMP,
    CONTROLLING_PLATEAU,
    DONE,
    INFEASIBLE
}
