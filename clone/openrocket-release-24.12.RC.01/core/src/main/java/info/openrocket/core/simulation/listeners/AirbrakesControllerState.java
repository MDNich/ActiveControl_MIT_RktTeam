package info.openrocket.core.simulation.listeners;

public enum AirbrakesControllerState{
    DISABLED,
    PREP,
    PREPROCESS,
    WAIT_FOR_START,
    CONTROLLING_RAMP,
    CONTROLLING_PLATEAU,
    DONE,
    INFEASIBLE
}
