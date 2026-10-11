package edu.mit.rocket_team.control.airbrakes;

/** Ordered phases of the included airbrake predictor preparation and control algorithm. */
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
