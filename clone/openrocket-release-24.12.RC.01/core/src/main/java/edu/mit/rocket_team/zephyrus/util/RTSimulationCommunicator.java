package edu.mit.rocket_team.zephyrus.util;

import info.openrocket.core.rocketcomponent.AirbrakeSet;
import info.openrocket.core.rocketcomponent.TabControlledTrapezoidFinSet;

public class RTSimulationCommunicator {

    // everything is static in this class
    public static AirbrakeSet airbrakes;
    public static TabControlledTrapezoidFinSet fins;

    public static void setAirbrakes(AirbrakeSet airbrakes) {
        RTSimulationCommunicator.airbrakes = airbrakes;
    }

    public static void setTabCtrlFins(TabControlledTrapezoidFinSet fins) {
        RTSimulationCommunicator.fins = fins;
    }

    public static void actuateAirbrakesServo(double deployedFraction) {
        airbrakes.setFracExposed(deployedFraction);
    }
    public static void actuateRollControlServo(double angle) {
        fins.setTabAngle(angle);
    }
}
