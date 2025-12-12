package edu.mit.rocket_team.zephyrus.FC;

import info.openrocket.core.simulation.SimulationStatus;

// Flight computer code goes here.
public class FC {

    public static SimulationStatus currentFCsimStat;

    public static void init() {
        // FC code from C.
    }

    public static void pre_loop(SimulationStatus fudgedStat) {
        currentFCsimStat = fudgedStat.clone();
        // Tell all sensors what to report here, according to the fudged status.
    }

    public static void loop() {
        // FC code from C
    }
}
