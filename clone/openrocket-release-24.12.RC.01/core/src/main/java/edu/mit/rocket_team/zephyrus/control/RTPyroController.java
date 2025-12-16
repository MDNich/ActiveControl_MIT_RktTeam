package edu.mit.rocket_team.zephyrus.control;

import edu.mit.rocket_team.zephyrus.util.RTController;
import edu.mit.rocket_team.zephyrus.util.data.RTPyroStatus;
import edu.mit.rocket_team.zephyrus.util.RTRocketState;
import info.openrocket.core.simulation.SimulationStatus;

public class RTPyroController extends RTController {

    public static final int NUM_PYROS = 12;

    public static SimulationStatus SIMULATION_STATUS;

    int pyroStatuses;
    double[] fire_times;
    boolean[] fired;
    boolean[] armed;
    //For Sim Use Only
    boolean[] shouldMisfire;
    boolean[] hasFired;

    public RTPyroController() {
        fire_times   = new double[NUM_PYROS];
        fired        = new boolean[NUM_PYROS];
        armed        = new boolean[NUM_PYROS];
        shouldMisfire= new boolean[NUM_PYROS];
        hasFired     = new boolean[NUM_PYROS];
        pyroStatuses = 0;
    }

    @Override
    public void setup() {
        for (int i = 0; i < NUM_PYROS; i++) {
            setPyroStatus(i,RTPyroStatus.PYRO_CONNECTED); // assume ready, fudgable later
        }
    }

    public void armPyro(int pyro) {
        armed[pyro] = true;
    }

    public void disarmPyro(int pyro) {
        armed[pyro] = false;
    }

    public void pyroMonitor(RTRocketState currentState) {
        for (int i = 0; i < NUM_PYROS; i++) {
            if (currentState == RTRocketState.GROUND_TESTING) {
                //In ground testing, we only care if pyro is connected or disconnected
                if (isPyroConnected(i)) {
                    setPyroStatus(i, RTPyroStatus.PYRO_CONNECTED);
                } else {
                    setPyroStatus(i, RTPyroStatus.PYRO_UNCONNECTED);
                }
            } else {
                //Otherwise, an unconnected pyro is a failure, unless it has been successfuly fired
                if(!isPyroConnected(i) && getPyroStatus(i) != RTPyroStatus.PYRO_SUCCESS && !fired[i]) {
                    setPyroStatus(i, RTPyroStatus.PYRO_FAILURE);
                }
            }

            if (fired[i] && (millis() - fire_times[i]) > 1000) {
                //digitalWrite(firePins[i], 0);
                if(!isPyroConnected(i) &&  getPyroStatus(i) != RTPyroStatus.PYRO_FAILURE) {
                    setPyroStatus(i, RTPyroStatus.PYRO_SUCCESS);
                } else {
                    setPyroStatus(i, RTPyroStatus.PYRO_FAILURE);
                }
                fired[i] = false;
            }
        }
    }

    public void firePyro(int pyro) {
        if (armed[pyro]) {
            //digitalWrite(firePins[pyro], 1);
            fired[pyro] = true;
            hasFired[pyro] = true;
            armed[pyro] = false;
            fire_times[pyro] = millis();
        }
    }

    private boolean isPyroConnected(int pyro) {
        // placeholder for onboard logic to check pyro connectivity
        return  !(hasFired[pyro] & !shouldMisfire[pyro]);
    }

    private RTPyroStatus getPyroStatus(int pyro) {
        return RTPyroStatus.values()[(pyroStatuses >> (pyro * 2)) & 0b11];
    }

    private void setPyroStatus(int pyro, RTPyroStatus status) {
        int mask = 0b11 << (pyro * 2);
        pyroStatuses &= ~mask;
        pyroStatuses |= (status.ID & 0b11) << (pyro * 2);
    }

    // For sim use only
    private int millis() {
        if (SIMULATION_STATUS != null) {
            return (int)(SIMULATION_STATUS.getSimulationTime()*1e3); // seconds to miliseconds
        } else {
            return (int)System.currentTimeMillis();
        }
    }

    // TODO : write Pyro Controller Fudging Procedure
    // TODO : include misfire fudging
    // TODO : make sure to provide SIMULATION_STATUS in order to have millis() work properly
}
