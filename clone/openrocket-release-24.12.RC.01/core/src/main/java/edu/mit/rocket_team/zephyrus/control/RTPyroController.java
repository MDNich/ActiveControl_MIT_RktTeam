package edu.mit.rocket_team.zephyrus.control;

import edu.mit.rocket_team.zephyrus.util.RTController;
import edu.mit.rocket_team.zephyrus.util.data.RTPyroStatus;
import info.openrocket.core.simulation.SimulationStatus;

public class RTPyroController extends RTController {

    public static final int NUM_PYROS = 12;

    public static SimulationStatus SIMULATION_STATUS;

    RTPyroStatus[] pyroStatuses;
    double[] fire_times;
    boolean[] fired;

    public RTPyroController() {
        pyroStatuses = new RTPyroStatus[NUM_PYROS];
        fire_times   = new double[NUM_PYROS];
        fired        = new boolean[NUM_PYROS];
    }

    @Override
    public void setup() {
        for (int i = 0; i < NUM_PYROS; i++) {
            setPyroStatus(i,RTPyroStatus.PYRO_CONNECTED); // assume ready, fudgable later
        }
    }

    public RTPyroStatus getPyroStatus(int pyro) {
        pyroMonitor(pyro);
        return pyroStatuses[pyro];
    }

    public void armPyro(int pyro) {
        if (isPyroConnected(pyro)) {
            if (getPyroFireStatus(pyro) == RTPyroStatus.PYRO_CONNECTED) {
                setPyroStatus(pyro, RTPyroStatus.PYRO_ARMED);
            }
        }
    }
    public void disarmPyro(int pyro) {
        if (isPyroConnected(pyro)) {
            if (getPyroFireStatus(pyro) == RTPyroStatus.PYRO_ARMED) {
                setPyroStatus(pyro, RTPyroStatus.PYRO_CONNECTED);
            }
        }
    }

    public void pyroMonitor(int pyro) {
        pyroMonitor(pyro, false);
    }

    private void pyroMonitor(int pyro, boolean shouldMisfire) {
        if (getPyroFireStatus(pyro) == RTPyroStatus.PYRO_FIRING) {
            firePyroAction(pyro);
            if (shouldMisfire) {
                setPyroStatus(pyro, RTPyroStatus.PYRO_FAILURE);
                fired[pyro] = false;
            }
            else {
                setPyroStatus(pyro, RTPyroStatus.PYRO_SUCCESS);
                fired[pyro] = true;
            }
        }
    }

    public boolean firePyro(int pyro) {
        if (pyroStatuses[pyro] == RTPyroStatus.PYRO_CONNECTED) {
            if (getPyroFireStatus(pyro) == RTPyroStatus.PYRO_ARMED) {
                // assume works every time, fudgeable later.
                setPyroStatus(pyro, RTPyroStatus.PYRO_FIRING);
                if (SIMULATION_STATUS != null) {
                    fire_times[pyro] = SIMULATION_STATUS.getSimulationTime()*1e3; // seconds to miliseconds
                }
                else {
                    fire_times[pyro] = System.currentTimeMillis();
                }
                return true;
            }
        }
        return false;
    }

    private boolean isPyroConnected(int pyro) {
        // placeholder for onboard logic to check pyro connectivity
        return  (pyroStatuses[pyro] == RTPyroStatus.PYRO_CONNECTED) ||
                (pyroStatuses[pyro] == RTPyroStatus.PYRO_ARMED)     ||
                (pyroStatuses[pyro] == RTPyroStatus.PYRO_FIRING) ;
    }

    private void setPyroStatus(int pyro, RTPyroStatus status) {
        pyroStatuses[pyro] = status;
    }
    private RTPyroStatus getPyroFireStatus(int pyro) {
        return pyroStatuses[pyro];
    }

    private void firePyroAction(int pyro) {
        // placeholder for actual pyro firing action
        // digitalWrite(firePins[pyro],0);
    }


}
