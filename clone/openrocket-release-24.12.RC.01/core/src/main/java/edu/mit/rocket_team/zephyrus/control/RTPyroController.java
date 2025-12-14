package edu.mit.rocket_team.zephyrus.control;

import edu.mit.rocket_team.zephyrus.util.RTController;
import edu.mit.rocket_team.zephyrus.util.data.RTPyroFireStatus;
import edu.mit.rocket_team.zephyrus.util.data.RTPyroConnectStatus;

public class RTPyroController extends RTController {

    public static final int NUM_PYROS = 6;

    RTPyroConnectStatus[] pyroConnectStatuses;
    RTPyroFireStatus[] pyroFireStatuses;

    public RTPyroController() {
        pyroConnectStatuses = new RTPyroConnectStatus[NUM_PYROS];
        pyroFireStatuses = new RTPyroFireStatus[NUM_PYROS];
    }

    @Override
    public void setup() {
        for (int i = 0; i < NUM_PYROS; i++) {
            pyroConnectStatuses[i] = RTPyroConnectStatus.PYRO_UNCONNECTED; // assume connectivity, fudgable later
            setPyroStatus(i,RTPyroFireStatus.PYRO_READY); // assume ready, fudgable later
        }
    }

    public int getPyroStatus(int pyro) {
        pyroMonitor(pyro);
        return pyroConnectStatuses[pyro].ID;
    }
    public void armPyro(int pyro) {
        if (isPyroConnected(pyro)) {
            if (getPyroFireStatus(pyro) == RTPyroFireStatus.PYRO_READY) {
                setPyroStatus(pyro, RTPyroFireStatus.PYRO_ARMED);
            }
        }
    }
    public void disarmPyro(int pyro) {
        if (isPyroConnected(pyro)) {
            if (getPyroFireStatus(pyro) == RTPyroFireStatus.PYRO_ARMED) {
                setPyroStatus(pyro, RTPyroFireStatus.PYRO_READY);
            }
        }
    }
    public void firePyro(int pyro) {
        if (pyroConnectStatuses[pyro] == RTPyroConnectStatus.PYRO_CONNECTED) {
            if (getPyroFireStatus(pyro) == RTPyroFireStatus.PYRO_ARMED) {
                setPyroStatus(pyro, RTPyroFireStatus.PYRO_FIRED);
                pyroConnectStatuses[pyro] = RTPyroConnectStatus.PYRO_SUCCESS; // assume success, fudgable later
            }
        }
    }
    public void pyroMonitor(int pyro) {
        // no-op, placeholder for onboard logic.
        // possibly: fudge sensors here as needed.
    }

    private boolean isPyroConnected(int pyro) {
        // placeholder for onboard logic to check pyro connectivity
        return pyroConnectStatuses[pyro] == RTPyroConnectStatus.PYRO_CONNECTED;
    }
    private void setPyroStatus(int pyro, RTPyroFireStatus status) {
        pyroFireStatuses[pyro] = status;
    }
    private RTPyroFireStatus getPyroFireStatus(int pyro) {
        return pyroFireStatuses[pyro];
    }



}
