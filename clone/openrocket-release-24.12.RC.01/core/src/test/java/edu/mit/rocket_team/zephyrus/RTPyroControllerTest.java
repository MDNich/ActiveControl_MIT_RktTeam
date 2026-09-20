package edu.mit.rocket_team.zephyrus;
import org.junit.jupiter.api.Test;
import edu.mit.rocket_team.zephyrus.control.RTPyroController;
import edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.Trace;
import static edu.mit.rocket_team.zephyrus.util.RTRocketState.*;
import static edu.mit.rocket_team.zephyrus.util.data.RTPyroStatus.*;
import static org.junit.jupiter.api.Assertions.*;
class RTPyroControllerTest {
    @Test void durationContinuityAndArmGateMatchFirmware() {
        Trace t=new Trace(s->{}); RTPyroController p=new RTPyroController(t); p.setup(); p.pyroMonitor(GROUND_TESTING);
        p.firePyro(0); assertFalse(p.isFired(0)); p.armPyro(0); p.firePyro(0); p.setConnected(0,false);
        t.time(250000); p.pyroMonitor(FLIGHT); assertTrue(p.isFired(0));
        t.time(251000); p.pyroMonitor(FLIGHT); assertFalse(p.isFired(0)); assertEquals(PYRO_SUCCESS,p.getPyroStatus(0));
        p.armPyro(1); p.firePyro(1); t.time(502000); p.pyroMonitor(FLIGHT); assertEquals(PYRO_FAILURE,p.getPyroStatus(1));
        assertThrows(IllegalArgumentException.class,()->p.armPyro(6));
    }
}
