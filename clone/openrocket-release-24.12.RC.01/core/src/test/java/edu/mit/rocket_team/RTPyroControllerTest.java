package edu.mit.rocket_team;

import org.junit.jupiter.api.Test;
import edu.mit.rocket_team.control.RTPyroController;
import edu.mit.rocket_team.util.RTUtilLibrary.Trace;
import static edu.mit.rocket_team.util.RTRocketState.*;
import static edu.mit.rocket_team.util.data.RTPyroStatus.*;
import static org.junit.jupiter.api.Assertions.*;

class RTPyroControllerTest {
    /**
     * Verifies that accepted commands retain microseconds and notify even during an active pulse.
     */
    @Test
    void acceptedCommandsRetainMicrosecondsAndNotifyEvenDuringAnActivePulse() {
        var trace = new Trace(s -> {});
        var pyro = new RTPyroController(trace);
        var commands = new java.util.ArrayList<RTPyroController.FireCommand>();
        pyro.setFireObserver(commands::add);
        trace.time(1_234_567);
        pyro.firePyro(2, "test");
        assertTrue(commands.isEmpty());
        pyro.armPyro(2);
        pyro.firePyro(2, "java_board:one");
        trace.time(1_234_890);
        pyro.armPyro(2);
        pyro.firePyro(2, "java_board:two");
        assertEquals(2, commands.size());
        assertEquals(1_234_567, commands.get(0).bootUs());
        assertEquals(1_234_890, commands.get(1).bootUs());
        assertEquals(2, commands.get(1).sequence());
        assertEquals("java_board:two", commands.get(1).origin());
        assertTrue(pyro.isFired(2));
        pyro.setFireObserver(null);
        pyro.armPyro(2);
        pyro.firePyro(2);
        assertEquals(2, commands.size());
    }

    /** Verifies that duration continuity and arm gate match firmware. */
    @Test
    void durationContinuityAndArmGateMatchFirmware() {
        Trace t = new Trace(s -> {});
        RTPyroController p = new RTPyroController(t);
        p.setup();
        p.pyroMonitor(GROUND_TESTING);
        p.firePyro(0);
        assertFalse(p.isFired(0));
        p.armPyro(0);
        p.firePyro(0);
        p.setConnected(0, false);
        t.time(250000);
        p.pyroMonitor(FLIGHT);
        assertTrue(p.isFired(0));
        t.time(251000);
        p.pyroMonitor(FLIGHT);
        assertFalse(p.isFired(0));
        assertEquals(PYRO_SUCCESS, p.getPyroStatus(0));
        p.armPyro(1);
        p.firePyro(1);
        t.time(502000);
        p.pyroMonitor(FLIGHT);
        assertEquals(PYRO_FAILURE, p.getPyroStatus(1));
        assertThrows(IllegalArgumentException.class, () -> p.armPyro(6));
    }
}
