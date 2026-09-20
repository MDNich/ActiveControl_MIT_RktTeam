package edu.mit.rocket_team.zephyrus;
import org.junit.jupiter.api.Test;
import edu.mit.rocket_team.zephyrus.FC.RTFC;
import static edu.mit.rocket_team.zephyrus.RTTestInputs.*;
import static edu.mit.rocket_team.zephyrus.util.RTRocketState.*;
import static org.junit.jupiter.api.Assertions.*;
class RTFCStateMachineTest {
    @Test void commandLatencyAndFlightEntryCalls() {
        RTFC c=fresh(); c.enqueueCommand(RTFC.stateCommand(PRE_FLIGHT)); tick(c,0,9.8065f,0,3); assertEquals(GROUND_TESTING,c.getState());
        tick(c,10,9.8065f,0,3); assertEquals(PRE_FLIGHT,c.getState());
        tick(c,20,39.7f,0,3); assertEquals(PRE_FLIGHT,c.getState());
        tick(c,30,39.9f,0,3); assertEquals(FLIGHT,c.getState());
        assertEquals(2,c.getAirbrakeUpdateCount()); assertEquals(2,c.getRollUpdateCount()); assertEquals(30,c.getFlightBeginTime());
    }
    @Test void strictApogeeAndDescentTimers() {
        RTFC c=flying(); c.enqueueCommand(RTFC.stateCommand(APOGEE)); tick(c,26020,9.8065f,0,3);
        tick(c,26030,9.8065f,0,3); assertEquals(FLIGHT,c.getState());
        tick(c,26040,9.8065f,0,3); assertEquals(APOGEE,c.getState()); assertTrue(c.pyroController.isFired(0)); assertTrue(c.pyroController.isFired(1));
        assertFalse(c.isAirbrakesEnabled()); c.latchPwm(); assertEquals(0,c.getOutput().exposedFraction());
        tick(c,29040,9.8065f,0,3); assertFalse(c.pyroController.isFired(3));
        tick(c,29050,9.8065f,0,3); assertTrue(c.pyroController.isFired(3)); assertTrue(c.pyroController.isFired(4));
        tick(c,31040,9.8065f,0,3); assertFalse(c.pyroController.isFired(5));
        tick(c,31050,9.8065f,0,3); assertTrue(c.pyroController.isFired(5));
        tick(c,81040,9.8065f,0,3); assertEquals(APOGEE,c.getState());
        tick(c,81050,9.8065f,0,3); assertEquals(MAIN,c.getState()); assertTrue(c.pyroController.isFired(2)); assertFalse(c.powerBoard.isEnabled(4));
        c.enqueueCommand(RTFC.stateCommand(END)); tick(c,81060,9.8065f,0,3); tick(c,81070,9.8065f,0,3); assertEquals(GROUND_TESTING,c.getState());
    }
    @Test void overrideAndGpsFixGate() {
        RTFC c=flying(); tick(c,26020,9.8065f,100,3); tick(c,26040,9.8065f,0,2); assertEquals(FLIGHT,c.getState());
        tick(c,26050,9.8065f,80,3); assertEquals(FLIGHT,c.getState());
        tick(c,26060,9.8065f,79.99f,3); assertEquals(APOGEE,c.getState());
        c=flying(); tick(c,35030,9.8065f,0,0); assertEquals(FLIGHT,c.getState()); tick(c,35040,9.8065f,0,0); assertEquals(APOGEE,c.getState());
    }
    @Test void malformedPacketsAndSourceEmergencyMask() {
        RTFC c=fresh(); c.enqueueCommand(RTFC.command(1,128)); tick(c,0,9.8065f,0,3); assertEquals(1,c.getBadPackets());
        byte[] bad=RTFC.command(2,1); bad[15]++; c.enqueueCommand(bad); tick(c,10,9.8065f,0,3); assertEquals(2,c.getBadPackets());
        c.enqueueCommand(RTFC.command(2,255)); tick(c,20,9.8065f,0,3); assertEquals(3,c.getBadPackets());
        c.enqueueCommand(RTFC.command(0x13,0)); tick(c,30,9.8065f,0,3); for(int i=0;i<5;i++) assertTrue(c.pyroController.isFired(i)); assertFalse(c.pyroController.isFired(5));
    }
    @Test void secondPreflightPreservesSourceOneShotFlagsAndDisabledRollSignal() {
        RTFC c=flying(); tick(c,35040,9.8065f,0,3); tick(c,40050,9.8065f,0,3);
        tick(c,90050,9.8065f,0,3); assertEquals(MAIN,c.getState());
        c.enqueueCommand(RTFC.stateCommand(END)); tick(c,90060,9.8065f,0,3); tick(c,90070,9.8065f,0,3);
        c.enqueueCommand(RTFC.stateCommand(PRE_FLIGHT)); tick(c,90080,9.8065f,0,3); tick(c,90090,9.8065f,0,3);
        assertEquals(PRE_FLIGHT,c.getState()); c.latchPwm(); assertEquals(0,c.getOutput().servo2Us());
        c.enqueueCommand(RTFC.stateCommand(FLIGHT)); tick(c,90100,9.8065f,0,3); tick(c,90110,9.8065f,0,3);
        tick(c,125120,9.8065f,0,3); assertEquals(APOGEE,c.getState());
        tick(c,131130,9.8065f,0,3);
        assertFalse(c.pyroController.isFired(3)); assertFalse(c.pyroController.isFired(4)); assertFalse(c.pyroController.isFired(5));
    }
    @Test void sourcePowerTimerRemainsUnadvanced() {
        var lines=new java.util.ArrayList<String>();
        RTFC c=new RTFC(new edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.Trace(lines::add)); c.init();
        tick(c,100,9.8065f,0,3); assertEquals(0,lines.stream().filter(s->s.contains("action=power.command")).count());
        tick(c,110,9.8065f,0,3); tick(c,120,9.8065f,0,3);
        assertEquals(2,lines.stream().filter(s->s.contains("action=power.command")).count());
    }
}
