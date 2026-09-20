package edu.mit.rocket_team.zephyrus;
import org.junit.jupiter.api.Test;
import edu.mit.rocket_team.zephyrus.FC.RTFC;
import static edu.mit.rocket_team.zephyrus.RTTestInputs.*;
import static org.junit.jupiter.api.Assertions.*;
class RTFCOutputTest {
    @Test void manualOutputIsHeldUntilPwmAndClosesAtPreflight() {
        RTFC c=fresh(); assertEquals(941,c.getOutput().airbrakePulseUs());
        c.enqueueCommand(RTFC.angleCommand(3,-117)); tick(c,0,9.8065f,0,3); assertEquals(0,c.getOutput().exposedFraction());
        c.latchPwm(); assertEquals(525,c.getOutput().airbrakePulseUs()); assertEquals(1,c.getOutput().exposedFraction());
        c.enqueueCommand(RTFC.stateCommand(edu.mit.rocket_team.zephyrus.util.RTRocketState.PRE_FLIGHT)); tick(c,10,9.8065f,0,3); tick(c,20,9.8065f,0,3);
        c.latchPwm(); assertEquals(0,c.getOutput().exposedFraction());
    }
    @Test void runsAreIndependentAndDuplicateTicksRejected() {
        RTFC a=flying(),b=fresh(); tick(b,0,9.8065f,0,3);
        assertNotEquals(a.getState(),b.getState()); assertEquals(2,a.getAirbrakeUpdateCount()); assertEquals(0,b.getAirbrakeUpdateCount());
        assertThrows(IllegalArgumentException.class,()->tick(b,0,9.8065f,0,3)); assertThrows(IllegalStateException.class,b::init);
    }
    @Test void nonfiniteManualCommandIsRejectedWithoutChangingOutput() {
        RTFC c=fresh(); c.enqueueCommand(RTFC.angleCommand(3,Float.NaN)); tick(c,0,9.8065f,0,3); c.latchPwm();
        assertEquals(1,c.getBadPackets()); assertEquals(941,c.getOutput().airbrakePulseUs());
        RTFC automatic=flying(); automatic.airbrakesController.setAirbrakesServo(Float.NaN);
        assertThrows(IllegalStateException.class,automatic::latchPwm);
    }
}
