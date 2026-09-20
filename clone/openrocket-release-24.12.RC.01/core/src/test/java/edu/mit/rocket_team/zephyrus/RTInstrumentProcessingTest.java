package edu.mit.rocket_team.zephyrus;
import org.junit.jupiter.api.Test;
import edu.mit.rocket_team.zephyrus.instrument.*;
import edu.mit.rocket_team.zephyrus.util.*;
import edu.mit.rocket_team.zephyrus.util.data.*;
import static edu.mit.rocket_team.zephyrus.util.RTRocketState.*;
import static org.junit.jupiter.api.Assertions.*;
class RTInstrumentProcessingTest {
    @Test void accelerationThresholdFreshnessAndClockWrap() {
        RTUtilLibrary.Trace time=new RTUtilLibrary.Trace(s->{}); RTAccel a=new RTAccel(time);
        time.time(10000); a.backdoorFudge(new RTAccelData(19f,0,0)); a.update(PRE_FLIGHT); assertEquals(0,a.getIntegratedVelo());
        time.time(20000); a.backdoorFudge(new RTAccelData(29.8065f,0,0)); a.update(PRE_FLIGHT); assertEquals(0.2,a.getIntegratedVelo(),1e-5);
        a.update(FLIGHT); assertEquals(0.2,a.getIntegratedVelo(),1e-5);
        time.time(30000); a.update(FLIGHT); assertEquals(0.2,a.getIntegratedVelo(),1e-5);
        a.zeroIntegratedVelo(); assertEquals(0,a.getIntegratedVelo());
        assertEquals(32,RTUtilLibrary.elapsed32(16,0xfffffff0L));
        time=new RTUtilLibrary.Trace(s->{}); a=new RTAccel(time);
        time.time(0xfffffff0L); a.backdoorFudge(new RTAccelData(9.8065f,0,0)); a.update(FLIGHT); a.zeroIntegratedVelo();
        time.time(0x1_00000010L); a.backdoorFudge(new RTAccelData(29.8065f,0,0)); a.update(FLIGHT); assertEquals(0.00064,a.getIntegratedVelo(),1e-6);
    }
    @Test void barometerWindowAndSourceRepeatedZero() {
        RTBaro b=new RTBaro(new RTUtilLibrary.Trace(s->{})); b.backdoorFudge(new RTBaroData(Float.NaN,Float.NaN,20,Float.NaN,900));
        float h=b.getAltitude(); b.updateAll(); assertEquals(h/20,b.getFilteredAltitude(),1e-4);
        for(int i=1;i<20;i++) b.updateAll(); assertEquals(h,b.getFilteredAltitude(),0.001);
        b.zeroAlt(); assertEquals(0,b.getFilteredAltitude(),0.001);
        b.zeroAlt(); assertEquals(h,b.getFilteredAltitude(),0.001); // Preserve source offset assignment quirk.
        assertEquals(0,b.getMaxAlt());
    }
    @Test void gpsHeightAndFixAndGyroSigns() {
        RTUtilLibrary.Trace clock=new RTUtilLibrary.Trace(s->{}); RTGPS gps=new RTGPS(clock);
        RTGPSData d=new RTGPSData(42f,-77f,100,0,0,0,true); gps.backdoorFudge(d); gps.updateAndParse(); gps.zeroAlt();
        d=new RTGPSData(42f,-77f,125.125f,0,0,0,true); gps.backdoorFudge(d); gps.updateAndParse(); assertEquals(25.125,gps.getHeight(),1e-5);
        d.setAltitude(150); d.setFixType(2); gps.backdoorFudge(d); gps.updateAndParse(); assertEquals(25.125,gps.getMaxAlt(),1e-5);
        RTGyro gyro=new RTGyro(clock); gyro.backdoorFudge(new RTGyroData(10,20,-30)); clock.time(1000000); gyro.update();
        assertEquals(-10,gyro.getRoll()); assertEquals(20,gyro.getPitch()); assertEquals(-30,gyro.getYaw());
        gyro.zeroRollPitchYaw(); gyro.update(); assertEquals(0,gyro.getRoll());
    }
}
