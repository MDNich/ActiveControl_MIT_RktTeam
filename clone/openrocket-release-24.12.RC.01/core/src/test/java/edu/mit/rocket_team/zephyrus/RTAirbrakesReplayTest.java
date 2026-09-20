package edu.mit.rocket_team.zephyrus;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import java.io.*;
import edu.mit.rocket_team.zephyrus.control.airbrakes.*;
import edu.mit.rocket_team.zephyrus.control.RTRollController;
import edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.Trace;
import edu.mit.rocket_team.zephyrus.util.data.RTFudgedAirbrakesData;
import static org.junit.jupiter.api.Assertions.*;
class RTAirbrakesReplayTest {
    @ParameterizedTest @ValueSource(strings={"integer","fractional","partial","late"})
    void matchesUnmodifiedCpp(String scenario) throws Exception {
        Trace clock=new Trace(s->{}); RTAirbrakesController c=new RTAirbrakesController(clock); RTRollController r=new RTRollController(clock); r.setup();
        try(BufferedReader reader=new BufferedReader(new InputStreamReader(getClass().getResourceAsStream("/zephyrus/airbrakes-"+scenario+".csv")))) {
            reader.readLine(); String line;
            while((line=reader.readLine())!=null) {
                String[] f=line.split(","); long us=Long.parseLong(f[0]); clock.time(us); float t=Float.parseFloat(f[1]),h=Float.parseFloat(f[2]),v=Float.parseFloat(f[3]);
                c.update(t,new RTFudgedAirbrakesData(h,v,Float.parseFloat(f[4]),f[5].equals("1")));
                assertEquals(Integer.parseInt(f[6]),c.getState().ordinal(),scenario+" at "+us);
                assertEquals(Integer.parseInt(f[7]),c.getSampleCount(),"sample count at "+us);
                check(Float.parseFloat(f[8]),c.getDeployment(),"deployment "+us);
                check(Float.parseFloat(f[9]),c.getPredictedAltitude(),"prediction "+us);
                check(Float.parseFloat(f[10]),c.getDesiredAltitude(),"target "+us);
                check(Float.parseFloat(f[11]),c.getIntegral(),"integral "+us);
                r.update((us/1000-1000)/1000.0f,h,v,2,-0.5f);
                check(Float.parseFloat(f[12]),r.getAngle(),"roll "+us);
            }
        }
    }
    private void check(float expected,float actual,String where) {
        assertTrue(Float.isFinite(expected),"Reference nonfinite: "+where);
        assertEquals(expected,actual,1e-5+1e-5*Math.abs(expected),where);
    }
}
