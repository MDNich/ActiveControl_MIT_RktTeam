package info.openrocket.core.simulation.flightcomputer;

import edu.mit.rocket_team.zephyrus.FC.RTFC;
import edu.mit.rocket_team.zephyrus.control.RTRollController;
import edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.Trace;
import edu.mit.rocket_team.zephyrus.util.data.RTFudgedAirbrakesData;
import info.openrocket.core.util.BaseTestCase;
import jakarta.json.Json;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.nio.file.Path;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class FlightComputerPidTest extends BaseTestCase {
    @TempDir Path directory;
    private static final List<String> KEYS=List.of("airbrakeKp","airbrakeKi","airbrakeKd","rollKp","rollKi","rollKd");

    private FlightComputerDesign gain(FlightComputerDesign d,String key,double value){
        return d.with("parameters",Json.createObjectBuilder(d.parameters()).add(key,value).build());
    }

    @Test void legacyFilesUseOriginalGainsAndEditsValidatePersistAndChangeFingerprint() throws Exception {
        var baseline=FlightComputerDesign.zephyrus();var oldParameters=Json.createObjectBuilder(baseline.parameters());KEYS.forEach(oldParameters::remove);
        var legacy=baseline.with("parameters",oldParameters.build());legacy.requireRunnable();
        for(var key:KEYS)assertEquals(baseline.parameter(key),legacy.parameter(key));
        var changed=baseline;for(int i=0;i<KEYS.size();i++)changed=gain(changed,KEYS.get(i),.012345+i);
        changed.requireRunnable();assertNotEquals(baseline.fingerprint(),changed.fingerprint());
        var file=directory.resolve("pid.fc");changed.write(file);var loaded=FlightComputerDesign.read(file);
        for(var key:KEYS)assertEquals(changed.parameter(key),loaded.parameter(key));
        assertThrows(IllegalArgumentException.class,()->gain(baseline,"rollKi",1_000_001).requireRunnable());
    }

    @ParameterizedTest @CsvSource({"airbrakeKp,1","airbrakeKi,1000","airbrakeKd,0.1","rollKp,0","rollKi,0.001","rollKd,0"})
    void eachSavedGainChangesItsActualControllerOutput(String key,double value) throws Exception {
        var baseline=FlightComputerDesign.zephyrus();
        // Isolate each airbrake term: the source P term saturates this fixture at closed,
        // which would otherwise hide changes to the integral and derivative terms.
        if(key.startsWith("airbrake"))for(var air:KEYS.subList(0,3))baseline=gain(baseline,air,0);
        var changed=gain(baseline,key,value);var path=directory.resolve("tuned.fc");changed.write(path);
        var a=new RTFC(new Trace(line->{}));var b=new RTFC(new Trace(line->{}));a.configure(baseline);b.configure(FlightComputerDesign.read(path));
        a.rollController.setup();b.rollController.setup();boolean different=false;
        try(var reader=new BufferedReader(new InputStreamReader(getClass().getResourceAsStream("/zephyrus/airbrakes-integer.csv")))){
            reader.readLine();String line;while((line=reader.readLine())!=null){
                var f=line.split(",");long us=Long.parseLong(f[0]);float t=Float.parseFloat(f[1]),h=Float.parseFloat(f[2]),v=Float.parseFloat(f[3]);
                for(var fc:List.of(a,b)){
                    fc.trace.time(us);fc.airbrakesController.update(t,new RTFudgedAirbrakesData(h,v,Float.parseFloat(f[4]),f[5].equals("1")));
                    fc.rollController.update((us/1000-1000)/1000.0f,h,v,2,-.5f);
                }
                float first=key.startsWith("airbrake")?a.airbrakesController.getDeployment():a.rollController.getAngle();
                float second=key.startsWith("airbrake")?b.airbrakesController.getDeployment():b.rollController.getAngle();
                assertTrue(Float.isFinite(second));different |= Math.abs(first-second)>1e-6;
            }
        }
        assertTrue(different,key+" must affect the controller, not just the saved settings");
    }

    @Test void rollIntegralUsesElapsedVirtualTimeHoldsAtSaturationAndResets(){
        var clock=new Trace(line->{});var roll=new RTRollController(clock);roll.setPidGains(0,.001f,0);roll.setup();
        roll.update(0,100,300,1,0);assertEquals(0,roll.getIntegral());
        clock.time(10_000);roll.update(.01f,100,300,1,0);assertEquals(-.01,roll.getIntegral(),1e-7);
        clock.time(50_000);roll.update(.05f,100,300,1,0);assertEquals(-.05,roll.getIntegral(),1e-7);
        roll.update(.05f,100,300,1,0);assertEquals(-.05,roll.getIntegral(),1e-7);
        roll.resetPidHistory();assertEquals(0,roll.getIntegral());
        roll.setPidGains(0,1_000_000,0);roll.update(.05f,100,300,100,0);
        clock.time(60_000);roll.update(.06f,100,300,100,0);assertEquals(0,roll.getIntegral());
        assertThrows(IllegalArgumentException.class,()->roll.setPidGains(Float.NaN,0,0));
    }
    @Test void airbrakeDerivativeUsesMicrosecondClockEvenWhenFlightSecondsAreQuantized()throws Exception{
        var clock=new Trace(line->{});var controller=new edu.mit.rocket_team.zephyrus.control.airbrakes.RTAirbrakesController(clock);controller.setPidGains(0,0,0);
        long now=0;float time=0,height=0,velocity=0;
        try(var reader=new BufferedReader(new InputStreamReader(getClass().getResourceAsStream("/zephyrus/airbrakes-integer.csv")))){
            reader.readLine();String line;while((line=reader.readLine())!=null){
                var f=line.split(",");now=Long.parseLong(f[0]);time=Float.parseFloat(f[1]);height=Float.parseFloat(f[2]);velocity=Float.parseFloat(f[3]);clock.time(now);
                boolean plateau=controller.getState()==edu.mit.rocket_team.zephyrus.control.airbrakes.RTAirbrakesControllerState.CONTROLLING_PLATEAU;
                controller.update(time,new RTFudgedAirbrakesData(height,velocity,Float.parseFloat(f[4]),false));if(plateau)break;
            }
        }
        assertEquals(0,controller.getErrorDerivative());
        clock.time(now+10000);controller.update(time,new RTFudgedAirbrakesData(height+1,velocity,-10,false));assertEquals(100,controller.getErrorDerivative(),.1);
        clock.time(now+30000);controller.update(time,new RTFudgedAirbrakesData(height+2,velocity,-10,false));assertEquals(50,controller.getErrorDerivative(),.1);
        controller.update(time,new RTFudgedAirbrakesData(height+2,velocity,-10,false));assertEquals(0,controller.getErrorDerivative());
        controller.resetPidHistory();assertEquals(0,controller.getErrorDerivative());assertEquals(0,controller.getIntegral());
    }
}
