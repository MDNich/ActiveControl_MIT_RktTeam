package info.openrocket.core.simulation.flightcomputer;

import edu.mit.rocket_team.zephyrus.FC.RTFC;
import edu.mit.rocket_team.zephyrus.RTFCVerificationRocket;
import edu.mit.rocket_team.zephyrus.telemetry.TelemetryLinkSettings;
import edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.Trace;
import info.openrocket.core.document.*;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener;
import info.openrocket.core.util.BaseTestCase;
import jakarta.json.*;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import java.nio.file.*;
import java.util.*;
import static org.junit.jupiter.api.Assertions.*;

class FlightComputerDesignerFeaturesTest extends BaseTestCase {
    @TempDir Path temp;
    String library,telemetry;
    @BeforeEach void setup(){library=System.getProperty("openrocket.fc.library");telemetry=System.getProperty("openrocket.fc.telemetryDir");System.setProperty("openrocket.fc.library",temp.resolve("library").toString());System.setProperty("openrocket.fc.telemetryDir",temp.resolve("telemetry").toString());}
    @AfterEach void cleanup(){FlightComputerDesignTest.set("openrocket.fc.library",library);FlightComputerDesignTest.set("openrocket.fc.telemetryDir",telemetry);}
    @Test void recoveryIntermediateStateRunsOnceOnItsVirtualTimeBoundary()throws Exception{
        var base=FlightComputerDesign.zephyrus();var actions=Json.createArrayBuilder().add(Json.createObjectBuilder().add("type","fire_recovery").add("channel",3)).build();
        var graph=FlightComputerStateMachine.insert(base.stateMachine(),"apogee","drogue_stable","Drogue stable",FlightComputerStateMachine.condition("state_ms",">=",50),actions);
        graph=Json.createObjectBuilder(graph).add("initial","apogee").add("automaticRecovery",false).build();var design=base.with("stateMachine",graph);design.requireRunnable();
        var file=temp.resolve("states.fc");design.write(file);design=FlightComputerDesign.read(file);var logs=new ArrayList<String>();var fc=new RTFC(new Trace(logs::add));fc.configure(design);fc.init();
        for(long t=0;t<50000;t+=10000){fc.pre_loop(t,FlightComputerDesignTest.sample(t,9.8065f));fc.loop();}assertEquals("apogee",fc.getDesignStateId());assertFalse(fc.pyroController.isFired(3));
        for(long t=50000;t<=100000;t+=10000){fc.pre_loop(t,FlightComputerDesignTest.sample(t,9.8065f));fc.loop();}
        assertEquals("drogue_stable",fc.getDesignStateId());assertTrue(fc.pyroController.isFired(3));assertFalse(fc.pyroController.isFired(4));
        assertEquals(1,logs.stream().filter(l->l.contains("action=pyro.fire channel=3 ")).count());assertTrue(logs.stream().anyMatch(l->l.contains("boot_us=50000 action=state.enter id=drogue_stable")));
    }
    @Test void materializedDefaultGraphPreservesControllerAndRecoveryTiming(){
        var d=FlightComputerDesign.zephyrus();var original=new RTFC(new Trace(line->{}));var graph=new RTFC(new Trace(line->{}));original.configure(d);graph.configure(d.with("stateMachine",d.stateMachine()));original.init();graph.init();
        original.enqueueCommand(RTFC.stateCommand(edu.mit.rocket_team.zephyrus.util.RTRocketState.PRE_FLIGHT));graph.enqueueCommand(RTFC.stateCommand(edu.mit.rocket_team.zephyrus.util.RTRocketState.PRE_FLIGHT));
        for(long t=0;t<=41000000;t+=10000){var input=FlightComputerDesignTest.sample(t,t<20000?9.8065f:50);original.pre_loop(t,input);original.loop();graph.pre_loop(t,input);graph.loop();
            assertEquals(original.getState(),graph.getState(),"state at "+t);assertEquals(original.getApogeeTime(),graph.getApogeeTime());
            for(int channel=0;channel<6;channel++)assertEquals(original.pyroController.isFired(channel),graph.pyroController.isFired(channel),"channel "+channel+" at "+t);
            if(t%20000==0){original.latchPwm();graph.latchPwm();assertEquals(original.getOutput(),graph.getOutput());}
        }
    }
    private FlightComputerDesign board(String source,long period,long cost){var d=FlightComputerDesign.zephyrus();var node=Json.createObjectBuilder(FlightComputerModels.newNode("java_board","",0,0)).add("program",source).add("properties",Json.createObjectBuilder().add("periodUs",period).add("executionUs",cost)).build();return d.with("nodes",Json.createArrayBuilder(d.nodes()).add(node).build());}
    @Test void javaBoardCompilesWithoutExecutingAndRequiresLocalSourceApproval(){
        String key="openrocket.test.java-board-init";
        String code="import info.openrocket.core.simulation.flightcomputer.JavaBoardProgram; public class BoardProgram extends JavaBoardProgram { static { System.setProperty(\""+key+"\",\"yes\"); } public void step(Context io) {} }";
        try{JavaBoardProgram.revoke(code);JavaBoardProgram.compile(code);assertNull(System.getProperty(key));var d=board(code,10000,0);assertThrows(IllegalArgumentException.class,()->new RTFC(new Trace(s->{})).configure(d));assertNull(System.getProperty(key));
            JavaBoardProgram.approve(code);new RTFC(new Trace(s->{})).configure(d);assertEquals("yes",System.getProperty(key));assertFalse(JavaBoardProgram.approved(code+" "));
            assertThrows(IllegalArgumentException.class,()->JavaBoardProgram.compile("public class BoardProgram { broken syntax }"));
        }finally{JavaBoardProgram.revoke(code);System.clearProperty(key);}
    }
    @Test void javaCallbacksHaveIndependentVirtualPeriodsAndResetBetweenRuns(){
        String code="import info.openrocket.core.simulation.flightcomputer.JavaBoardProgram; public class BoardProgram extends JavaBoardProgram { private static int calls; public void step(Context io) { calls++; io.log(\"calls=\"+calls+\" t=\"+io.timeUs()); io.setAirbrakes(0.4); } }";
        try{JavaBoardProgram.approve(code);var d=board(code,20000,6000);
            for(int run=0;run<2;run++){var lines=new ArrayList<String>();var result=FlightControllerSimulatorListener.bench(d,45000,t->FlightComputerDesignTest.sample(t,9.8065f),lines::add);
                assertEquals(3000,result.timing().maxExecutionUs());assertEquals(0,result.timing().overruns());assertEquals(2,lines.stream().filter(l->l.contains("calls=")).count());
                assertTrue(lines.stream().anyMatch(l->l.contains("boot_us=6000")&&l.contains("calls=1 t=0")));assertTrue(lines.stream().anyMatch(l->l.contains("boot_us=26000")&&l.contains("calls=2 t=20000")));assertTrue(result.points().stream().anyMatch(p->p.output()>.39));}
            var slowLines=new ArrayList<String>();var slow=FlightControllerSimulatorListener.bench(board(code,10000,12000),60000,t->FlightComputerDesignTest.sample(t,9.8065f),slowLines::add);assertEquals(3000,slow.timing().maxExecutionUs());assertEquals(0,slow.timing().overruns());assertTrue(slowLines.stream().anyMatch(l->l.contains("event=step")&&l.contains("overrun_us=2000")));
        }finally{JavaBoardProgram.revoke(code);}
    }
    @Test void slowAndFastBoardsRunTogetherWithoutSerializingTheirWork()throws Exception{
        String a="import info.openrocket.core.simulation.flightcomputer.JavaBoardProgram; public class BoardProgram extends JavaBoardProgram { public void step(Context io) { io.log(\"FAST start=\"+io.timeUs()+\" dt=\"+io.dtUs()); io.setAirbrakes(0.2); } }";
        String b="import info.openrocket.core.simulation.flightcomputer.JavaBoardProgram; public class BoardProgram extends JavaBoardProgram { public void step(Context io) { io.log(\"SLOW start=\"+io.timeUs()+\" dt=\"+io.dtUs()); io.setRollDegrees(3); } }";
        JavaBoardProgram.approve(a);JavaBoardProgram.approve(b);
        try{
            var design=board(a,7000,1000);var slow=Json.createObjectBuilder(FlightComputerModels.newNode("java_board","",10,10)).add("program",b).add("properties",Json.createObjectBuilder().add("periodUs",11000).add("executionUs",25000)).build();
            design=design.with("nodes",Json.createArrayBuilder(design.nodes()).add(slow).build());var lines=new ArrayList<String>();
            var result=FlightControllerSimulatorListener.bench(design,52000,t->FlightComputerDesignTest.sample(t,9.8065f),lines::add);
            assertEquals(3000,result.timing().maxExecutionUs());assertEquals(0,result.timing().overruns());
            for(long start=0;start<=49000;start+=7000){final long expected=start;assertTrue(lines.stream().anyMatch(l->l.contains("boot_us="+(expected+1000)+" ")&&l.contains("FAST start="+expected+" dt="+(expected==0?0:7000))));}
            assertTrue(lines.stream().anyMatch(l->l.contains("boot_us=25000 ")&&l.contains("SLOW start=0 dt=0")));
            assertTrue(lines.stream().anyMatch(l->l.contains("boot_us=50000 ")&&l.contains("SLOW start=25000 dt=25000")));
            assertTrue(lines.stream().anyMatch(l->l.contains("overrun_us=14000")));
            assertTrue(lines.stream().anyMatch(l->l.contains("action=output.command port=roll value=3.0")));
            var file=temp.resolve("parallel.fc");design.write(file);var simulation=RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());simulation.getOptions().setMaxSimulationTime(.05);
            ZephyrusFlightComputer.apply(simulation,true,TelemetryLinkSettings.DEFAULT);ZephyrusFlightComputer.selectFile(simulation,file);simulation.simulate();
            assertTrue(simulation.hasSimulationData());assertTrue(Files.readString(simulation.getFlightComputerLogPath()).contains("FAST start="));
        }finally{JavaBoardProgram.revoke(a);JavaBoardProgram.revoke(b);}
    }
    @Test void boardsReadSnapshotsAtStartAndPublishOnlyWhenTheirOwnWorkFinishes(){
        String code="import info.openrocket.core.simulation.flightcomputer.JavaBoardProgram; public class BoardProgram extends JavaBoardProgram { public void step(Context io) { io.setAirbrakes(io.altitudeM()/100); } } // shared input snapshot test";
        JavaBoardProgram.approve(code);
        try{
            var d=FlightComputerDesign.zephyrus();var nodes=Json.createArrayBuilder(d.nodes());
            for(String id:List.of("a","b"))nodes.add(Json.createObjectBuilder(FlightComputerModels.newNode("java_board","",0,0)).add("id",id).add("program",code).add("properties",Json.createObjectBuilder().add("periodUs",10000).add("executionUs",id.equals("a")?20000:5000)));
            var runtime=new JavaBoardRuntime(d.with("nodes",nodes.build()));var outputs=new ArrayList<String>();var clock=new java.util.concurrent.atomic.AtomicLong();
            while(runtime.nextDeadlineUs()<=20000){long now=runtime.nextDeadlineUs();clock.set(now);runtime.dispatch(now,now,"flight",key->now==0?42:84,port->true,(id,port,value,sample)->outputs.add(clock.get()+":"+id+":"+value+":"+sample),line->{});}
            assertEquals(List.of("5000:b:0.42:0","15000:b:0.84:10000","20000:a:0.42:0"),outputs);
        }finally{JavaBoardProgram.revoke(code);}
    }
    @Test void absentAirbrakeOutputIsDiscardedAndAirbrakelessRocketRuns()throws Exception{
        var d=FlightComputerDesign.zephyrus();var nodes=Json.createArrayBuilder();for(var n:d.nodes().getValuesAs(JsonObject.class))if(!n.getString("type").equals("airbrakes"))nodes.add(n);
        var links=Json.createArrayBuilder();for(var c:d.connections().getValuesAs(JsonObject.class))if(!c.getString("port").equals("airbrakes"))links.add(c);d=d.with("nodes",nodes.build()).with("connections",links.build());d.requireRunnable();
        var logs=new ArrayList<String>();var fc=new RTFC(new Trace(logs::add));fc.configure(d);fc.init();fc.enqueueCommand(RTFC.angleCommand(3,RTFC.OPEN_ANGLE));fc.pre_loop(0,FlightComputerDesignTest.sample(0,9.8065f));fc.loop();fc.latchPwm();assertEquals(0,fc.getOutput().exposedFraction());assertEquals(0,fc.getOutput().airbrakePulseUs());assertTrue(logs.stream().anyMatch(l->l.contains("action=output.discard port=airbrakes")));
        var rocket=RTFCVerificationRocket.rocket();for(var n:new ArrayList<>(rocket.getSelectedConfiguration().getActiveComponents()))if(n instanceof info.openrocket.core.rocketcomponent.AirbrakeSet)n.getParent().removeChild(n);
        var simulation=RTFCVerificationRocket.simulation(rocket);simulation.getOptions().setMaxSimulationTime(.04);Path file=temp.resolve("no-airbrakes.fc");d.write(file);ZephyrusFlightComputer.apply(simulation,true,TelemetryLinkSettings.DEFAULT);ZephyrusFlightComputer.selectFile(simulation,file);simulation.simulate();assertTrue(simulation.hasSimulationData());
    }
    @Test void gaussianPercentNoiseHasConfiguredScaleAndRepeatableSeed(){
        var base=FlightComputerDesign.zephyrus();var d=base.with("parameters",Json.createObjectBuilder(base.parameters()).add("measurementNoisePercent",10).build());var a=new FlightComputerSensors(d);var b=new FlightComputerSensors(d);double sum=0,squares=0;
        for(int i=0;i<4000;i++){var input=FlightComputerDesignTest.sample(i*10000L,100);double x=a.sample(i*10000L,input).accel().getAccelX();assertEquals(x,b.sample(i*10000L,input).accel().getAccelX());sum+=x;squares+=(x-100)*(x-100);}
        assertEquals(100,sum/4000,.6);assertEquals(10,Math.sqrt(squares/4000),.6);
    }
    @Test void multipleProcessorsCanShareSensorsAndDistributePortsWithoutAmbiguousSources(){
        var base=FlightComputerDesign.zephyrus();var processor=Json.createObjectBuilder(FlightComputerModels.newNode("processor","main",20,20)).add("id","aux_cpu").build();
        var design=base.with("nodes",Json.createArrayBuilder(base.nodes()).add(processor).build());
        var links=Json.createArrayBuilder();for(var c:base.connections().getValuesAs(JsonObject.class))links.add(c.getString("port").equals("gyroscope")?Json.createObjectBuilder(c).add("to","aux_cpu").build():c);
        var barometer=base.connections().getValuesAs(JsonObject.class).stream().filter(c->c.getString("port").equals("barometer")).findFirst().orElseThrow();
        links.add(Json.createObjectBuilder(barometer).add("id","shared_baro").add("to","aux_cpu"));design=design.with("connections",links.build());design.requireRunnable();
        var result=FlightControllerSimulatorListener.bench(design,30000,t->FlightComputerDesignTest.sample(t,9.8065f),s->{});assertTrue(result.timing().completedLoops()>0);
        var other=Json.createObjectBuilder(base.node("baro")).add("id","baro2").build();var ambiguous=design.with("nodes",Json.createArrayBuilder(design.nodes()).add(other).build());
        var badLinks=Json.createArrayBuilder();for(var c:design.connections().getValuesAs(JsonObject.class))badLinks.add(c.getString("id").equals("shared_baro")?Json.createObjectBuilder(c).add("from","baro2").build():c);
        var invalid=ambiguous.with("connections",badLinks.build());assertTrue(assertThrows(IllegalArgumentException.class,invalid::requireRunnable).getMessage().contains("more than one source"));
    }
    @Test void javaBoardCanHostTheDesignWithoutASeparateProcessorSymbol(){
        var base=FlightComputerDesign.zephyrus();String source=JavaBoardProgram.template()+"\n// host test "+UUID.randomUUID();
        var host=Json.createObjectBuilder(FlightComputerModels.newNode("java_board","main",20,20)).add("id","cpu").add("program",source).build();
        var nodes=Json.createArrayBuilder();for(var n:base.nodes().getValuesAs(JsonObject.class))nodes.add(n.getString("id").equals("cpu")?host:n);
        var design=base.with("nodes",nodes.build());design.requireRunnable();JavaBoardProgram.approve(source);
        try{var logs=new ArrayList<String>();var result=FlightControllerSimulatorListener.bench(design,30000,t->FlightComputerDesignTest.sample(t,9.8065f),logs::add);
            assertTrue(result.timing().completedLoops()>0);assertTrue(logs.stream().anyMatch(s->s.contains("board=cpu")&&s.contains("event=step")));
        }finally{JavaBoardProgram.revoke(source);}
    }
    @Test void fullFlightPlotSamplesImportAsExternalResultWithUnitsAndPhysicalComparisons()throws Exception{
        var sim=RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());sim.getOptions().setMaxSimulationTime(.12);var design=FlightComputerDesign.zephyrus();Path file=temp.resolve("plot.fc");design.write(file);
        ZephyrusFlightComputer.apply(sim,true,TelemetryLinkSettings.DEFAULT);ZephyrusFlightComputer.selectFile(sim,file);sim.simulate();var branch=sim.getSimulatedData().getBranch(0);
        assertTrue(branch.get(FlightComputerData.STATE).stream().anyMatch(Double::isFinite));assertTrue(branch.get(FlightComputerData.ACCEL[0].truth()).stream().anyMatch(Double::isFinite));assertTrue(branch.get(FlightComputerData.PWM[0]).stream().allMatch(v->!Double.isFinite(v)||(v>0&&v<.003)));
        var result=FlightComputerLogReader.read(sim.getFlightComputerLogPath()).get(0);assertTrue(result.data().getBranch(0).getLength()>5);
        assertEquals(info.openrocket.core.unit.UnitGroup.UNITS_PRESSURE,FlightComputerData.PRESSURE.measured().getUnitGroup());
        for(double v:result.data().getBranch(0).get(FlightComputerData.PRESSURE.measured()))assertTrue(v>1000);
        var doc=OpenRocketDocumentFactory.createDocumentFromRocket(sim.getRocket());var imported=new Simulation(doc,sim.getRocket(),Simulation.Status.EXTERNAL,"Imported",sim.getOptions().clone(),List.of(),result.data());assertEquals(Simulation.Status.EXTERNAL,imported.getStatus());
        // Log analysis must also work in a new document with no motors or FC file.
        var empty=OpenRocketDocumentFactory.createNewRocket();var detached=new Simulation(empty,empty.getRocket(),Simulation.Status.EXTERNAL,"Log only",new SimulationOptions(),List.of(),result.data());empty.addSimulation(detached);
        detached.getOptions().setLaunchAltitude(999);assertEquals(Simulation.Status.EXTERNAL,detached.getStatus());
        for(boolean saveSimulationData:new boolean[]{true,false}){
        var storage=new StorageOptions();storage.setSaveSimulationData(saveSimulationData);var saved=new java.io.ByteArrayOutputStream();
        new info.openrocket.core.file.openrocket.OpenRocketSaver().save(saved,empty,storage,new info.openrocket.core.logging.WarningSet(),new info.openrocket.core.logging.ErrorSet());
        var context=new info.openrocket.core.file.DocumentLoadingContext();context.setOpenRocketDocument(OpenRocketDocumentFactory.createEmptyRocket());context.setAttachmentFactory(new info.openrocket.core.file.FileSystemAttachmentFactory(temp.toFile()));
        new info.openrocket.core.file.openrocket.importt.OpenRocketLoader().loadFromStream(context,new java.io.ByteArrayInputStream(saved.toByteArray()),"imported.ork");
        var reopened=context.getOpenRocketDocument().getSimulation(0);assertEquals(Simulation.Status.EXTERNAL,reopened.getStatus());assertEquals(result.data().getFlightComputerProvenance(),reopened.getSimulatedData().getFlightComputerProvenance());
        assertEquals(result.data().getBranch(0).get(FlightComputerData.PRESSURE.measured()),reopened.getSimulatedData().getBranch(0).get(FlightComputerData.PRESSURE.measured()));
        }
    }
    @Test void oldLogsRecoverOnlyPresentQuantitiesAndMalformedLogsAreRejected()throws Exception{
        Path log=temp.resolve("old.log");Files.writeString(log,"ZEPHYRUS run=1 boot_us=1000000 action=baro.update pressure_hPa=1000 temperature_C=20 filtered_m=10\\n".replace("\\n","\n")+"ZEPHYRUS run=1 boot_us=1000000 action=physics.accept simulation_s=0 truth_altitude_m=150 truth_velocity_mps=0\nZEPHYRUS run=1 boot_us=1010000 action=physics.accept simulation_s=0.01 truth_altitude_m=152 truth_velocity_mps=20\n");
        var result=FlightComputerLogReader.read(log).get(0);var branch=result.data().getBranch(0);assertEquals(2,branch.getLast(FlightDataType.TYPE_ALTITUDE));assertEquals(100000,branch.getLast(FlightComputerData.PRESSURE.measured()));assertNull(branch.get(FlightDataType.TYPE_POSITION_X));assertTrue(result.note().startsWith("Older log"));
        Files.writeString(log,"ZEPHYRUS run=1 boot_us=1 action=fc.plot {broken json}\n");assertThrows(java.io.IOException.class,()->FlightComputerLogReader.read(log));
    }
}
