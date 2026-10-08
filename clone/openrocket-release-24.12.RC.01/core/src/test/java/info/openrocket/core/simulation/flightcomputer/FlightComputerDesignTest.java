package info.openrocket.core.simulation.flightcomputer;
import edu.mit.rocket_team.zephyrus.RTFCVerificationRocket;
import edu.mit.rocket_team.zephyrus.FC.RTFC;
import edu.mit.rocket_team.zephyrus.telemetry.*;
import edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.Trace;
import edu.mit.rocket_team.zephyrus.util.RTRocketState;
import edu.mit.rocket_team.zephyrus.util.data.*;
import info.openrocket.core.util.BaseTestCase;
import info.openrocket.core.document.*;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.simulation.listeners.*;
import info.openrocket.core.simulation.ensemble.EnsembleSettings;
import jakarta.json.*;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import java.nio.file.*;
import java.util.*;
import static org.junit.jupiter.api.Assertions.*;

class FlightComputerDesignTest extends BaseTestCase {
    @TempDir Path temp;String oldLibrary,oldTelemetry;
    @BeforeEach void setup(){oldLibrary=System.getProperty("openrocket.fc.library");oldTelemetry=System.getProperty("openrocket.fc.telemetryDir");System.setProperty("openrocket.fc.library",temp.resolve("library").toString());System.setProperty("openrocket.fc.telemetryDir",temp.resolve("telemetry").toString());}
    @AfterEach void restore(){set("openrocket.fc.library",oldLibrary);set("openrocket.fc.telemetryDir",oldTelemetry);}
    static void set(String key,String value){if(value==null)System.clearProperty(key);else System.setProperty(key,value);}
    Path imported()throws Exception{Path external=temp.resolve("external/zephyrus.fc");FlightComputerDesign.zephyrus().write(external);return FlightComputerLibrary.importFile(external);}
    FlightComputerDesign parameter(FlightComputerDesign d,String k,double v){return d.with("parameters",Json.createObjectBuilder(d.parameters()).add(k,v).build());}
    @Test void templateIsRunnableAndRoundTripsWithoutInformationLoss()throws Exception{
        var d=FlightComputerDesign.zephyrus();d.requireRunnable();var path=FlightComputerLibrary.template();assertEquals(d.json(),FlightComputerDesign.read(path).json());assertTrue(FlightComputerLibrary.protectedFile(path));
        assertEquals(3000,d.timing().sensorReadUs());assertEquals("baro",d.active("barometer").getString("id"));
    }
    @Test void importingCopiesDeduplicatesAndNeverOverwritesNames()throws Exception{
        var first=imported();var bytes=Files.readAllBytes(first);assertEquals(first,FlightComputerLibrary.importFile(temp.resolve("external/zephyrus.fc")));
        Path secondSource=temp.resolve("other/zephyrus.fc");parameter(FlightComputerDesign.zephyrus(),"extraWorkUs",9000).write(secondSource);var second=FlightComputerLibrary.importFile(secondSource);
        assertNotEquals(first,second);assertArrayEquals(bytes,Files.readAllBytes(first));assertArrayEquals(Files.readAllBytes(secondSource),Files.readAllBytes(second));
        Files.delete(secondSource);assertTrue(Files.exists(second));assertThrows(java.io.IOException.class,()->FlightComputerLibrary.resolve("library:../../escape.fc"));
    }
    @Test void layoutOnlyAndNameEditsDoNotInvalidateSemantics(){
        var d=FlightComputerDesign.zephyrus();var n=d.node("baro");var nodes=Json.createArrayBuilder();for(var item:d.nodes().getValuesAs(JsonObject.class))nodes.add(item==n?Json.createObjectBuilder(item).add("name","Renamed barometer").add("layout",Json.createObjectBuilder().add("x",999).add("y",4).add("collapsed",false)).build():item);
        assertEquals(d.fingerprint(),d.with("nodes",nodes.build()).fingerprint());assertNotEquals(d.fingerprint(),parameter(d,"extraWorkUs",1).fingerprint());
    }
    @Test void invalidTopologyAndRulesAreRejected(){
        var d=FlightComputerDesign.zephyrus();var links=Json.createArrayBuilder(d.connections()).add(d.connections().get(0)).build();assertThrows(IllegalArgumentException.class,()->d.with("connections",links).requireRunnable());
        var rules=Json.createArrayBuilder(d.rules()).add(Json.createObjectBuilder().add("id","arbitrary code").add("condition",Json.createObjectBuilder().add("op","==").add("signal","unknown").add("value",0))).build();assertThrows(IllegalArgumentException.class,()->d.with("rules",rules).requireRunnable());
        assertThrows(IllegalArgumentException.class,()->parameter(d,"extraWorkUs",-1).requireRunnable());
    }
    @Test void savedTransitionEditChangesTheActualFlightComputer(){
        var d=FlightComputerDesign.zephyrus();var rules=Json.createArrayBuilder();for(var r:d.rules().getValuesAs(JsonObject.class))rules.add(r.getString("id").equals("launch")?Json.createObjectBuilder(r).add("condition",Json.createObjectBuilder().add("signal","accel_vertical").add("op",">").add("value",1000)).build():r);
        var normal=new RTFC(new Trace(s->{}));normal.configure(d);normal.init();var edited=new RTFC(new Trace(s->{}));edited.configure(d.with("rules",rules.build()));edited.init();
        for(var fc:List.of(normal,edited)){fc.enqueueCommand(RTFC.stateCommand(RTRocketState.PRE_FLIGHT));for(long t=0;t<=30000;t+=10000){fc.pre_loop(t,sample(t,t<20000?9.8065f:100));fc.loop();}}
        assertEquals(RTRocketState.FLIGHT,normal.getState());assertEquals(RTRocketState.PRE_FLIGHT,edited.getState());
    }
    static RTFC.Inputs sample(long t,float accel){return new RTFC.Inputs(t,new RTAccelData(accel,0,0),new RTBaroData(Float.NaN,Float.NaN,20,Float.NaN,1013.25f),new RTGPSData(0.0,0.0,0.0,0.0,0.0,0.0,true),new RTGyroData(0f,0f,0f));}
    @Test void sensorEditsAndStableIdNoiseAreExecuted(){
        var d=FlightComputerDesign.zephyrus();var nodes=Json.createArrayBuilder();for(var n:d.nodes().getValuesAs(JsonObject.class))nodes.add(n.getString("id").equals("accel")?Json.createObjectBuilder(n).add("properties",Json.createObjectBuilder(n.getJsonObject("properties")).add("bias",4).add("noise",.5).add("periodUs",10000)).build():n);
        d=d.with("nodes",nodes.build());var a=new FlightComputerSensors(d);var b=new FlightComputerSensors(d);var original=sample(0,10);var aa=a.sample(0,original);var bb=b.sample(0,original);assertEquals(aa.accel().getAccelX(),bb.accel().getAccelX());assertNotEquals(10,aa.accel().getAccelX());assertNull(a.sample(1,original).accel());
    }
    @Test void defaultFileMatchesExistingFlightIntegration()throws Exception{
        var a=RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());var b=RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());a.getOptions().setMaxSimulationTime(.15);b.getOptions().setMaxSimulationTime(.15);
        a.simulate(new FlightControllerSimulatorListener(s->{},.0025,false));ZephyrusFlightComputer.apply(b,true,TelemetryLinkSettings.DEFAULT);ZephyrusFlightComputer.selectFile(b,imported());b.simulate();
        assertEquals(a.getSimulatedData().getMaxAltitude(),b.getSimulatedData().getMaxAltitude(),1e-10);assertEquals(a.getSimulatedData().getMaxVelocity(),b.getSimulatedData().getMaxVelocity(),1e-10);assertEquals(FlightComputerDesign.zephyrus().fingerprint(),b.getSimulatedData().getFlightComputerProvenance().get("semanticHash"));
    }
    @Test void changedAndMissingFilesPreserveResultsButMarkThemOutdated()throws Exception{
        var s=RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());s.getOptions().setMaxSimulationTime(.03);ZephyrusFlightComputer.apply(s,true,TelemetryLinkSettings.DEFAULT);var path=imported();ZephyrusFlightComputer.selectFile(s,path);s.simulate();var result=s.getSimulatedData();assertEquals(Simulation.Status.UPTODATE,s.getStatus());
        parameter(FlightComputerDesign.read(path),"extraWorkUs",5).write(path);assertEquals(Simulation.Status.OUTDATED,s.getStatus());Files.delete(path);assertThrows(info.openrocket.core.simulation.exception.SimulationException.class,s::simulate);assertSame(result,s.getSimulatedData());
    }
    @Test void ensembleFreezesDesignDespiteExternalEdits()throws Exception{
        var s=RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());s.getOptions().setMaxSimulationTime(.04);s.getOptions().setEnsembleSettings(new EnsembleSettings(true,2,0,.05,0,0,0,123));
        var path=imported();var initial=FlightComputerDesign.read(path);ZephyrusFlightComputer.apply(s,true,TelemetryLinkSettings.DEFAULT);ZephyrusFlightComputer.selectFile(s,path);var changed=new java.util.concurrent.atomic.AtomicBoolean();
        s.simulate(new AbstractSimulationListener(){public void postStep(info.openrocket.core.simulation.SimulationStatus status){if(changed.compareAndSet(false,true))try{parameter(initial,"extraWorkUs",50000).write(path);}catch(Exception e){throw new RuntimeException(e);}}});
        assertTrue(changed.get());var result=s.getSimulatedData();assertEquals(initial.fingerprint(),result.getFlightComputerProvenance().get("semanticHash"));var doc=OpenRocketDocumentFactory.createDocumentFromRocket(s.getRocket());
        var runs=result.getEnsembleResult().individualRuns();for(int i=0;i<2;i++)assertEquals(initial.fingerprint(),runs.read(i,doc).getFlightComputerProvenance().get("semanticHash"));
        assertEquals(Simulation.Status.OUTDATED,s.getStatus());
    }
    @Test void orkStoresReferenceAndResultProvenanceOnly()throws Exception{
        var s=RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());s.getOptions().setMaxSimulationTime(.03);ZephyrusFlightComputer.apply(s,true,TelemetryLinkSettings.DEFAULT);ZephyrusFlightComputer.selectFile(s,imported());s.simulate();
        var doc=OpenRocketDocumentFactory.createDocumentFromRocket(s.getRocket());doc.addSimulation(s);var storage=new StorageOptions();storage.setSaveSimulationData(true);var out=new java.io.ByteArrayOutputStream();
        new info.openrocket.core.file.openrocket.OpenRocketSaver().save(out,doc,storage,new info.openrocket.core.logging.WarningSet(),new info.openrocket.core.logging.ErrorSet());String xml=out.toString(java.nio.charset.StandardCharsets.UTF_8);
        assertTrue(xml.contains("library:zephyrus.fc"));assertTrue(xml.contains("<fcprovenance"));assertFalse(xml.contains("openrocket.flight-computer"));assertFalse(xml.contains("filterSamples"));assertFalse(xml.contains("sensorReadUs"));
        var context=new info.openrocket.core.file.DocumentLoadingContext();context.setOpenRocketDocument(OpenRocketDocumentFactory.createEmptyRocket());context.setAttachmentFactory(new info.openrocket.core.file.FileSystemAttachmentFactory(temp.toFile()));
        var db=new info.openrocket.core.database.motor.ThrustCurveMotorSetDatabase();for(var motor:s.getActiveConfiguration().getAllMotors())db.addMotor((info.openrocket.core.motor.ThrustCurveMotor)motor.getMotor());context.setMotorFinder((type,manufacturer,designation,diameter,length,digest,warnings)->db.findMotors(digest,type,manufacturer,designation,diameter,length).stream().findFirst().orElse(null));
        new info.openrocket.core.file.openrocket.importt.OpenRocketLoader().loadFromStream(context,new java.io.ByteArrayInputStream(out.toByteArray()),"test.ork");
        assertEquals(s.getSimulatedData().getFlightComputerProvenance(),context.getOpenRocketDocument().getSimulation(0).getSimulatedData().getFlightComputerProvenance());
        var reopened=context.getOpenRocketDocument().getSimulation(0).getSimulatedData().getBranch(0);assertNotNull(reopened.get(FlightComputerData.PWM[0]));assertEquals(s.getSimulatedData().getBranch(0).get(FlightComputerData.PRESSURE.measured()),reopened.get(FlightComputerData.PRESSURE.measured()));
    }
    @Test void benchUsesExistingSchedulerAndReportsOverruns(){
        var d=parameter(FlightComputerDesign.zephyrus(),"extraWorkUs",10000);var result=FlightControllerSimulatorListener.bench(d,100000,t->sample(t,9.8065f),s->{});
        assertTrue(result.timing().overruns()>0);assertEquals(13000,result.timing().maxExecutionUs());assertFalse(result.points().isEmpty());assertTrue(Files.exists(result.csv()));
    }
}
