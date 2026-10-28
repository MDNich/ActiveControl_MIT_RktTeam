package info.openrocket.core.simulation.mission;

import edu.mit.rocket_team.RTFCVerificationRocket;
import info.openrocket.core.simulation.ensemble.*;
import info.openrocket.core.util.BaseTestCase;
import jakarta.json.*;
import java.nio.file.*;
import java.util.*;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import static org.junit.jupiter.api.Assertions.*;

class MissionEnsembleTest extends BaseTestCase {
    @TempDir Path temp;
    @Test void serialAndParallelMembersRetainIdenticalCanonicalStreams() throws Exception {
        String python=System.getenv("MISSION_TEST_PYTHON");Assumptions.assumeTrue(python!=null);System.setProperty("openrocket.mission.python",python);
        try {
            var flight=RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());flight.getOptions().setMaxSimulationTime(2);flight.getOptions().setTimeStep(.05);
            flight.getOptions().setEnsembleSettings(new EnsembleSettings(true,3,0,.05,1,.01,.1,42,0,.01,0,0,0,2));
            var store=new MissionStore(temp.resolve("store"),digest->null);var draft=store.create("Ensemble parity");draft=store.bindFlight(MissionStore.id(draft),0,flight,"ensemble.ork");
            var sites=Json.createArrayBuilder();draft.getJsonArray("sites").forEach(v->sites.add(Json.createObjectBuilder(v.asJsonObject()).add("heightM",250)));
            draft=store.update(MissionStore.id(draft),1,Json.createObjectBuilder(draft).add("sites",sites).add("video",Json.createObjectBuilder(draft.getJsonObject("video")).add("streams",JsonValue.EMPTY_JSON_ARRAY)).build());
            var resolved=store.resolve(MissionStore.id(draft),2);
            for(int parallel:List.of(1,2)) {
                var mission=Json.createObjectBuilder(resolved.mission()).add("execution",Json.createObjectBuilder(resolved.mission().getJsonObject("execution")).add("parallelMembers",parallel)).build();
                try(var recorder=new MissionRecorder(temp.resolve("stage-"+parallel),mission)) {
                    var result=MissionEnsemble.execute(new MissionStore.Resolved(mission,resolved.flightDirectory(),MissionStore.hash(mission)),recorder,temp.resolve("private-"+parallel),new MissionRun.Control(){public void check(){}public void progress(long t,long end,String phase){}},JsonValue.EMPTY_JSON_OBJECT);
                    assertEquals("complete",result.status());assertEquals(3,result.metrics().getInt("completed"));
                    recorder.finish(temp.resolve("result-"+parallel+".rktmission"),result.status());
                }
            }
            try(var serial=new MissionArchive(temp.resolve("result-1.rktmission"));var parallel=new MissionArchive(temp.resolve("result-2.rktmission"))) {
                for(String name:serial.names()) if(name.startsWith("runs/") && !name.endsWith("run.json"))
                    assertArrayEquals(serial.read(name,16*1024*1024),parallel.read(name,16*1024*1024),name);
                assertNotEquals(new String(serial.read("runs/member-0/ensemble-inputs.json",10000)),new String(serial.read("runs/member-1/ensemble-inputs.json",10000)));
            }
            var failureMission=Json.createObjectBuilder(resolved.mission()).add("execution",Json.createObjectBuilder(resolved.mission().getJsonObject("execution"))
                    .add("clockMode","paced").add("applicationMember",1)).build();
            try(var recorder=new MissionRecorder(temp.resolve("partial"),failureMission)) {
                var outcome=MissionEnsemble.execute(new MissionStore.Resolved(failureMission,resolved.flightDirectory(),MissionStore.hash(failureMission)),recorder,temp.resolve("failure-private"),
                        new MissionRun.Control(){public void check(){} public void progress(long t,long end,String phase){if(phase.startsWith("Member 2/"))throw new IllegalStateException("Injected independent member failure");}},JsonValue.EMPTY_JSON_OBJECT);
                assertEquals("complete-with-failures",outcome.status());assertEquals(2,outcome.metrics().getInt("completed"));
                assertTrue(Files.exists(recorder.root().resolve("runs/member-1/failure.json")));
                assertTrue(Files.exists(recorder.root().resolve("runs/member-2/metrics.json")));
            }
        } finally {System.clearProperty("openrocket.mission.python");}
    }
    @Test void informationAgeAndFreshnessHaveExplicitDifferentDenominators() {
        var metrics=new MissionReceptionMetrics();metrics.advance(0,false,10,false);
        metrics.sample(Json.createObjectBuilder().add("gps_fix",3).add("latitude",42).add("longitude",-77)
                .add("details",Json.createObjectBuilder().add("acquisitionTimeUs","-500000")).build(),0,true);
        metrics.advance(3_000_000,true,20,true);var result=metrics.json(4_000_000);
        assertEquals("2000000",result.getString("freshUs"));assertEquals("1000000",result.getString("trackingUs"));
        assertEquals(.4,result.getJsonNumber("freshFraction").doubleValue(),1e-10);
        assertEquals(2.5,result.getJsonNumber("meanInformationAgeSeconds").doubleValue(),1e-10);
    }
}
