package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.nio.file.*;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import static org.junit.jupiter.api.Assertions.*;

class MissionResultsTest {
    @TempDir Path temp;
    @BeforeEach void python(){String p=System.getenv("MISSION_TEST_PYTHON");Assumptions.assumeTrue(p!=null);System.setProperty("openrocket.mission.python",p);}
    @AfterEach void clean(){System.clearProperty("openrocket.mission.python");}
    @Test void indexedWindowAndRecoveryIgnoreUncommittedTail() throws Exception {
        Path staging=temp.resolve("staging"),utilities=temp.resolve("utilities");Files.createDirectories(utilities);
        try(var recorder=new MissionRecorder(staging,MissionDefinition.createUrrg("Recovery fixture").json())) {
            recorder.raw("runs/member-0/stations/gs1/receiver",0,new byte[]{1,2,3},JsonValue.EMPTY_JSON_OBJECT);
            recorder.event("member-0",Json.createObjectBuilder().add("timeUs","0").add("kind","committed").build());
            recorder.checkpoint("incomplete");
            Files.writeString(staging.resolve("runs/member-0/events/chunk-00000.jsonl"),"uncommitted garbage",StandardOpenOption.APPEND);
            Files.write(staging.resolve("runs/member-0/stations/gs1/receiver/chunk-00000.bin"),new byte[]{9,9},StandardOpenOption.APPEND);
            Path recovered=temp.resolve("recovered.rktmission");MissionResults.recover(staging,recovered,utilities);
            var result=MissionResults.query(recovered,utilities,Json.createObjectBuilder().add("stream","runs/member-0/events").add("startUs","0").add("endUs","0").build());
            assertEquals(1,result.getInt("count"));assertEquals("committed",result.getJsonArray("records").getJsonObject(0).getString("kind"));
            try(var archive=new MissionArchive(recovered)) {
                archive.verifyAll();assertEquals("incomplete",archive.manifest().getString("status"));
                assertArrayEquals(new byte[]{1,2,3},archive.read("runs/member-0/stations/gs1/receiver/chunk-00000.bin",100));
            }
            assertEquals(5,Files.size(staging.resolve("runs/member-0/stations/gs1/receiver/chunk-00000.bin")));
            Files.write(staging.resolve("runs/member-0/stations/gs1/receiver/chunk-00000.bin"),new byte[]{4},StandardOpenOption.WRITE);
            assertThrows(java.io.IOException.class,()->MissionResults.recover(staging,temp.resolve("corrupt.rktmission"),utilities));
        }
    }
    @Test void indexAndAtomicResultOpenWithoutOriginalStaging() throws Exception {
        Path staging=temp.resolve("staging"),archive=temp.resolve("result.rktmission");
        try(var recorder=new MissionRecorder(staging,MissionDefinition.createUrrg("Indexed fixture").json())) {
            for(int i=0;i<100;i++)recorder.event("member-0",Json.createObjectBuilder().add("timeUs",Integer.toString(i*1000)).add("kind","tick").build());
            MissionResults.index(recorder,temp.resolve("utilities"));recorder.finish(archive,"complete");
        }
        try(var files=Files.walk(staging)){for(var file:files.sorted(java.util.Comparator.reverseOrder()).toList())Files.delete(file);}
        var summary=MissionResults.query(archive,temp.resolve("utilities"),JsonValue.EMPTY_JSON_OBJECT);assertEquals(1,summary.getJsonArray("members").size());
        var result=MissionResults.query(archive,temp.resolve("utilities"),Json.createObjectBuilder().add("stream","runs/member-0/events").add("startUs","10000").add("endUs","20000").add("limit",3).build());
        assertEquals(3,result.getInt("count"));assertEquals("10000",result.getJsonArray("records").getJsonObject(0).getString("timeUs"));
    }
}
