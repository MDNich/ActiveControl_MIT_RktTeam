package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.nio.file.*;
import javax.imageio.ImageIO;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import static org.junit.jupiter.api.Assertions.*;

class MissionVideoTest {
    @TempDir Path temp;
    @Test void deterministicStaticAssetsAndSeparateLogicalLossTimeline() throws Exception {
        var draft=MissionDefinition.createUrrg("Video fixture").json();
        var config=Json.createObjectBuilder(draft.getJsonObject("video")).add("frameRateHz",30).add("lossFraction",0)
                .add("delayMs",10).add("outages",Json.createArrayBuilder().add(Json.createObjectBuilder().add("startUs","0").add("endUs","6000000"))).build();
        var mission=Json.createObjectBuilder(draft).add("video",config).build();
        try(var recorder=new MissionRecorder(temp.resolve("stage"),mission)) {
            var video=new MissionVideo(mission,recorder,"member-0",0);video.end(6_200_000);
            var assets=video.applicationStreams();assertEquals(2,assets.size());
            var image=ImageIO.read(Path.of(assets.getJsonObject(0).getString("assetPath")).toFile());assertEquals(960,image.getWidth());assertEquals(540,image.getHeight());
            assertNotEquals(assets.getJsonObject(0).getString("sha256"),assets.getJsonObject(1).getString("sha256"));
            while(video.next()<=6_300_000)video.advance(video.next());
            assertEquals(2,video.state("gs1").size());
            assertTrue(Long.parseLong(video.state("gs1").getJsonObject(0).getString("timeUs"))>=6_000_000);
            Path archive=temp.resolve("video.rktmission");recorder.finish(archive,"complete");
            try(var result=new MissionArchive(archive)){result.verifyAll();assertEquals(2,result.names().stream().filter(n->n.endsWith(".png")).count());}
        }
    }
}
