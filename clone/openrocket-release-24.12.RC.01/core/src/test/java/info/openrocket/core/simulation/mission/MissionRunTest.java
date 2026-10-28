package info.openrocket.core.simulation.mission;

import edu.mit.rocket_team.RTFCVerificationRocket;
import info.openrocket.core.util.BaseTestCase;
import jakarta.json.*;
import java.nio.file.*;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import static org.junit.jupiter.api.Assertions.*;

class MissionRunTest extends BaseTestCase {
    @TempDir Path temp;
    private MissionStore.Resolved ready() throws Exception {
        var store = new MissionStore(temp.resolve("inputs"), digest -> null); var draft = store.create("Headless mission");
        var flight = RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());
        flight.getOptions().setTimeStep(.05); flight.getOptions().setMaxSimulationTime(8);
        draft = store.bindFlight(MissionStore.id(draft), 0, flight, "verification.ork");
        var sites = Json.createArrayBuilder(); for (var entry : draft.getJsonArray("sites")) sites.add(Json.createObjectBuilder(entry.asJsonObject()).add("heightM", 250));
        draft = store.update(MissionStore.id(draft), 1, Json.createObjectBuilder(draft).add("sites", sites)
                .add("video", Json.createObjectBuilder(draft.getJsonObject("video")).add("streams", Json.createArrayBuilder()))
                .add("telemetry", Json.createObjectBuilder(draft.getJsonObject("telemetry")).add("links", Json.createArrayBuilder()
                        .add(Json.createObjectBuilder().add("receiverId", "gs2").add("packetLossFraction", 1)))).build());
        return store.resolve(MissionStore.id(draft), 2);
    }
    @Test void completeHeadlessFlightUsesFiveIndependentReceiversAndReopensWithoutInputs() throws Exception {
        String python = System.getenv("MISSION_TEST_PYTHON"); Assumptions.assumeTrue(python != null);
        System.setProperty("openrocket.mission.python", python);
        var resolved = ready(); var staging = temp.resolve("staging"); var archive = temp.resolve("result.rktmission");
        try (var recorder = new MissionRecorder(staging, resolved.mission())) {
            var metrics = new MissionRun(resolved, recorder, "member-0", 0, new MissionRun.Control() {
                public void check() { }
                public void progress(long t, long end, String phase) { }
            }).execute(temp.resolve("private"));
            var receivers = metrics.getJsonObject("receivers"); assertEquals(5, receivers.size());
            assertEquals(0, receivers.getJsonObject("gs2").getInt("acceptedFrames"));
            assertTrue(receivers.getJsonObject("gs1").getInt("acceptedFrames") > 20);
            assertTrue(receivers.getJsonObject("gs1").getInt("pointerCommands") > 0);
            assertEquals(0, receivers.getJsonObject("launch").getInt("pointerCommands"));
            assertEquals(receivers.getJsonObject("gs1").getInt("eligibleTransmissions"), receivers.getJsonObject("gs1").getInt("acceptedFrames"));
            recorder.finish(archive, "complete");
        } finally { System.clearProperty("openrocket.mission.python"); }
        try (var result = new MissionArchive(archive)) {
            result.verifyAll(); assertEquals("complete", result.manifest().getString("status"));
            assertTrue(result.names().stream().anyMatch(n -> n.contains("/truth/chunk-")));
            assertFalse(result.names().stream().anyMatch(n -> n.contains("/fc/") || n.contains("private") || n.contains("credential")));
            assertTrue(result.names().stream().anyMatch(n -> n.contains("/pointer/tx/")));
        }
    }
    @Test void separatedStagesKeepStableIdentitiesAndIndependentTrajectories() throws Exception {
        var rocket = info.openrocket.core.util.TestRockets.makeMultiStageEventTestRocket();
        var flight = new info.openrocket.core.document.Simulation(rocket);
        flight.setFlightConfigurationId(rocket.getSelectedConfiguration().getFlightConfigurationID());
        flight.getOptions().setTimeStep(.05); flight.getOptions().setMaxSimulationTime(15);
        var truth = MissionTruth.compute(flight, () -> { });
        assertTrue(truth.branches().size() >= 2);
        var root = truth.branches().get(0); var child = truth.branches().get(1);
        assertNotEquals(root.componentId, child.componentId);
        var parentBinding = Json.createObjectBuilder().add("componentId", root.componentId).build();
        var childBinding = Json.createObjectBuilder().add("componentId", child.componentId).build();
        assertArrayEquals(truth.at(parentBinding, 0).ecef(), truth.at(childBinding, 0).ecef(), 1e-8);
        long after = Math.min(root.end(), child.end());
        double[] parent = truth.at(parentBinding, after).ecef(), separate = truth.at(childBinding, after).ecef();
        double distance = Math.sqrt(Math.pow(parent[0]-separate[0],2)+Math.pow(parent[1]-separate[1],2)+Math.pow(parent[2]-separate[2],2));
        assertTrue(distance > 1, "Separated branch trajectories must diverge");
    }
    @Test void pausedJobFreezesClockAndCancellationProducesReadablePrefix() throws Exception {
        String python = System.getenv("MISSION_TEST_PYTHON"); Assumptions.assumeTrue(python != null);
        System.setProperty("openrocket.mission.python", python);
        var resolved = ready(); var paced = Json.createObjectBuilder(resolved.mission()).add("execution",
                Json.createObjectBuilder(resolved.mission().getJsonObject("execution")).add("clockMode", "paced")).build();
        try (var jobs = new MissionJobs(temp.resolve("runtime"))) {
            String id = jobs.start(new MissionStore.Resolved(paced,resolved.flightDirectory(),MissionStore.hash(paced))).getString("id");
            long deadline = System.nanoTime()+10_000_000_000L;
            JsonObject status;
            do { Thread.sleep(10); status=jobs.request(id,"status",JsonValue.EMPTY_JSON_OBJECT); }
            while (!status.getString("phase").startsWith("Replaying") && System.nanoTime()<deadline);
            assertEquals("running", status.getString("status"));
            jobs.request(id,"pause",JsonValue.EMPTY_JSON_OBJECT); Thread.sleep(100);
            String stopped = jobs.request(id,"status",JsonValue.EMPTY_JSON_OBJECT).getString("timeUs");
            Thread.sleep(150); assertEquals(stopped,jobs.request(id,"status",JsonValue.EMPTY_JSON_OBJECT).getString("timeUs"));
            jobs.request(id,"cancel",JsonValue.EMPTY_JSON_OBJECT);
            do { Thread.sleep(20); status=jobs.request(id,"status",JsonValue.EMPTY_JSON_OBJECT); }
            while (status.getString("result").isEmpty() && System.nanoTime()<deadline);
            assertEquals("cancelled",status.getString("status"));
            try (var archive=new MissionArchive(Path.of(status.getString("result")))) { archive.verifyAll(); assertEquals("cancelled",archive.manifest().getString("status")); }
        } finally { System.clearProperty("openrocket.mission.python"); }
    }
    @Test void geodesyRoundTripsAndQuaternionInterpolationUsesShortestArc() {
        for (var point : new double[][] {{42.7,-77.2,215}, {89.9,179.9,4000}, {-40,12,-30}, {90,0,100}})
            assertArrayEquals(point, MissionGeodesy.geodetic(MissionGeodesy.ecef(point[0],point[1],point[2])), 1e-6);
        assertArrayEquals(new double[] {1,0,0,0}, MissionTruth.slerp(new double[] {1,0,0,0}, new double[] {-1,0,0,0}, .5), 1e-12);
    }
}
