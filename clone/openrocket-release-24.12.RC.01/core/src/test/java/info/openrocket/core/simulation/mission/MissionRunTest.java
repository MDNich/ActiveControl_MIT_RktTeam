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
    @org.junit.jupiter.params.ParameterizedTest
    @org.junit.jupiter.params.provider.ValueSource(booleans={false,true})
    void actualTelemetryApplicationOwnsOneReceiverAndArchivesItsNativeSession(boolean attach) throws Exception {
        String python = System.getenv("MISSION_TEST_PYTHON"), source = System.getenv("MISSION_TEST_UI");
        Assumptions.assumeTrue(python != null && source != null);
        System.setProperty("openrocket.mission.python", python);
        var original = ready(); var stations = Json.createArrayBuilder();
        for (var value : original.mission().getJsonArray("stationBindings")) {
            var station = value.asJsonObject(); stations.add(Json.createObjectBuilder(station).add("owner", station.getString("siteId").equals("gs1") ? "interactive-ui" : "headless-policy"));
        }
        var mission = Json.createObjectBuilder(original.mission()).add("stationBindings", stations).add("execution",
                Json.createObjectBuilder(original.mission().getJsonObject("execution")).add("actions", Json.createArrayBuilder().add(Json.createObjectBuilder()
                        .add("id", "start-ui-policy").add("timeUs", "0").add("siteId", "gs1").add("kind", "start-tracking")))).build();
        var resolved = new MissionStore.Resolved(mission, original.flightDirectory(), MissionStore.hash(mission));
        var preferences=Json.createObjectBuilder().add("sourceProject",source).add("pythonExecutable",python).build();
        assertTrue(MissionWorker.applicationCapabilities(preferences).getJsonArray("capabilities").contains(Json.createValue("ownership-transfer.v1")));
        Process waiting=null;
        if(attach) {
            Path ticket=temp.resolve("waiting.json");
            waiting=new ProcessBuilder(python,"-I","-c","import sys,runpy;sys.path.insert(0,sys.argv.pop(1));runpy.run_module('rocket_gnc_monitor',run_name='__main__')",Path.of(source).resolve("src").toString(),"--mission-listen",ticket.toString())
                    .redirectOutput(temp.resolve("attach.stdout").toFile()).redirectError(temp.resolve("attach.stderr").toFile()).start();
            long deadline=System.nanoTime()+15_000_000_000L;while(!Files.exists(ticket) && System.nanoTime()<deadline)Thread.sleep(50);
            assertTrue(MissionAttachment.check(ticket).getBoolean("ready"));
            String binding=mission.getJsonArray("stationBindings").stream().map(JsonValue::asJsonObject).filter(b->b.getString("siteId").equals("gs1")).findFirst().orElseThrow().getString("id");
            preferences=Json.createObjectBuilder(preferences).add("attachments",Json.createObjectBuilder().add(binding,ticket.toString())).build();
        }
        try (var recorder = new MissionRecorder(temp.resolve("app-staging"), mission)) {
            var metrics = new MissionRun(resolved, recorder, "member-0", 0, new MissionRun.Control() {
                public void check() { }
                MissionParticipants participants; boolean transferred;
                public void participants(MissionParticipants p){participants=p;}
                public void progress(long t, long end, String phase) {
                    if(participants!=null && t>=1_000_000 && !transferred) {
                        try {
                            participants.pause(true,t);
                            assertEquals("headless-policy",participants.transfer("gs1","headless-policy",t).getString("owner"));
                            assertEquals("interactive-ui",participants.transfer("gs1","interactive-ui",t).getString("owner"));
                            participants.pause(false,t);transferred=true;
                        } catch(java.io.IOException e){throw new java.io.UncheckedIOException(e);}
                    }
                }
            }).applicationPreferences(preferences).execute(temp.resolve("app-private"));
            assertTrue(metrics.getJsonObject("receivers").getJsonObject("gs1").getInt("pointerCommands") > 0);
            Path archive = temp.resolve("application.rktmission"); recorder.finish(archive, "complete");
            try (var result = new MissionArchive(archive)) {
                result.verifyAll();
                assertTrue(result.names().stream().anyMatch(n -> n.endsWith("session.sqlite")));
                assertTrue(result.names().stream().anyMatch(n -> n.endsWith("mission-clock.jsonl")));
                assertFalse(result.names().stream().anyMatch(n -> n.contains("private.json")));
                assertTrue(result.names().stream().anyMatch(n -> n.contains("ownership/events/")));
            }
        } finally { if(waiting!=null){assertTrue(waiting.isAlive());waiting.destroy();waiting.waitFor();}System.clearProperty("openrocket.mission.python"); }
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
    @Test void actualCustomFcRunsThroughDeclaredCodecAndKeepsOriginalDiagnostics() throws Exception {
        String python = System.getenv("MISSION_TEST_PYTHON"); Assumptions.assumeTrue(python != null);
        System.setProperty("openrocket.mission.python", python);
        var store = new MissionStore(temp.resolve("fc-inputs"), digest -> null);
        var flight = RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket()); flight.getOptions().setTimeStep(.05); flight.getOptions().setMaxSimulationTime(2);
        var base = info.openrocket.core.simulation.flightcomputer.FlightComputerDesign.zephyrus();
        var design = new info.openrocket.core.simulation.flightcomputer.FlightComputerDesign(Json.createObjectBuilder(base.json()).add("version",3)
                .add("telemetryFormat",MissionJson.resource("codec-vectors.json").getJsonObject("codec")).build());
        Path source=temp.resolve("custom.fc");design.write(source);
        var fc = new info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer();
        fc.setMissionDesign(new info.openrocket.core.simulation.flightcomputer.FlightComputerLibrary.Resolved(design,source,MissionFlightSnapshot.hash(source)));
        var config=fc.getConfig();config.put("enabled",true);fc.setConfig(config);flight.getSimulationExtensions().add(fc);
        var draft=store.create("Coupled custom codec");draft=store.bindFlight(MissionStore.id(draft),0,flight,"custom.ork");
        var sites=Json.createArrayBuilder();for(var site:draft.getJsonArray("sites"))sites.add(Json.createObjectBuilder(site.asJsonObject()).add("heightM",250));
        draft=store.update(MissionStore.id(draft),1,Json.createObjectBuilder(draft).add("sites",sites)
                .add("telemetry",Json.createObjectBuilder(draft.getJsonObject("telemetry")).add("mode","custom-fc"))
                .add("execution",Json.createObjectBuilder(draft.getJsonObject("execution")).add("mode","coupled").add("actions", Json.createArrayBuilder()
                    .add(Json.createObjectBuilder().add("id","gps-zero").add("kind","uplink").add("siteId","launch").add("timeUs","-700000").add("opcode",10))
                    .add(Json.createObjectBuilder().add("id","unsupported").add("kind","uplink").add("siteId","launch").add("timeUs","-600000").add("dataHex","aa000000000000000000000000070007")))).build());
        var resolved=store.resolve(MissionStore.id(draft),2);
        try(var recorder=new MissionRecorder(temp.resolve("fc-staging"),resolved.mission())){
            var metrics=new MissionRun(resolved,recorder,"member-0",0,new MissionRun.Control(){public void check(){}public void progress(long t,long end,String phase){}}).execute(temp.resolve("fc-private"));
            assertTrue(metrics.getJsonObject("receivers").getJsonObject("gs1").getInt("acceptedFrames")>20);
            assertEquals(0,metrics.getJsonObject("receivers").getJsonObject("gs1").getInt("pointerCommands"),"Missing longitude/fix in codec must not be replaced by truth");
            Path result=temp.resolve("custom.rktmission");recorder.finish(result,"complete");
            try(var archive=new MissionArchive(result)){
                archive.verifyAll();
                assertTrue(archive.names().stream().anyMatch(n->n.endsWith("/fc/balius/OR.log")));
                var events=archive.names().stream().filter(n->n.contains("/events/chunk-")).findFirst().orElseThrow();
                String diagnostics=new String(archive.read(events,1000000),java.nio.charset.StandardCharsets.UTF_8);
                assertTrue(diagnostics.contains("unknown_opcode opcode=7"));
                assertTrue(diagnostics.contains("gps.zero"));
                assertTrue(diagnostics.contains("unsupported"));
                var index=archive.names().stream().filter(n->n.contains("/transmitters/balius/tx/index-")).findFirst().orElseThrow();
                String records=new String(archive.read(index,1000000),java.nio.charset.StandardCharsets.UTF_8);
                for(String line:records.split("\n")){var row=MissionJson.parse(line.getBytes(java.nio.charset.StandardCharsets.UTF_8));assertEquals(14,row.getInt("length"));assertTrue(Long.parseLong(row.getString("txTimeUs"))<=2_000_000);}
            }
        } finally {System.clearProperty("openrocket.mission.python");}
    }
    @Test void geodesyRoundTripsAndQuaternionInterpolationUsesShortestArc() {
        for (var point : new double[][] {{42.7,-77.2,215}, {89.9,179.9,4000}, {-40,12,-30}, {90,0,100}})
            assertArrayEquals(point, MissionGeodesy.geodetic(MissionGeodesy.ecef(point[0],point[1],point[2])), 1e-6);
        assertArrayEquals(new double[] {1,0,0,0}, MissionTruth.slerp(new double[] {1,0,0,0}, new double[] {-1,0,0,0}, .5), 1e-12);
    }
}
