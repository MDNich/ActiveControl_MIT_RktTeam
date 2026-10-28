package info.openrocket.core.simulation.mission;

import edu.mit.rocket_team.RTFCVerificationRocket;
import info.openrocket.core.document.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.util.BaseTestCase;
import jakarta.json.*;
import java.net.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import static org.junit.jupiter.api.Assertions.*;

class MissionWorkspaceTest extends BaseTestCase {
    @TempDir Path temp;
    private Simulation fixture() {
        var simulation = RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());
        simulation.getOptions().setTimeStep(.05); return simulation;
    }
    private JsonObject ready(MissionStore store) throws Exception {
        var draft = store.create("Workspace test"); String id = MissionStore.id(draft);
        draft = store.bindFlight(id, 0, fixture(), "test.ork");
        var sites = Json.createArrayBuilder();
        for (var site : draft.getJsonArray("sites")) sites.add(Json.createObjectBuilder(site.asJsonObject()).add("heightM", 250));
        return store.update(id, 1, Json.createObjectBuilder(draft).add("sites", sites).build());
    }

    @Test void editingAndSnapshotRoundTripIncludesStageRecoveryIgnitionAndEveryOption() throws Exception {
        var source = fixture(); var original = MissionFlightEditor.capture(source);
        var edited = Json.createPatchBuilder().replace("/stages/0/delayS", Json.createValue(2.5))
                .replace("/motors/0/ignitionOverride", JsonValue.TRUE).replace("/motors/0/delayS", Json.createValue(.3))
                .replace("/recovery/0/event", Json.createValue("ALTITUDE")).replace("/recovery/0/altitudeM", Json.createValue(650.0))
                .replace("/options/maximumStepAngleRad", Json.createValue(.04)).build().apply(original);
        MissionFlightSnapshot.capture(source, temp.resolve("original"));
        var copy = MissionFlightSnapshot.restore(temp.resolve("original"));
        MissionFlightEditor.apply(copy, edited, digest -> null);
        assertEquals(edited, MissionFlightEditor.capture(copy));
        assertEquals(original, MissionFlightEditor.capture(source));
        MissionFlightSnapshot.capture(copy, temp.resolve("edited"));
        assertEquals(edited, MissionFlightEditor.capture(MissionFlightSnapshot.restore(temp.resolve("edited"))));
    }

    @Test void revisionsAndAtomicDraftSavesDoNotOverwriteNewerEdits() throws Exception {
        var store = new MissionStore(temp, digest -> null); var first = store.create("First"); String id = MissionStore.id(first);
        assertEquals(7, store.validate(id).size());
        var second = store.update(id, 0, first);
        assertEquals(1, second.getJsonObject("identity").getInt("revision"));
        assertThrows(MissionStore.Conflict.class, () -> store.update(id, 0, first));
        assertEquals(second, store.get(id));
        var path = temp.resolve("saved.json"); store.save(id, path);
        assertEquals(second, MissionDefinition.read(path).json());
        var imported = store.load(MissionDefinition.read(path));
        assertEquals(id, MissionStore.id(imported));
        assertEquals(2, imported.getJsonObject("identity").getInt("revision"));
    }

    @Test void resolutionIsStableAndUsesExplicitSiteDatumWithoutMutatingDraft() throws Exception {
        var store = new MissionStore(temp, digest -> null); var draft = ready(store); String id = MissionStore.id(draft);
        assertTrue(store.validate(id).isEmpty(), store.validate(id).toString());
        var a = store.resolve(id, 2); var b = store.resolve(id, 2);
        assertEquals(a.hash(), b.hash()); assertEquals(draft, store.get(id));
        var site = a.mission().getJsonArray("sites").getJsonObject(0);
        assertEquals(215.43214114497787, site.getJsonNumber("ellipsoidalHeightM").doubleValue(), 1e-9);
        var flight = MissionFlightSnapshot.restore(a.flightDirectory());
        assertEquals(250, flight.getOptions().getLaunchAltitude());
        assertEquals(42.70416776971639, flight.getOptions().getLaunchLatitude());
        assertThrows(MissionStore.Conflict.class, () -> store.resolve(id, 1));
        Files.writeString(a.flightDirectory().resolve("rocket.ork"), "changed");
        assertThrows(java.io.IOException.class, () -> MissionFlightSnapshot.restore(a.flightDirectory()));
    }

    @Test void incompleteSettingsRemainDraftableAndCannotResolve() throws Exception {
        var store = new MissionStore(temp, digest -> null); var draft = ready(store); String id = MissionStore.id(draft);
        var broken = Json.createPatchBuilder().replace("/flight/editor/options/timeStepS", Json.createValue(0)).build().apply(draft);
        store.update(id, 2, broken);
        assertEquals("flight-input", store.validate(id).getJsonObject(0).getString("code"));
        assertThrows(IllegalArgumentException.class, () -> store.resolve(id, 3));
        assertEquals(.05, MissionFlightSnapshot.restore(store.snapshot(draft.getJsonObject("flight").getString("snapshotHash"))).getOptions().getTimeStep());
    }

    @Test void loopbackApiRequiresCredentialAndOriginAndStartsIdempotently() throws Exception {
        var store = new MissionStore(temp, digest -> null); var draft = ready(store); String id = MissionStore.id(draft);
        var starts = new AtomicInteger();
        var jobs = new MissionApi.Jobs() {
            public JsonObject capabilities() { return Json.createObjectBuilder().add("available", true).build(); }
            public JsonObject start(MissionStore.Resolved resolved) { starts.incrementAndGet(); return Json.createObjectBuilder().add("id", "job1").build(); }
            public JsonObject request(String id, String action, JsonObject body) { return JsonValue.EMPTY_JSON_OBJECT; }
            public void close() { }
        };
        var host = new MissionApi.Host() {
            public Path choose(String purpose, boolean save) { return null; }
            public OpenRocketDocument currentDocument() { return null; }
        };
        try (var api = new MissionApi(store, host, jobs)) {
            assertEquals(403, request(api, "GET", "capabilities", null, null, JsonValue.EMPTY_JSON_OBJECT).status());
            assertEquals(403, request(api, "POST", "drafts", api.token(), "https://unrelated.example", JsonValue.EMPTY_JSON_OBJECT).status());
            assertEquals(403, request(api, "POST", "drafts", api.token(), null, JsonValue.EMPTY_JSON_OBJECT).status());
            assertEquals(200, request(api, "GET", "capabilities", api.token(), null, JsonValue.EMPTY_JSON_OBJECT).status());
            String hash = store.resolve(id, 2).hash();
            var start = Json.createObjectBuilder().add("draftId", id).add("expectedRevision", 2).add("resolvedHash", hash).add("idempotencyKey", "a-sufficiently-long-start-key").build();
            assertEquals(200, request(api, "POST", "jobs", api.token(), api.origin(), start).status());
            store.update(id, 2, draft); // Retrying the accepted request still returns the original job.
            assertEquals(200, request(api, "POST", "jobs", api.token(), api.origin(), start).status());
            assertEquals(1, starts.get());
            var conflict = Json.createObjectBuilder(start).add("resolvedHash", "different").build();
            assertEquals(409, request(api, "POST", "jobs", api.token(), api.origin(), conflict).status());
        }
    }
    record Response(int status, String body) { }
    private Response request(MissionApi api, String method, String path, String token, String origin, JsonObject body) throws Exception {
        int port = URI.create(api.origin()).getPort();
        try (var socket = new Socket("127.0.0.1", port)) {
            socket.setSoTimeout(10000); byte[] payload = MissionJson.bytes(body);
            String headers = method + " /mission-api/v1/" + path + " HTTP/1.1\r\nHost: 127.0.0.1:" + port + "\r\nConnection: close\r\nContent-Type: application/json\r\nContent-Length: " + payload.length + "\r\n"
                    + (token == null ? "" : "X-Mission-Token: " + token + "\r\n") + (origin == null ? "" : "Origin: " + origin + "\r\n") + "\r\n";
            socket.getOutputStream().write(headers.getBytes(StandardCharsets.US_ASCII)); socket.getOutputStream().write(payload); socket.getOutputStream().flush();
            String response = new String(socket.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
            int status = Integer.parseInt(response.split(" ", 3)[1]);
            if (status >= 500) fail(response);
            return new Response(status, response.substring(response.indexOf("\r\n\r\n") + 4));
        }
    }
}
