package info.openrocket.core.simulation.mission;

import edu.mit.rocket_team.RTFCVerificationRocket;
import info.openrocket.core.document.*;
import info.openrocket.core.util.BaseTestCase;
import java.nio.file.*;

/** Development-only browser acceptance host; never included in distribution JARs. */
public final class MissionWorkspacePreview {
    public static void main(String[] args) throws Exception {
        BaseTestCase.setUp();
        if(System.getenv("MISSION_TEST_PYTHON")!=null)System.setProperty("openrocket.mission.python",System.getenv("MISSION_TEST_PYTHON"));
        Path root = Path.of(args[0]);
        var source = RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket()); source.getOptions().setTimeStep(.05);
        source.getOptions().setMaxSimulationTime(4);
        var document = OpenRocketDocumentFactory.createDocumentFromRocket(source.getRocket()); document.addSimulation(source);
        var store = new MissionStore(root, digest -> null);
        var draft = store.create("URRG mission acceptance");
        draft = store.bindFlight(MissionStore.id(draft), 0, source, "verification.ork");
        var api = new MissionApi(store, new MissionApi.Host() {
            public Path choose(String purpose, boolean save) throws Exception {
                Path selection = root.resolve("chooser-next-path");
                return Files.exists(selection) ? Path.of(Files.readString(selection).strip()) : null;
            }
            public OpenRocketDocument currentDocument() { return document.copy(); }
        }, new MissionJobs(root));
        Runtime.getRuntime().addShutdownHook(new Thread(api::close));
        Path address = root.resolve("workspace-url"); Files.writeString(address, api.uri(MissionStore.id(draft)).toString());
        try { Files.setPosixFilePermissions(address, java.nio.file.attribute.PosixFilePermissions.fromString("rw-------")); }
        catch (UnsupportedOperationException ignored) { }
        new java.util.concurrent.CountDownLatch(1).await();
    }
}
