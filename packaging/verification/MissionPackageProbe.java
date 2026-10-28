import info.openrocket.core.simulation.mission.*;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.simulation.flightcomputer.*;
import info.openrocket.core.startup.Application;
import info.openrocket.core.plugin.PluginModule;
import info.openrocket.swing.startup.GuiModule;
import edu.mit.rocket_team.RTFCVerificationRocket;
import jakarta.json.*;
import java.nio.file.*;

/** Run standard and coupled fixtures against a distribution JAR, bundled Java and bundled Python. */
public final class MissionPackageProbe {
    public static void main(String[] args) throws Exception {
        if (MissionPackageProbe.class.getClassLoader().getParent() != ClassLoader.getPlatformClassLoader())
            throw new AssertionError("Probe must run through the real isolated application loader");
        Class.forName("com.sun.net.httpserver.HttpServer");
        Class.forName("java.net.http.HttpClient");
        if (MissionPackageProbe.class.getResource("/ReleaseNotes.md") == null ||
                info.openrocket.core.communication.WelcomeInfoRetriever.retrieveWelcomeInfo() == null)
            throw new AssertionError("Packaged welcome notes are missing");
        Path root=Path.of(args[0]).toAbsolutePath();Files.createDirectories(root);
        if(System.getProperty("openrocket.mission.python")!=null || System.getProperty("openrocket.mission.runtime")!=null)throw new AssertionError("Probe must discover the packaged runtime");
        System.setProperty("openrocket.bypass.motors","true");System.setProperty("openrocket.bypass.presets","true");
        var module=new GuiModule();Application.setInjector(com.google.inject.Guice.createInjector(module,new PluginModule()));module.startLoader();
        var reports=Json.createArrayBuilder();
        for(String mode:new String[]{"standard","custom-fc"}) {
            Path folder=root.resolve(mode);Files.createDirectories(folder);
            var flight=RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());flight.getOptions().setTimeStep(.05);flight.getOptions().setMaxSimulationTime(2);
            if(mode.equals("custom-fc")) {
                var design=new FlightComputerDesign(Json.createObjectBuilder(FlightComputerDesign.zephyrus().json()).add("version",3)
                        .add("telemetryFormat",MissionJson.resource("codec-vectors.json").getJsonObject("codec")).build());
                Path file=folder.resolve("custom.fc");design.write(file);
                var fc=new ZephyrusFlightComputer();fc.setMissionDesign(new FlightComputerLibrary.Resolved(design,file,MissionFlightSnapshot.hash(file)));
                var config=fc.getConfig();config.put("enabled",true);fc.setConfig(config);flight.getSimulationExtensions().add(fc);
            }
            var store=new MissionStore(folder.resolve("store"),digest->null);var draft=store.create("Packaged mission qualification");
            draft=store.bindFlight(MissionStore.id(draft),0,flight,"qualification.ork");
            var sites=Json.createArrayBuilder();draft.getJsonArray("sites").forEach(v->sites.add(Json.createObjectBuilder(v.asJsonObject()).add("heightM",250)));
            draft=store.update(MissionStore.id(draft),1,Json.createObjectBuilder(draft).add("sites",sites)
                    .add("telemetry",Json.createObjectBuilder(draft.getJsonObject("telemetry")).add("mode",mode))
                    .add("execution",Json.createObjectBuilder(draft.getJsonObject("execution")).add("mode",mode.equals("custom-fc")?"coupled":"ground-link")).build());
            var resolved=store.resolve(MissionStore.id(draft),2);Path archive=folder.resolve("qualification.rktmission");
            JsonObject metrics;
            try(var recorder=new MissionRecorder(folder.resolve("staging"),resolved.mission())) {
                try(var inputs=Files.list(resolved.flightDirectory())){for(var input:inputs.toList())recorder.asset("assets/"+resolved.mission().getJsonObject("flight").getString("snapshotHash")+"/"+input.getFileName(),input);}
                metrics=new MissionRun(resolved,recorder,"member-0",0,new MissionRun.Control(){public void check(){}public void progress(long t,long end,String phase){}}).execute(folder.resolve("private"));
                if(metrics.getJsonObject("receivers").getJsonObject("gs1").getInt("acceptedFrames")<20)throw new AssertionError("Station did not receive the packaged fixture");
                MissionResults.index(recorder,folder.resolve("utilities"));recorder.finish(archive,"complete");
            }
            try(var result=new MissionArchive(archive)){result.verifyAll();if(result.names().stream().noneMatch(n->n.endsWith(".png")))throw new AssertionError("No video slate");}
            var summary=MissionResults.query(archive,folder.resolve("utilities"),JsonValue.EMPTY_JSON_OBJECT);
            if(summary.getJsonArray("streams").isEmpty())throw new AssertionError("No indexed result channels");
            var recordedDraft=store.deriveResult(archive,"member-0","recorded-tx");
            var recorded=store.resolve(MissionStore.id(recordedDraft),0);int replayAccepted;
            try(var recorder=new MissionRecorder(folder.resolve("recorded-staging"),recorded.mission())) {
                var result=new MissionRun(recorded,recorder,"member-0",0,new MissionRun.Control(){public void check(){}public void progress(long t,long end,String phase){}}).execute(folder.resolve("recorded-private"));
                replayAccepted=result.getJsonObject("receivers").getJsonObject("gs1").getInt("acceptedFrames");
                if(replayAccepted!=metrics.getJsonObject("receivers").getJsonObject("gs1").getInt("acceptedFrames"))throw new AssertionError("Saved packet experiment changed unmodified reception");
            }
            reports.add(Json.createObjectBuilder().add("recordedAccepted",replayAccepted).add("mode",mode).add("accepted",metrics.getJsonObject("receivers").getJsonObject("gs1").getInt("acceptedFrames")));
        }
        MissionJson.writeAtomic(root.resolve("report.json"),Json.createObjectBuilder().add("passed",true).add("os",System.getProperty("os.name"))
                .add("architecture",System.getProperty("os.arch")).add("javaHome",System.getProperty("java.home")).add("python",MissionWorker.interpreter().toString()).add("fixtures",reports).build());
        System.out.println("PACKAGED_MISSION_QUALIFICATION=PASS");System.exit(0);
    }
}
