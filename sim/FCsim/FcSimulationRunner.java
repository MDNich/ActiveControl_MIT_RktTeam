import com.google.inject.*;
import com.google.inject.util.Modules;
import info.openrocket.core.database.MotorDatabaseLoader;
import info.openrocket.core.database.motor.MotorDatabase;
import info.openrocket.core.startup.Application;
import info.openrocket.core.plugin.PluginModule;
import info.openrocket.swing.startup.GuiModule;
import info.openrocket.core.document.*;
import info.openrocket.core.file.*;
import info.openrocket.core.file.motor.GeneralMotorLoader;
import info.openrocket.core.database.motor.ThrustCurveMotorSetDatabase;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import edu.mit.rocket_team.zephyrus.telemetry.TelemetryLinkSettings;
import info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener;
import edu.mit.rocket_team.zephyrus.RTFCVerificationRocket;
import java.nio.file.*;

/** CLI support for the existing simulator/listener, not another simulation engine. */
public class FcSimulationRunner {
    public static void main(String[] args) throws Exception {
        Double loss = null; Integer delay = null, seed = null;
        var positional = new java.util.ArrayList<String>();
        for (String arg : args) {
            if (arg.startsWith("--loss-percent=")) loss = Double.parseDouble(arg.substring(15))/100;
            else if (arg.startsWith("--delay-ms=")) delay = Integer.parseInt(arg.substring(11));
            else if (arg.startsWith("--seed=")) seed = Integer.parseInt(arg.substring(7));
            else positional.add(arg);
        }
        args = positional.toArray(String[]::new);
        if(args.length<1 || !(args[0].equals("--synthetic") || args[0].equals("--inspect") || args[0].equals("--run")))
            throw new IllegalArgumentException("Usage: --synthetic | --inspect ORK [ENG] | --run ORK [ENG]");
        System.setProperty("openrocket.bypass.presets","true");
        // Keep the existing motor loader, with its blocking provider instead of a Swing progress dialog.
        System.setProperty("openrocket.bypass.motors","true");
        MotorDatabaseLoader motorLoader=new MotorDatabaseLoader();
        GuiModule module=new GuiModule();
        Application.setInjector(Guice.createInjector(Modules.override(module).with(new AbstractModule() {
            @Override protected void configure() {
                bind(ThrustCurveMotorSetDatabase.class).toProvider(motorLoader::getDatabase).in(Scopes.SINGLETON);
                bind(MotorDatabase.class).toProvider(motorLoader::getDatabase).in(Scopes.SINGLETON);
            }
        }),new PluginModule()));
        module.startLoader(); motorLoader.startLoading(); motorLoader.blockUntilLoaded();
        OpenRocketDocument doc; Simulation simulation;
        if(args[0].equals("--synthetic")) {
            var rocket=RTFCVerificationRocket.rocket(); simulation=RTFCVerificationRocket.simulation(rocket);
            doc=OpenRocketDocumentFactory.createDocumentFromRocket(rocket); doc.addSimulation(simulation);
        } else {
            if(args.length<2) throw new IllegalArgumentException("Supply the ORK file");
            if(args.length>2) try(var in=Files.newInputStream(Path.of(args[2]))) {
                var db=Application.getInjector().getInstance(ThrustCurveMotorSetDatabase.class);
                for(var motor:new GeneralMotorLoader().load(in,Path.of(args[2]).getFileName().toString())) db.addMotor(motor.build());
            }
            var loader=new GeneralRocketLoader(Path.of(args[1]).toFile()); doc=loader.load();
            if(doc.getSimulationCount()==0) throw new IllegalArgumentException("No saved simulation");
            simulation=doc.getSimulation(0);
            System.out.println("FC_RUNNER source="+Path.of(args[1]).toAbsolutePath()+" simulation="+simulation.getName()+" loader_warnings="+loader.getWarnings());
            if(!simulation.getActiveConfiguration().hasMotors()) throw new IllegalArgumentException("The selected configuration has no resolved motor; supply its ENG/RSE file");
            if(args[0].equals("--inspect")) {
                System.out.println("FC_RUNNER inspection=PASS rocket="+doc.getRocket().getName()+" launch_lat="+simulation.getOptions().getLaunchLatitude()+" launch_lon="+simulation.getOptions().getLaunchLongitude()+" launch_alt_m="+simulation.getOptions().getLaunchAltitude());
                System.exit(0); return;
            }
        }
        boolean hasFc = simulation.getSimulationExtensions().stream().anyMatch(ZephyrusFlightComputer::isFlightComputer);
        var saved = ZephyrusFlightComputer.read(simulation);
        var link = saved.getLinkSettings();
        if (!hasFc || loss != null || delay != null || seed != null) {
            ZephyrusFlightComputer.apply(simulation, !hasFc || saved.isEnabled(), new TelemetryLinkSettings(
                loss == null ? link.packetLossFraction() : loss,
                delay == null ? link.delayMs() : delay, seed == null ? link.randomSeed() : seed));
        }
        final FlightControllerSimulatorListener[] captured = {null};
        simulation.simulate(new info.openrocket.core.simulation.listeners.AbstractSimulationListener() {
            @Override public void startSimulation(info.openrocket.core.simulation.SimulationStatus status) {
                captured[0] = FlightControllerSimulatorListener.active(status);
            }
        });
        var listener = captured[0];
        if (listener == null) {
            System.out.println("FC_RUNNER result=PASS flight_computer=disabled");
            System.exit(0); return;
        }
        var fc=listener.getFlightComputer();
        System.out.println("FC_RUNNER result=PASS loops="+fc.getLoopCount()+" state="+fc.getState()+" max_altitude_m="+simulation.getSimulatedData().getMaxAltitude());
        System.out.println("FC_RUNNER telemetry="+fc.telemetry.getCsvPath()+" packets="+fc.telemetry.getPacketPath());
        if(args[0].equals("--synthetic")) {
            Path example=fc.telemetry.getCsvPath().getParent().resolve("synthetic-fc-verification.ork");
            doc.getDefaultStorageOptions().setSaveSimulationData(true); new GeneralRocketSaver().save(example.toFile(),doc);
            System.out.println("FC_RUNNER synthetic_ork="+example);
        }
        System.exit(0);
    }
}
