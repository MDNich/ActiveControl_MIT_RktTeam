import com.google.inject.AbstractModule;
import com.google.inject.Guice;
import com.google.inject.util.Modules;
import info.openrocket.core.database.motor.MotorDatabase;
import info.openrocket.core.database.motor.ThrustCurveMotorSetDatabase;
import info.openrocket.core.document.OpenRocketDocument;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.file.GeneralRocketLoader;
import info.openrocket.core.file.GeneralRocketSaver;
import info.openrocket.core.motor.ThrustCurveMotor;
import info.openrocket.core.plugin.PluginModule;
import info.openrocket.core.rocketcomponent.FinSet;
import info.openrocket.core.simulation.FlightEvent;
import info.openrocket.core.startup.Application;
import info.openrocket.core.util.BuildProperties;
import info.openrocket.swing.startup.GuiModule;

import java.io.File;
import java.io.ObjectInputStream;
import java.nio.file.Path;
import java.util.List;

/** Compile and run using ONLY the downloaded official 24.12 JAR, never MIT classes. */
public final class Vanilla24CompatibilityProbe {
    public static void initialize() throws Exception {
        System.setProperty("openrocket.bypass.presets", "true");
        System.setProperty("openrocket.bypass.motors", "true");
        var database = new ThrustCurveMotorSetDatabase();
        var module = new GuiModule();
        Application.setInjector(Guice.createInjector(Modules.override(module).with(new AbstractModule() {
            @Override protected void configure() {
                bind(ThrustCurveMotorSetDatabase.class).toInstance(database);
                bind(MotorDatabase.class).toInstance(database);
            }
        }), new PluginModule()));
        module.startLoader();
        try (var in = new ObjectInputStream(Vanilla24CompatibilityProbe.class
                .getResourceAsStream("/datafiles/thrustcurves/thrustcurves.ser"))) {
            for (Object motor : (List<?>) in.readObject()) database.addMotor((ThrustCurveMotor) motor);
        }
    }

    public static OpenRocketDocument load(File file) throws Exception {
        var loader = new GeneralRocketLoader(file);
        var document = loader.load();
        if (!loader.getWarnings().isEmpty()) throw new AssertionError(file + ": " + loader.getWarnings());
        return document;
    }

    public static void main(String[] args) {
        try {
            initialize();
            if (!"24.12".equals(BuildProperties.getVersion()))
                throw new AssertionError("Not the official 24.12 runtime: " + BuildProperties.getVersion());
            try {
                Class.forName("info.openrocket.core.file.openrocket.VanillaOpenRocketExporter");
                throw new AssertionError("MIT classes leaked into the official verification JVM");
            } catch (ClassNotFoundException expected) { }
            for (String arg : args) {
                File file = new File(arg);
                var document = load(file);
                int components = 0;
                for (var component : document.getRocket()) {
                    components++;
                    if (component instanceof FinSet fins && Math.abs(fins.getCantAngle()) > Math.toRadians(15))
                        throw new AssertionError("Invalid fin cant");
                }
                for (var simulation : document.getSimulations()) {
                    if (!simulation.getSimulationExtensions().isEmpty() || simulation.getSimulatedData() != null)
                        throw new AssertionError("Simulation was not exported as a clean baseline");
                }
                Path roundtrip = file.toPath().resolveSibling(file.getName() + ".vanilla-roundtrip.ork");
                new GeneralRocketSaver().save(roundtrip.toFile(), document);
                var reloaded = load(roundtrip.toFile());
                if (document.getSimulationCount() != reloaded.getSimulationCount()
                        || !document.getRocket().getIds().equals(reloaded.getRocket().getIds()))
                    throw new AssertionError("Configurations changed across vanilla save/reload");
                for (var component : document.getRocket()) {
                    var copy = reloaded.getRocket().findComponent(component.getID());
                    if (copy == null || !component.getClass().equals(copy.getClass()))
                        throw new AssertionError("Component changed across vanilla save/reload");
                }
                System.out.println("VANILLA24_LOAD_ROUNDTRIP_PASS file=" + file.getName()
                        + " components=" + components + " configurations=" + document.getRocket().getIds().size()
                        + " simulations=" + document.getSimulationCount() + " warnings=0");
                // The augmented example has known runnable motors and tests the actual vanilla solver.
                if (file.getName().startsWith("mixed-features")) {
                    Simulation simulation = document.getSimulation(0);
                    simulation.simulate();
                    var data = simulation.getSimulatedData();
                    if (simulation.hasErrors() || data == null || !Double.isFinite(data.getMaxAltitude())
                            || data.getBranch(0).getFirstEvent(FlightEvent.Type.APOGEE) == null
                            || data.getBranch(0).getFirstEvent(FlightEvent.Type.GROUND_HIT) == null)
                        throw new AssertionError("Vanilla flight did not complete: " + simulation.getStatus());
                    System.out.println("VANILLA24_SIMULATION_PASS apogee_m=" + data.getMaxAltitude()
                            + " flight_s=" + data.getFlightTime());
                }
            }
            System.out.println("VANILLA24_COMPATIBILITY=PASS");
            System.exit(0);
        } catch (Throwable failure) {
            failure.printStackTrace();
            System.exit(1);
        }
    }
}
