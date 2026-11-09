import com.google.inject.AbstractModule;
import com.google.inject.Guice;
import com.google.inject.util.Modules;
import info.openrocket.core.database.motor.MotorDatabase;
import info.openrocket.core.database.motor.ThrustCurveMotorSetDatabase;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.file.GeneralRocketLoader;
import info.openrocket.core.file.GeneralRocketSaver;
import info.openrocket.core.file.motor.RASPMotorLoader;
import info.openrocket.core.plugin.PluginModule;
import info.openrocket.core.models.wind.PinkNoiseWindModel;
import info.openrocket.core.models.wind.MultiLevelPinkNoiseWindModel;
import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.simulation.listeners.AbstractSimulationListener;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.startup.Application;
import info.openrocket.swing.startup.GuiModule;
import java.nio.file.*;
import java.io.PrintWriter;
import static info.openrocket.core.simulation.FlightDataType.*;

/** Compare the first simulation in an .ork with identical inputs and wind realization. */
public final class CompareFlights {
    public static void main(String[] args) throws Exception {
        if (args.length < 3) throw new IllegalArgumentException("Usage: CompareFlights rocket.ork output-directory motor.eng ...");
        Path source = Path.of(args[0]), output = Path.of(args[1]);
        Files.createDirectories(output);
        var database = new ThrustCurveMotorSetDatabase();
        System.setProperty("openrocket.bypass.presets", "true");
        System.setProperty("openrocket.bypass.motors", "true");
        var gui = new GuiModule();
        Application.setInjector(Guice.createInjector(Modules.override(gui).with(new AbstractModule() {
            @Override protected void configure() {
                bind(ThrustCurveMotorSetDatabase.class).toInstance(database);
                bind(MotorDatabase.class).toInstance(database);
            }
        }), new PluginModule()));
        gui.startLoader();
        for (int i = 2; i < args.length; i++) {
            Path curve = Path.of(args[i]);
            try (var stream = Files.newInputStream(curve)) {
                for (var definition : new RASPMotorLoader().load(stream, curve.getFileName().toString())) {
                    var motor = definition.build();
                    database.addMotor(motor);
                    System.out.println("MOTOR " + motor.getDesignation() + " " + motor.getDigest());
                }
            }
        }
        var loader = new GeneralRocketLoader(source.toFile());
        var document = loader.load();
        System.out.println("LOAD " + loader.getWarnings());
        Simulation legacy = document.getSimulation(0);
        legacy.setName("Legacy fins");
        legacy.getOptions().setFiniteWingFins(false);
        var candidate = legacy.duplicateSimulation(document.getRocket());
        candidate.setName("Experimental finite-wing fins");
        candidate.getOptions().setFiniteWingFins(true);
        document.addSimulation(candidate);
        Path copy = output.resolve("fin-model-comparison.ork");
        if (copy.toAbsolutePath().normalize().equals(source.toAbsolutePath().normalize()))
            throw new IllegalArgumentException("Use a separate output directory");
        var storage = document.getDefaultStorageOptions().clone();
        storage.setSaveSimulationData(false);
        new GeneralRocketSaver().save(copy.toFile(), document, storage);
        var reopened = new GeneralRocketLoader(copy.toFile()).load();
        for (boolean useCandidate : new boolean[]{false, true}) {
            var sim = reopened.getSimulation(useCandidate ? reopened.getSimulationCount() - 1 : 0);
            if (sim.getOptions().isFiniteWingFins() != useCandidate) throw new AssertionError("Model selection lost on reload");
            sim.getOptions().setRandomSeed(20261108);
            sim.simulate(new AbstractSimulationListener() {
                @Override public boolean isSystemListener() { return true; }
                @Override public void startSimulation(SimulationStatus status) {
                    // The wind generator has its own seed independent of RK4 noise.
                    var conditions = status.getSimulationConditions();
                    var wind = conditions.getWindModel();
                    if (wind instanceof PinkNoiseWindModel average)
                        conditions.setWindModel(average.withSeed(20261108));
                    else if (wind instanceof MultiLevelPinkNoiseWindModel multi)
                        conditions.setWindModel(multi.withSeed(20261108));
                    else throw new IllegalArgumentException("Unsupported wind model for matched runs");
                }
            });
            if (sim.hasErrors()) throw new AssertionError("Simulation failed: " + sim.getStatus());
            var data = sim.getSimulatedData();
            var branch = data.getBranch(0);
            String model = useCandidate ? "candidate" : "legacy";
            System.out.println("FLIGHT " + model + " branches=" + data.getBranchCount() + " apogee_m=" + data.getMaxAltitude());
            for (var event : branch.getEvents()) {
                System.out.println("EVENT " + model + " " + event.getTime() + " " + event.getType().name() + " " + event.getData());
            }
            try (var out = new PrintWriter(Files.newBufferedWriter(output.resolve(model + "-sustainer.csv")))) {
                out.println("time_s,mach,cp_m,cg_m,margin_cal,diameter_m,aoa_rad,altitude_m");
                var types = new FlightDataType[]{TYPE_TIME, TYPE_MACH_NUMBER, TYPE_CP_LOCATION, TYPE_CG_LOCATION,
                        TYPE_STABILITY, TYPE_REFERENCE_LENGTH, TYPE_AOA, TYPE_ALTITUDE};
                for (int row = 0; row < branch.getLength(); row++) {
                    for (int column = 0; column < types.length; column++) {
                        if (column > 0) out.print(',');
                        out.print(branch.getByIndex(types[column], row));
                    }
                    out.println();
                }
                if (out.checkError()) throw new java.io.IOException("Could not write trajectory");
            }
        }
        System.exit(0);
    }
}
