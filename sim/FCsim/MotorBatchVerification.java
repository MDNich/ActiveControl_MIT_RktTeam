import com.google.inject.*;
import com.google.inject.util.Modules;
import info.openrocket.core.database.MotorDatabaseLoader;
import info.openrocket.core.database.motor.*;
import info.openrocket.core.document.*;
import info.openrocket.core.file.*;
import info.openrocket.core.file.motor.GeneralMotorLoader;
import info.openrocket.core.motor.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.motorbatch.*;
import info.openrocket.core.startup.Application;
import info.openrocket.core.plugin.PluginModule;
import info.openrocket.swing.startup.GuiModule;
import info.openrocket.swing.gui.dialogs.motor.thrustcurve.PreferredMotorCurve;
import java.nio.file.*;
import java.util.*;

/** Reproducible real-model clearance/catalogue check using the packaged batch implementation. */
public class MotorBatchVerification {
    /**
     * Runs the motor batch verification entry point, reporting verification failures to the
     * invoking process.
     *
     * @param args command-line arguments
     * @throws Exception if the requested operation cannot be completed
     * @throws IllegalArgumentException when rejected: Usage: ORK ENG OUTPUT_DIRECTORY
     */
    public static void main(String[] args) throws Exception {
        if (args.length != 3) throw new IllegalArgumentException("Usage: ORK ENG OUTPUT_DIRECTORY");
        System.setProperty("openrocket.bypass.presets", "true");
        System.setProperty("openrocket.bypass.motors", "true");
        var loader = new MotorDatabaseLoader();
        var module = new GuiModule();
        Application.setInjector(
                Guice.createInjector(
                        Modules.override(module)
                                .with(
                                        new AbstractModule() {
                                            /**
                                             * Configures application services for the standalone
                                             * verification run.
                                             */
                                            @Override
                                            protected void configure() {
                                                bind(ThrustCurveMotorSetDatabase.class)
                                                        .toProvider(loader::getDatabase)
                                                        .in(Scopes.SINGLETON);
                                                bind(MotorDatabase.class)
                                                        .toProvider(loader::getDatabase)
                                                        .in(Scopes.SINGLETON);
                                            }
                                        }),
                        new PluginModule()));
        module.startLoader();
        loader.startLoading();
        loader.blockUntilLoaded();
        var db = loader.getDatabase();
        try (var in = Files.newInputStream(Path.of(args[1]))) {
            for (var m :
                    new GeneralMotorLoader().load(in, Path.of(args[1]).getFileName().toString()))
                db.addMotor(m.build());
        }
        var file = new GeneralRocketLoader(Path.of(args[0]).toFile());
        var doc = file.load();
        var source = doc.getSimulation(0);
        var motorConfig = source.getActiveConfiguration().getAllMotors().iterator().next();
        var mount = motorConfig.getMount();
        var motor = (ThrustCurveMotor) motorConfig.getMotor();
        var output = Path.of(args[2]);
        Files.createDirectories(output);
        var preferred =
                db.getMotorSets().stream()
                        .filter(s -> s.getMotorCount() > 0)
                        .map(PreferredMotorCurve::select)
                        .toList();
        // A deliberately generous user length limit demonstrates that geometry cannot be bypassed.
        var settings =
                new MotorBatchSettings(
                        ((RocketComponent) mount).getID(),
                        4,
                        motor.getDiameter(),
                        3,
                        new HashSet<>(
                                preferred.stream()
                                        .map(m -> m.getManufacturer().getDisplayName())
                                        .toList()),
                        "",
                        "ALL",
                        "ALL",
                        false,
                        0,
                        1e9,
                        0,
                        1e9,
                        0,
                        1e6,
                        MotorBatchSettings.DelayPolicy.KEEP_BASELINE,
                        Map.of());
        var snapshot = MotorBatchSnapshot.capture(source, settings);
        var candidates = MotorCandidateSelector.select(preferred, settings, snapshot.baseline());
        System.out.println(
                "VERIFY model="
                        + source.getRocket().getName()
                        + " mount="
                        + ((RocketComponent) mount).getName()
                        + " baseline_motor="
                        + motor.getDesignation()
                        + " dimensions="
                        + motor.getDiameter()
                        + "x"
                        + motor.getLength());
        System.out.println(
                "VERIFY baseline_clearance="
                        + MotorClearance.conflicts(source.getActiveConfiguration(), mount, motor));
        var runner = new MotorBatchRunner(snapshot, settings, candidates);
        var result = runner.run(p -> {});
        MotorBatchCSV.write(
                output.resolve("zephyrus-motor-candidates.csv"),
                result,
                MotorBatchCSV.Scope.ALL_CANDIDATES,
                Set.of());
        Files.writeString(output.resolve("zephyrus-motor-batch.json"), MotorBatchIO.write(result));
        Files.writeString(
                output.resolve("summary.txt"),
                "Model: "
                        + source.getRocket().getName()
                        + "\nOriginal model unchanged.\nCandidates: "
                        + candidates.size()
                        + "\nTrials: "
                        + runner.totalTrials()
                        + "\nPeak workers: "
                        + result.peakThreads()
                        + "\nBaseline clearance: "
                        + MotorClearance.conflicts(source.getActiveConfiguration(), mount, motor)
                        + "\n");
        System.exit(0);
    }
}
