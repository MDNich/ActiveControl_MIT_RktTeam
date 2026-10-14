import com.google.inject.*;
import com.google.inject.util.Modules;
import info.openrocket.core.database.motor.*;
import info.openrocket.core.document.*;
import info.openrocket.core.file.*;
import info.openrocket.core.motor.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.motorbatch.*;
import info.openrocket.core.startup.Application;
import info.openrocket.core.plugin.PluginModule;
import info.openrocket.swing.startup.GuiModule;
import info.openrocket.swing.gui.dialogs.motor.thrustcurve.PreferredMotorCurve;
import java.io.*;
import java.nio.file.*;
import java.util.*;

/** Independent shortlist analysis; never saves or edits the input rocket. */
public class DemonstratorL2Check {
    public static void main(String[] args) throws Exception {
        System.setProperty("openrocket.bypass.presets", "true");
        System.setProperty("openrocket.bypass.motors", "true");
        var database = new ThrustCurveMotorSetDatabase();
        var module = new GuiModule();
        Application.setInjector(Guice.createInjector(Modules.override(module).with(
            new AbstractModule() {
                protected void configure() {
                    bind(ThrustCurveMotorSetDatabase.class).toInstance(database);
                    bind(MotorDatabase.class).toInstance(database);
                }
            }), new PluginModule()));
        module.startLoader();
        try (var in = new ObjectInputStream(DemonstratorL2Check.class.getResourceAsStream(
                "/datafiles/thrustcurves/thrustcurves.ser"))) {
            for (var m : (List<?>) in.readObject()) database.addMotor((ThrustCurveMotor) m);
        }
        var loader = new GeneralRocketLoader(Path.of(args[0]).toFile());
        var doc = loader.load();
        System.out.println("LOAD_WARNINGS " + loader.getWarnings());
        Simulation selected = null;
        var preferred = new LinkedHashMap<String,ThrustCurveMotor>();
        for (var set : database.getMotorSets()) if (set.getMotorCount() > 0) {
            var m = PreferredMotorCurve.select(set);
            preferred.put(m.getManufacturer().getDisplayName() + ":" + m.getDesignation(), m);
        }
        for (var sim : doc.getSimulations()) {
            for (var config : sim.getActiveConfiguration().getAllMotors()) {
                if (config.getMotor() instanceof ThrustCurveMotor m) {
                    preferred.put(m.getManufacturer().getDisplayName() + ":" + m.getDesignation(), m);
                    if (m.getDesignation().contains("J1265")) selected = sim;
                }
            }
        }
        if (selected == null) throw new IllegalStateException("J1265 baseline missing");
        var source = MotorBatchSnapshot.detached(selected);
        var mount = source.getActiveConfiguration().getAllMotors().iterator().next().getMount();
        System.out.println("BASELINE " + selected.getName() + " " + source.getRocket().getName());
        System.out.println("MOUNT " + ((RocketComponent)mount).getName() + " bore=" + mount.getMotorMountDiameter());
        // The model deploys its recovery at apogee + 1 s. Disable unused motor-charge events
        // on this analysis copy, so arbitrary listed charge delays do not filter the catalogue.
        mount.getMotorConfig(source.getFlightConfigurationId()).setEjectionDelay(Motor.PLUGGED_DELAY);
        var settings = new MotorBatchSettings(((RocketComponent)mount).getID(), 4, .054, .45,
            new HashSet<>(preferred.values().stream().map(m -> m.getManufacturer().getDisplayName()).toList()),
            "", "ALL", "L2", true, 640.01, 1400, 0, 1e6, 0, 1e6,
            MotorBatchSettings.DelayPolicy.KEEP_BASELINE,
            Map.of(MotorBatchMetrics.Metric.APOGEE, new MotorBatchSettings.Bounds(null, 914.4)))
            .withIgnoreComponentIntersections(true);
        var snapshot = MotorBatchSnapshot.capture(source, settings);
        var candidates = new ArrayList<MotorCandidateSelector.Candidate>();
        System.out.println("MANUFACTURERS " + settings.manufacturers());
        for (var m : preferred.values()) {
            String maker = m.getManufacturer().getDisplayName();
            if (!(maker.equals("AeroTech") || maker.contains("Cesaroni"))) continue;
            if (args.length > 2 && !maker.contains(args[2])) continue;
            if (m.getDiameter() > .0544 || m.getDiameter() < .037) continue;
            if (!MotorCertification.classify(m).level().name().equals("L2")) continue;
            if (!m.isAvailable() || m.getTotalImpulseEstimate() > 1400) continue;
            var conflict = MotorClearance.conflicts(source.getActiveConfiguration(), mount, m);
            System.out.printf(Locale.ROOT, "CATALOG %s %s %.0fmm %.0fmm %s %.1fNs clearance=%s%n",
                maker,m.getDesignation(),m.getDiameter()*1000,m.getLength()*1000,
                m.getMotorType(),m.getTotalImpulseEstimate(),conflict);
            // User confirmed on 2026-10-14: assume actual RCR/RRC hardware clears 54 mm motors.
            // Preserve all other modeled obstacles, including the pressure bulkhead and avionics.
            if (conflict.stream().anyMatch(c -> !(c.startsWith("MOTOR_INTERSECTS:RCR [")
                    || c.startsWith("MOTOR_INTERSECTS:RRC [")))) continue;
            candidates.add(new MotorCandidateSelector.Candidate("motor-" + candidates.size(),m,""));
        }
        var out = Path.of(args[1]);
        Files.createDirectories(out);
        var runner = new MotorBatchRunner(snapshot,settings,candidates);
        System.out.println("TRIALS " + runner.totalTrials());
        var result = runner.run(p -> {});
        Files.writeString(out.resolve("motor-batch-results.json"), MotorBatchIO.write(result));
        MotorBatchCSV.write(out.resolve("motor-batch-results.csv"), result, MotorBatchCSV.Scope.ALL_CANDIDATES,Set.of());
        System.out.println("COMPLETE rows=" + result.rows().size() + " workers=" + result.peakThreads());
        System.exit(0);
    }
}
