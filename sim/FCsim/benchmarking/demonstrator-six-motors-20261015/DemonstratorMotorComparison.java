import com.google.inject.*;
import com.google.inject.util.Modules;
import info.openrocket.core.database.motor.*;
import info.openrocket.core.document.*;
import info.openrocket.core.file.*;
import info.openrocket.core.motor.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.motorbatch.*;
import info.openrocket.core.simulation.ensemble.*;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.simulation.listeners.AbstractSimulationListener;
import info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener;
import jakarta.json.*;
import info.openrocket.core.startup.Application;
import info.openrocket.core.plugin.PluginModule;
import info.openrocket.swing.startup.GuiModule;
import info.openrocket.swing.gui.dialogs.motor.thrustcurve.PreferredMotorCurve;
import java.io.*;
import java.nio.file.*;
import java.security.MessageDigest;
import java.util.*;

/** Run the six requested catalogue motors on immutable copies of both demonstrators. */
public class DemonstratorMotorComparison {
    static final List<String> REQUESTED = List.of("J270W", "J415W", "J800T", "K550W", "K480W", "K700W");
    static String name(ThrustCurveMotor m) { return m.getDesignation().replaceFirst("^HP-", ""); }
    static String sha(Path p) throws Exception {
        return HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(Files.readAllBytes(p)));
    }
    static double mean(double[] samples) {
        return Arrays.stream(samples).average().orElseThrow();
    }
    static double sd(double[] samples) {
        double avg = mean(samples);
        return Math.sqrt(Arrays.stream(samples).map(v -> (v-avg)*(v-avg)).sum()/(samples.length-1));
    }
    static void runEnsemble(Simulation baseline, MotorBatchSettings batchSettings, ThrustCurveMotor motor,
            Path output, int iteration, boolean exportPlot) throws Exception {
        var trial = MotorBatchSnapshot.trial(baseline,batchSettings,motor,Motor.PLUGGED_DELAY,"mass-cd-ensemble");
        var ensembleSettings = new EnsembleSettings(true,20,0,.05,0,0,0,12345,.05,.05,0,0,0,4);
        trial.getOptions().setEnsembleSettings(ensembleSettings);
        var observedThreads = java.util.concurrent.ConcurrentHashMap.<String>newKeySet();
        var checkedRuns = new java.util.concurrent.atomic.AtomicInteger();
        var check = new AbstractSimulationListener() {
            @Override public boolean isSystemListener() { return true; }
            @Override public void startSimulation(SimulationStatus status) {
                if (FlightControllerSimulatorListener.active(status)!=null)
                    throw new IllegalStateException("Flight computer unexpectedly active");
                for (var c : status.getConfiguration().getActiveComponents())
                    if (c instanceof TabControlledTrapezoidFinSet f && f.getTabAngle()!=0)
                        throw new IllegalStateException("Roll tabs unexpectedly deflected");
                observedThreads.add(Thread.currentThread().getName());
                checkedRuns.incrementAndGet();
            }
        };
        System.out.println("ENSEMBLE_CASE_START iteration=" + iteration + " motor=" + name(motor));
        trial.simulate(check);
        var data = trial.getSimulatedData();
        var ensemble = data.getEnsembleResult();
        var altitude = ensemble.samples(0,EnsembleMetric.MAX_ALTITUDE);
        var time = ensemble.samples(0,EnsembleMetric.TIME_TO_APOGEE);
        if (altitude.length!=20 || time.length!=20 || checkedRuns.get()!=20 || observedThreads.size()!=4)
            throw new IllegalStateException("Incomplete ensemble or parallelism verification");
        var runs = Json.createArrayBuilder();
        var individualSeries = Json.createArrayBuilder();
        for (int i=0;i<20;i++) {
            if (!Double.isFinite(altitude[i]) || !Double.isFinite(time[i]))
                throw new IllegalStateException("Nonfinite ensemble metric");
            var inputs = ensemble.individualRuns().parameters(i);
            var flight = ensemble.individualRuns().read(i,trial.getDocument());
            var branch = flight.getBranch(0);
            if (branch.getFirstEvent(FlightEvent.Type.APOGEE)==null
                    || branch.getFirstEvent(FlightEvent.Type.GROUND_HIT)==null
                    || branch.getFirstEvent(FlightEvent.Type.SIM_ABORT)!=null)
                throw new IllegalStateException("Incomplete member " + (i+1));
            runs.add(Json.createObjectBuilder().add("run",i+1)
                .add("mass_multiplier",inputs.massMultiplier()).add("cd_multiplier",inputs.dragMultiplier())
                .add("apogee_m_agl",altitude[i]).add("time_to_apogee_s",time[i])
                .add("apogee_event_s",branch.getFirstEvent(FlightEvent.Type.APOGEE).getTime())
                .add("ground_hit",true));
            if (exportPlot) individualSeries.add(Json.createObjectBuilder().add("run",i+1)
                .add("time_s",array(branch.get(FlightDataType.TYPE_TIME)))
                .add("altitude_m",array(branch.get(FlightDataType.TYPE_ALTITUDE)))
                .add("speed_m_s",array(branch.get(FlightDataType.TYPE_VELOCITY_TOTAL)))
                .add("ground_hit_s",branch.getFirstEvent(FlightEvent.Type.GROUND_HIT).getTime()));
        }
        var settingsJson=Json.createObjectBuilder();
        ensembleSettings.attributes().forEach(settingsJson::add);
        var result=Json.createObjectBuilder().add("iteration",iteration).add("motor",name(motor))
            .add("curve_digest",motor.getDigest()).add("settings",settingsJson)
            .add("fc_enabled",false).add("roll_tab_angle_deg",0).add("verified_runs",checkedRuns.get())
            .add("observed_worker_threads",observedThreads.size()).add("standard_deviation","sample (N-1)")
            .add("mean_apogee_m_agl",mean(altitude)).add("stdev_apogee_m",sd(altitude))
            .add("mean_time_to_apogee_s",mean(time)).add("stdev_time_to_apogee_s",sd(time))
            .add("warnings",data.getWarningSet().toString()).add("runs",runs).build();
        Files.writeString(output.resolve(name(motor)+"-ensemble.json"),result.toString());
        if (exportPlot) {
            var average=data.getBranch(0);
            var deviation=ensemble.deviation(0);
            var series=Json.createObjectBuilder().add("iteration",iteration).add("motor",name(motor))
                .add("settings",settingsJson).add("fc_enabled",false).add("roll_tab_angle_deg",0)
                .add("description","OpenRocket ensemble mean and pointwise sample standard deviation; common physical-time interval")
                .add("time_s",array(average.get(FlightDataType.TYPE_TIME)))
                .add("mean_altitude_m",array(average.get(FlightDataType.TYPE_ALTITUDE)))
                .add("stdev_altitude_m",array(deviation.get(FlightDataType.TYPE_ALTITUDE)))
                .add("mean_speed_m_s",array(average.get(FlightDataType.TYPE_VELOCITY_TOTAL)))
                .add("stdev_speed_m_s",array(deviation.get(FlightDataType.TYPE_VELOCITY_TOTAL)))
                .add("individual_runs",individualSeries).build();
            Files.writeString(output.resolve(name(motor)+"-trajectory.json"),series.toString());
        }
        System.out.println("ENSEMBLE_CASE_END iteration=" + iteration + " motor=" + name(motor)
            + " mean_apogee_m=" + mean(altitude) + " sd_apogee_m=" + sd(altitude)
            + " mean_time_s=" + mean(time) + " sd_time_s=" + sd(time));
    }
    static JsonArray array(List<Double> values) {
        var a=Json.createArrayBuilder();
        for (double v : values) {
            if (!Double.isFinite(v)) throw new IllegalArgumentException("Nonfinite plotted sample");
            a.add(v);
        }
        return a.build();
    }
    public static void main(String[] args) throws Exception {
        boolean rangeCheck = args.length>2 && args[2].equals("neutral-tabs-range");
        boolean plotJ270 = args.length>2 && args[2].equals("neutral-tabs-j270w-plot");
        var requestedMotors = plotJ270 ? List.of("J270W") : rangeCheck
            ? List.of("J1299N","J460T","J450DM","J1265T","J1799N","J540R","J401FJ")
            : REQUESTED;
        System.setProperty("openrocket.bypass.presets", "true");
        System.setProperty("openrocket.bypass.motors", "true");
        var database = new ThrustCurveMotorSetDatabase();
        var module = new GuiModule();
        Application.setInjector(Guice.createInjector(Modules.override(module).with(new AbstractModule() {
            protected void configure() {
                bind(ThrustCurveMotorSetDatabase.class).toInstance(database);
                bind(MotorDatabase.class).toInstance(database);
            }
        }), new PluginModule()));
        module.startLoader();
        try (var in = new ObjectInputStream(DemonstratorMotorComparison.class.getResourceAsStream(
                "/datafiles/thrustcurves/thrustcurves.ser"))) {
            for (var m : (List<?>) in.readObject()) database.addMotor((ThrustCurveMotor) m);
        }
        var preferred = new LinkedHashMap<String, ThrustCurveMotor>();
        for (var set : database.getMotorSets()) if (set.getMotorCount() > 0) {
            var m = PreferredMotorCurve.select(set);
            if (m.getManufacturer().getDisplayName().equals("AeroTech") && requestedMotors.contains(name(m))) {
                if (preferred.put(name(m), m) != null) throw new IllegalStateException("Ambiguous motor " + name(m));
            }
        }
        if (!preferred.keySet().containsAll(requestedMotors)) throw new IllegalStateException("Missing motors: " + preferred.keySet());
        var modelDir = Path.of(args[0]);
        var outputDir = Path.of(args[1]);
        boolean neutralTabs = args.length>2 && args[2].startsWith("neutral-tabs-");
        boolean ensembles = plotJ270 || args.length>2 && args[2].equals("neutral-tabs-ensemble");
        for (int iteration : plotJ270 ? new int[]{3} : new int[]{3,4}) {
            var path = modelDir.resolve("20267.demonstrator.iteration" + iteration + ".ork");
            String hash = sha(path);
            var loader = new GeneralRocketLoader(path.toFile());
            var doc = loader.load();
            System.out.println("LOAD iteration=" + iteration + " warnings=" + loader.getWarnings());
            var selected = doc.getSimulation(0);
            if (!selected.getSimulationExtensions().isEmpty()) throw new IllegalStateException("Baseline has extensions");
            var motors = new LinkedHashMap<>(preferred);
            // Retain the exact saved J415W/J800T curve when that motor already exists in the model.
            for (var sim : doc.getSimulations()) for (var config : sim.getActiveConfiguration().getAllMotors()) {
                if (config.getMotor() instanceof ThrustCurveMotor m && requestedMotors.contains(name(m))
                    && m.getManufacturer().getDisplayName().equals("AeroTech")) motors.put(name(m),m);
            }
            var source = MotorBatchSnapshot.detached(selected);
            source.getSimulationExtensions().removeIf(ZephyrusFlightComputer::isFlightComputer);
            if (ZephyrusFlightComputer.read(source).isEnabled()) throw new IllegalStateException("Flight computer enabled");
            if (neutralTabs) for (var c : source.getRocket()) {
                if (c instanceof TabControlledTrapezoidFinSet f) {
                    if (iteration==4) f.setTabAngle(0);
                    if (f.getTabAngle()!=0) throw new IllegalStateException("Expected neutral tabs");
                }
            }
            source.getOptions().setRandomSeed(12345);
            if (source.getActiveConfiguration().getAllMotors().size() != 1) throw new IllegalStateException("Expected one motor");
            var mount = source.getActiveConfiguration().getAllMotors().iterator().next().getMount();
            for (var c : source.getActiveConfiguration().getActiveComponents()) {
                if (c instanceof RecoveryDevice r && r.getDeploymentConfigurations().get(source.getFlightConfigurationId()).getDeployEvent()
                    == DeploymentConfiguration.DeployEvent.EJECTION) throw new IllegalStateException("Motor charge controls recovery");
            }
            // Both models use apogee + 1 s recovery. Suppress unused motor-charge events only on the analysis copy.
            mount.getMotorConfig(source.getFlightConfigurationId()).setEjectionDelay(Motor.PLUGGED_DELAY);
            var settings = new MotorBatchSettings(((RocketComponent)mount).getID(),4,.054,.45,
                Set.of("AeroTech"),"","ALL","L2",false,0,1e6,0,1e6,0,1e6,
                MotorBatchSettings.DelayPolicy.KEEP_BASELINE,
                Map.of(MotorBatchMetrics.Metric.APOGEE,new MotorBatchSettings.Bounds(null,914.4)),true);
            var snapshot = MotorBatchSnapshot.capture(source,settings);
            if (snapshot.input().getBoolean("fcEnabled")) throw new IllegalStateException("Snapshot has FC enabled");
            var candidates = new ArrayList<MotorCandidateSelector.Candidate>();
            for (String requested : requestedMotors) {
                var m = motors.get(requested);
                double expectedDiameter = requested.equals("J270W") ? .038 : .054;
                Motor.Type expectedType = List.of("J270W","J450DM","J1265T").contains(requested)
                    ? Motor.Type.SINGLE : Motor.Type.RELOAD;
                if (Math.abs(m.getDiameter()-expectedDiameter) > .001 || m.getMotorType()!=expectedType)
                    throw new IllegalStateException("Unexpected motor variant " + requested);
                System.out.println("MOTOR iteration=" + iteration + " designation=" + requested + " length=" + m.getLength()
                    + " digest=" + m.getDigest() + " conflicts=" + MotorClearance.conflicts(source.getActiveConfiguration(),mount,m));
                // User requested all six simulations: retain and report every clearance warning rather than omit a trial.
                candidates.add(new MotorCandidateSelector.Candidate(requested,m,""));
            }
            var runner = new MotorBatchRunner(snapshot,settings,candidates);
            if (runner.totalTrials()!=requestedMotors.size()) throw new IllegalStateException("Not all motors scheduled");
            var result = runner.run(p -> {});
            var output = outputDir.resolve("iteration" + iteration);
            Files.createDirectories(output);
            Files.writeString(output.resolve("motor-batch-results.json"), MotorBatchIO.write(result));
            if (!hash.equals(sha(path))) throw new IllegalStateException("Source model changed during simulation");
            Files.writeString(output.resolve("provenance.txt"),"Source: " + path + "\nSHA256: " + hash
                + "\nBaseline: " + selected.getName() + "\nRandom seed: 12345\n"
                + "Saved launch conditions retained; FC disabled.\n"
                + (neutralTabs ? "Roll tabs neutral on both models.\n"
                    : "Saved geometry retained; ensembles disabled.\n")
                + (ensembles ? "Ensemble: 20 runs, mass sigma=5%, Cd sigma=5%, other sigmas=0, threads=4.\n" : "")
                + "Recovery remains apogee + 1 s; unused motor charge events suppressed.\n"
                + "Requested motors simulated with intersection override; warnings remain in results.\n"
                + "Source file unchanged.\n");
            System.out.println("COMPLETE iteration=" + iteration + " rows=" + result.rows().size());
            if (ensembles) for (String requested : requestedMotors)
                runEnsemble(source,settings,motors.get(requested),output,iteration,plotJ270);
            if (!hash.equals(sha(path))) throw new IllegalStateException("Source model changed during ensembles");
        }
        System.exit(0);
    }
}
