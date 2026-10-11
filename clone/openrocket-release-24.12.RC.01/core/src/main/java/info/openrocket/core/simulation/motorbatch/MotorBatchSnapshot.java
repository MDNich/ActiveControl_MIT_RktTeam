package info.openrocket.core.simulation.motorbatch;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.*;
import jakarta.json.*;
import info.openrocket.core.document.*;
import info.openrocket.core.file.DocumentLoadingContext;
import info.openrocket.core.file.openrocket.OpenRocketSaver;
import info.openrocket.core.file.openrocket.importt.OpenRocketLoader;
import info.openrocket.core.logging.*;
import info.openrocket.core.motor.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.simulation.flightcomputer.FlightComputerDesign;
import info.openrocket.core.util.BuildProperties;

/** The source is copied only by the coordinating thread; each worker receives its own copy. */
public final class MotorBatchSnapshot {
    private final Simulation baseline;
    private final JsonObject input;

    /**
     * Creates a MotorBatchSnapshot with the supplied dependencies and per-instance state.
     *
     * @param baseline initial baseline
     * @param input input snapshot for this operation
     */
    private MotorBatchSnapshot(Simulation baseline, JsonObject input) {
        this.baseline = baseline;
        this.input = input;
    }

    /**
     * Returns the captured immutable batch input.
     *
     * @return the captured immutable batch input
     */
    public JsonObject input() {
        return input;
    }

    /**
     * Returns the stored baseline simulation used to create isolated trials.
     *
     * @return the stored baseline simulation used to create isolated trials
     */
    public Simulation baseline() {
        return baseline;
    }

    /**
     * Creates a detached copy of the baseline rocket document.
     *
     * @param source source text, model or measured-channel selection
     * @return the resulting Simulation
     */
    public static Simulation detached(Simulation source) {
        var rocket = (Rocket) source.getRocket().copyWithOriginalID();
        var doc = OpenRocketDocumentFactory.createDocumentFromRocket(rocket);
        var sim = source.duplicateSimulation(doc, rocket);
        doc.addSimulation(sim);
        return sim;
    }

    /**
     * Creates a worker-local baseline so parallel trials do not share mutable rocket state.
     *
     * @return the resulting Simulation
     */
    public Simulation newWorkerBaseline() {
        return detached(baseline);
    }

    /**
     * Freezes the selected simulation, rocket and external FC design for the batch.
     *
     * @param source source text, model or measured-channel selection
     * @param settings validated configuration for this operation
     * @return the resulting MotorBatchSnapshot
     * @throws Exception if the requested operation cannot be completed
     * @throws IllegalArgumentException when rejected: Choose an editable baseline simulation
     */
    public static MotorBatchSnapshot capture(Simulation source, MotorBatchSettings settings)
            throws Exception {
        if (source.getStatus() == Simulation.Status.EXTERNAL)
            throw new IllegalArgumentException("Choose an editable baseline simulation");
        if (source.getActiveConfiguration().getActiveStageCount() != 1)
            throw new IllegalArgumentException(
                    "Motor batches currently require one active flight stage");
        var sim = detached(source);
        for (var extension : sim.getSimulationExtensions()) {
            if (settings.threads() > 1 && !ZephyrusFlightComputer.isFlightComputer(extension))
                throw new IllegalArgumentException(
                        "Parallel motor batches cannot isolate extension '"
                                + extension.getName()
                                + "'. Choose 1 thread or remove this extension from the baseline.");
            if (extension instanceof ZephyrusFlightComputer fc) fc.prepareDesign();
        }
        target(sim, settings.mountId());
        String xml = serialize(sim);
        var motors = Json.createArrayBuilder();
        var digests = new HashSet<String>();
        for (var component : sim.getRocket())
            if (component instanceof MotorMount mount)
                for (var it = mount.getMotorIterator(); it.hasNext(); ) {
                    var motor = it.next().getMotor();
                    if (motor instanceof ThrustCurveMotor m && digests.add(m.getDigest()))
                        motors.add(MotorBatchIO.motor(m));
                }
        var fc = ZephyrusFlightComputer.read(sim);
        String fingerprint =
                fc.getPreparedDesign() == null ? "" : fc.getPreparedDesign().design().fingerprint();
        var metadata =
                Json.createObjectBuilder()
                        .add("baseline", source.getName())
                        .add("configuration", source.getFlightConfigurationId().toString())
                        .add("version", BuildProperties.getVersion())
                        .add("seed", source.getOptions().getRandomSeed())
                        .add("fcEnabled", fc.isEnabled())
                        .add("fcReference", fc.getDesignReference())
                        .add("fcFingerprint", fingerprint)
                        .add("fcSeed", fc.getLinkSettings().randomSeed())
                        .add("fcTimingSeed", fc.getTimingSettings().randomSeed())
                        .add(
                                "targetName",
                                ((RocketComponent) target(sim, settings.mountId())).getName())
                        .add(
                                "motorCount",
                                target(sim, settings.mountId())
                                        .getMotorCountIncludingAssemblyCopies())
                        .add("xml", xml)
                        .add("motors", motors);
        String hash =
                FlightComputerDesign.hash((xml + fingerprint).getBytes(StandardCharsets.UTF_8));
        metadata.add("hash", hash);
        return new MotorBatchSnapshot(sim, metadata.build());
    }

    /**
     * Serializes the baseline document for independent reconstruction.
     *
     * @param source source text, model or measured-channel selection
     * @return formatted or resolved text
     * @throws IOException if the requested operation cannot be completed
     */
    private static String serialize(Simulation source) throws IOException {
        var doc =
                OpenRocketDocumentFactory.createDocumentFromRocket(
                        (Rocket) source.getRocket().copyWithOriginalID());
        doc.addSimulation(source.duplicateSimulation(doc, doc.getRocket()));
        var storage = new StorageOptions();
        storage.setSaveSimulationData(false);
        var bytes = new ByteArrayOutputStream();
        new OpenRocketSaver().save(bytes, doc, storage, new WarningSet(), new ErrorSet());
        return bytes.toString(StandardCharsets.UTF_8);
    }

    /**
     * Finds the configured motor mount in a copied rocket by stable identity.
     *
     * @param s component value
     * @param id stable identifier
     * @return the resulting MotorMount
     * @throws IllegalArgumentException when rejected: Choose a motor mount in the active flight
     *     configuration
     */
    public static MotorMount target(Simulation s, UUID id) {
        var component = s.getRocket().findComponent(id);
        if (!(component instanceof MotorMount mount)
                || !mount.isMotorMount()
                || !s.getActiveConfiguration().isComponentActive(component))
            throw new IllegalArgumentException(
                    "Choose a motor mount in the active flight configuration");
        return mount;
    }

    /**
     * Creates a trial simulation with the candidate motor and ejection delay installed.
     *
     * @param workerBaseline worker baseline
     * @param settings validated configuration for this operation
     * @param motor motor
     * @param delay delay
     * @param tag tag
     * @return the resulting Simulation
     */
    public static Simulation trial(
            Simulation workerBaseline,
            MotorBatchSettings settings,
            ThrustCurveMotor motor,
            double delay,
            String tag) {
        var s = detached(workerBaseline);
        var mount = target(s, settings.mountId());
        var id = s.getFlightConfigurationId();
        var config = new MotorConfiguration(mount, id, mount.getMotorConfig(id));
        config.setMotor(motor);
        config.setEjectionDelay(delay);
        mount.setMotorConfig(config, id);
        s.getOptions().setEnsembleSettings(s.getOptions().getEnsembleSettings().disabled());
        s.setRunOutputTag(tag);
        return s;
    }

    /** Create ordinary, independently configured reruns from the recorded input document. */
    public static OpenRocketDocument createSimulations(
            MotorBatchReport report, List<MotorBatchReport.Row> rows) throws Exception {
        if (rows.isEmpty() || rows.stream().anyMatch(r -> Double.isNaN(r.selected().delay())))
            throw new IllegalArgumentException(
                    "Select completed motor/delay results to create simulations");
        var target = restore(report);
        var base = target.getSimulation(0);
        target.removeSimulation(base);
        for (var row : rows) {
            var simulation = base.duplicateSimulation(target, target.getRocket());
            var configId = new FlightConfigurationId();
            var copy = base.getActiveConfiguration().copy(configId);
            copy.copyStageActiveness(base.getActiveConfiguration());
            target.getRocket().setFlightConfiguration(configId, copy);
            for (var component : target.getRocket())
                if (component instanceof FlightConfigurableComponent f)
                    f.copyFlightConfiguration(base.getFlightConfigurationId(), configId);
            simulation.setFlightConfigurationId(configId);
            var mount = target(simulation, report.settings().mountId());
            var config = new MotorConfiguration(mount, configId, mount.getMotorConfig(configId));
            config.setMotor(row.candidate().motor());
            config.setEjectionDelay(row.selected().delay());
            mount.setMotorConfig(config, configId);
            simulation
                    .getOptions()
                    .setEnsembleSettings(simulation.getOptions().getEnsembleSettings().disabled());
            simulation.setRunOutputTag(
                    "batch-"
                            + report.id()
                            + "-rerun-"
                            + row.candidate().id()
                            + "-"
                            + UUID.randomUUID());
            simulation.setName(row.candidate().motor().getDesignation() + " — batch rerun");
            target.addSimulation(simulation);
            System.out.println(
                    "MOTOR_BATCH create_simulation batch="
                            + report.id()
                            + " candidate="
                            + row.candidate().id()
                            + " delay="
                            + row.selected().delay());
        }
        return target;
    }

    /**
     * Saved summaries can always be viewed. Rerunning requires the original external FC revision.
     */
    public static OpenRocketDocument restore(MotorBatchReport report) throws Exception {
        var doc = OpenRocketDocumentFactory.createEmptyRocket();
        var ctx = new DocumentLoadingContext();
        ctx.setOpenRocketDocument(doc);
        var motors =
                report.input().getJsonArray("motors").getValuesAs(JsonObject.class).stream()
                        .map(MotorBatchIO::motor)
                        .toList();
        ctx.setMotorFinder(
                (type, mfr, name, diameter, length, digest, warnings) ->
                        motors.stream()
                                .filter(m -> Objects.equals(digest, m.getDigest()))
                                .findFirst()
                                .orElse(null));
        String xml = report.input().getString("xml");
        if (xml.length() > 32 * 1024 * 1024 || xml.contains("<motorbatch"))
            throw new IllegalArgumentException("Invalid batch input snapshot");
        new OpenRocketLoader()
                .loadFromStream(
                        ctx,
                        new ByteArrayInputStream(xml.getBytes(StandardCharsets.UTF_8)),
                        "motor-batch-snapshot.ork");
        if (doc.getSimulationCount() != 1)
            throw new IllegalArgumentException("Invalid batch baseline");
        var fc = ZephyrusFlightComputer.read(doc.getSimulation(0));
        if (fc.isEnabled() && fc.hasDesignFile()) {
            fc.prepareDesign();
            if (!fc.getPreparedDesign()
                    .design()
                    .fingerprint()
                    .equals(report.input().getString("fcFingerprint")))
                throw new IllegalArgumentException(
                        "This batch requires the original external FC file revision. Saved summaries remain available.");
        }
        return doc;
    }
}
