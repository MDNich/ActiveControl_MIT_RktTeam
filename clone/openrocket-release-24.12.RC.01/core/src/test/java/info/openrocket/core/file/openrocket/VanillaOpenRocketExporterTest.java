package info.openrocket.core.file.openrocket;

import static org.junit.jupiter.api.Assertions.*;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InterruptedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.UUID;
import java.util.zip.ZipFile;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import info.openrocket.core.document.*;
import info.openrocket.core.file.GeneralRocketLoader;
import info.openrocket.core.logging.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.rocketcomponent.position.AxialMethod;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.customexpression.CustomExpression;
import info.openrocket.core.simulation.ensemble.EnsembleSettings;
import info.openrocket.core.simulation.extension.impl.JavaCode;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.simulation.motorbatch.*;
import info.openrocket.core.util.BaseTestCase;

class VanillaOpenRocketExporterTest extends BaseTestCase {
    @TempDir Path directory;

    private OpenRocketDocument fixture() {
        var doc = OpenRocketDocumentFactory.createEmptyRocket();
        doc.getRocket().addChild(new AxialStage());
        var body = new BodyTube(.6, .05);
        doc.getRocket().getChild(0).addChild(body);
        var fins = new TabControlledTrapezoidFinSet();
        body.addChild(fins);
        fins.setName("Roll fins & leading edge");
        fins.setRootChord(.14);
        fins.setCrossSection(FinSet.CrossSection.TRIANGULAR);
        fins.setLeadingEdgeAngle(.3);
        fins.setTabAngle(.8);
        fins.setTabHeight(.012);
        fins.setTabLength(.022);
        fins.setTabOffsetMethod(AxialMethod.TOP);
        fins.setMountingTabOffset(.009);
        fins.setCantAngle(Math.toRadians(-40));
        fins.setFilletRadius(.001);
        var brakes = new AirbrakeSet();
        body.addChild(brakes);
        brakes.setAxialMethod(AxialMethod.BOTTOM);
        brakes.setAxialOffset(-.12);
        brakes.setCGOverridden(true);
        brakes.setOverrideCGX(.004);
        var chute = new Parachute();
        body.addChild(chute);
        chute.getDeploymentConfigurations().getDefault().setDeployEvent(DeploymentConfiguration.DeployEvent.APOGEE);
        var options = new SimulationOptions();
        options.setTimeStep(.05);
        options.setInitialRollRate(7);
        options.setLaunchIntoWind(false);
        options.setLaunchRodDirection(.72);
        options.setEnsembleSettings(new EnsembleSettings(true, 5, 10, .05, 2, .01, 1, 42));
        var java = new JavaCode();
        java.setClassName("info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener");
        var sim = new Simulation(doc, doc.getRocket(), Simulation.Status.EXTERNAL, "Flight & controller",
                options, List.of(new ZephyrusFlightComputer(), java), new FlightData());
        doc.addSimulation(sim);
        doc.putMotorBatch(new MotorBatchReport("test-batch", "now", MotorBatchSettings.defaults(UUID.randomUUID(), .038, .3),
                jakarta.json.Json.createObjectBuilder().build(), List.of(), 0));
        doc.setFile(directory.resolve("original.ork").toFile());
        doc.getDefaultStorageOptions().setSaveSimulationData(true);
        doc.setSaved(true);
        return doc;
    }

    private static String ordinaryXml(OpenRocketDocument doc) throws IOException {
        var out = new ByteArrayOutputStream();
        new OpenRocketSaver().save(out, doc, doc.getDefaultStorageOptions(), new WarningSet(), new ErrorSet());
        return out.toString(StandardCharsets.UTF_8);
    }

    private Path export(OpenRocketDocument doc) throws IOException {
        Path target = directory.resolve("vanilla.ork");
        new VanillaOpenRocketExporter().export(doc, target.toFile());
        return target;
    }

    private String xml(Path file) throws IOException {
        try (var zip = new ZipFile(file.toFile())) {
            return new String(zip.getInputStream(zip.getEntry("rocket.ork")).readAllBytes(), StandardCharsets.UTF_8);
        }
    }

    @Test
    void exportsCompatibleBaselinesWithoutMutatingOriginal() throws Exception {
        var doc = fixture();
        String before = ordinaryXml(doc);
        var file = doc.getFile();
        var modID = doc.getRocket().getModID();
        var flightData = doc.getSimulation(0).getSimulatedData();
        boolean saved = doc.isSaved();
        String exported = xml(export(doc));
        for (String forbidden : List.of("tabctrltrapezoidfinset", "leadingedgeangle", "triangular", "airbrakeset",
                "<extension", "<ensemble", "<motorbatch", "<initialrollrate", "<flightdata", "tabspan", "tabangle"))
            assertFalse(exported.contains(forbidden), forbidden);
        assertTrue(exported.contains("<crosssection>rounded</crosssection>"));
        assertTrue(exported.contains("<simulation status=\"notsimulated\">"));
        assertTrue(exported.contains("<launchintowind>false</launchintowind>"));
        assertEquals(before, ordinaryXml(doc));
        assertSame(file, doc.getFile());
        assertEquals(modID, doc.getRocket().getModID());
        assertSame(flightData, doc.getSimulation(0).getSimulatedData());
        assertEquals(saved, doc.isSaved());
        assertTrue(doc.getDefaultStorageOptions().getSaveSimulationData());
        assertTrue(before.contains("<extension"));
        assertTrue(before.contains("<airbrakeset>"));
    }

    @Test
    void preservesFinGeometryMountingTabsBallastAndRecovery() throws Exception {
        var doc = fixture();
        var loader = new GeneralRocketLoader(export(doc).toFile());
        var loaded = loader.load();
        assertTrue(loader.getWarnings().isEmpty(), loader.getWarnings().toString());
        for (var source : doc.getRocket()) {
            var target = loaded.getRocket().findComponent(source.getID());
            assertNotNull(target);
            assertEquals(source.getName(), target.getName());
            if (source instanceof FinSet fins) {
                assertEquals(TrapezoidFinSet.class, target.getClass());
                var converted = (TrapezoidFinSet) target;
                assertEquals(.14, converted.getRootChord());
                assertEquals(fins.getFinCount(), converted.getFinCount());
                assertEquals(fins.getThickness(), converted.getThickness());
                assertEquals(fins.getTabOffset(fins.getTabOffsetMethod()), converted.getTabOffset());
                assertEquals(fins.getTabHeight(), converted.getTabHeight());
                assertEquals(fins.getFilletRadius(), converted.getFilletRadius());
                assertEquals(-Math.toRadians(15), converted.getCantAngle());
            }
            if (source instanceof AirbrakeSet) {
                assertInstanceOf(MassComponent.class, target);
                assertEquals(source.getMass(), target.getMass());
                assertEquals(source.getPosition().x, target.getPosition().x, 1e-12);
                assertEquals(source.getCG().x, target.getCG().x, 1e-12);
            }
            if (target instanceof Parachute chute)
                assertEquals(DeploymentConfiguration.DeployEvent.APOGEE,
                        chute.getDeploymentConfigurations().getDefault().getDeployEvent());
        }
        var sim = loaded.getSimulation(0);
        assertEquals(doc.getSimulation(0).getId(), sim.getId());
        // This geometry-only fixture has no motor: the UI derives CANT_RUN from that.
        assertEquals(Simulation.Status.CANT_RUN, sim.getStatus());
        assertNull(sim.getSimulatedData());
        assertTrue(sim.getSimulationExtensions().isEmpty());
        assertEquals(.72, sim.getOptions().getLaunchRodDirection(), 1e-12);
        assertFalse(sim.getOptions().getEnsembleSettings().enabled());
        assertEquals(0, sim.getOptions().getInitialRollRate());
    }

    @Test
    void dropsOnlyUnsupportedExpressionsAndTheirDependents() throws Exception {
        var doc = fixture();
        for (var entry : new String[][] { {"good", "t * 2"}, {"attitude", "Qw * 2"},
                {"control", "FC_state"}, {"dependent", "attitude + 1"}, {"good2", "good + 3"} })
            doc.addCustomExpression(new CustomExpression(doc, entry[0], entry[0], "", entry[1]));
        assertEquals(List.of("good", "good2"), VanillaOpenRocketExporter.compatibleExpressions(doc)
                .stream().map(CustomExpression::getSymbol).toList());
        String exported = xml(export(doc));
        assertTrue(exported.contains("<symbol>good</symbol>"));
        assertFalse(exported.contains("<symbol>dependent</symbol>"));
        assertEquals(5, doc.getCustomExpressions().size());
    }

    @Test
    void launchIntoWindExportDoesNotChangeLiveOptionsOrFireEvents() throws Exception {
        var doc = fixture();
        var options = doc.getSimulation(0).getOptions();
        options.setLaunchRodDirection(.1);
        options.getAverageWindModel().setDirection(1.2);
        options.setLaunchIntoWind(true);
        var before = options.clone();
        var events = new java.util.concurrent.atomic.AtomicInteger();
        options.addChangeListener(e -> events.incrementAndGet());
        export(doc);
        assertEquals(before, options);
        assertEquals(0, events.get());
    }

    @Test
    void refusesOriginalFileAndAliases() throws Exception {
        var doc = fixture();
        Path original = doc.getFile().toPath();
        Files.writeString(original, "original bytes");
        Path alias = directory.resolve("alias.ork");
        Files.createSymbolicLink(alias, original);
        Path hardlink = directory.resolve("hardlink.ork");
        Files.createLink(hardlink, original);
        for (Path path : List.of(original, alias, hardlink))
            assertThrows(IOException.class, () -> new VanillaOpenRocketExporter().export(doc, path.toFile()));
        assertEquals("original bytes", Files.readString(original));
    }

    @Test
    void cancellationKeepsExistingDestinationAndCleansTemporaryFile() throws Exception {
        var doc = fixture();
        Path target = directory.resolve("vanilla.ork");
        Files.writeString(target, "keep me");
        Thread.currentThread().interrupt();
        try {
            assertThrows(InterruptedIOException.class, () -> export(doc));
        } finally {
            Thread.interrupted();
        }
        assertEquals("keep me", Files.readString(target));
        try (var files = Files.list(directory)) {
            assertEquals(List.of(target), files.toList());
        }
    }

    @Test
    void preservesOutsideAndInsideTexturesAndFailsAtomicallyWhenMissing() throws Exception {
        var doc = fixture();
        var body = (BodyTube) doc.getRocket().getChild(0).getChild(0);
        var png = java.util.Base64.getDecoder().decode(
                "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+a1N8AAAAASUVORK5CYII=");
        Path imageFile = directory.resolve("texture.png");
        Files.write(imageFile, png);
        var image = doc.getDecalImage(new info.openrocket.core.document.attachments.FileSystemAttachment(
                "decals/test.png", imageFile.toFile()));
        var decal = new info.openrocket.core.appearance.Decal(
                info.openrocket.core.util.Coordinate.ZERO, info.openrocket.core.util.Coordinate.ZERO,
                new info.openrocket.core.util.Coordinate(1, 1, 0), 0, image,
                info.openrocket.core.appearance.Decal.EdgeMode.REPEAT);
        var appearance = new info.openrocket.core.appearance.Appearance(
                new info.openrocket.core.util.ORColor(255, 0, 0), .5, decal);
        body.setAppearance(appearance);
        body.getInsideColorComponentHandler().setInsideAppearance(appearance);
        Path target = export(doc);
        byte[] originalExport = Files.readAllBytes(target);
        try (var zip = new ZipFile(target.toFile())) {
            assertArrayEquals(png, zip.getInputStream(zip.getEntry(image.getName())).readAllBytes());
            assertEquals(2, zip.size());
        }
        assertTrue(xml(target).contains("<insideappearance>"));
        Files.delete(imageFile);
        assertThrows(IOException.class, () -> export(doc));
        assertArrayEquals(originalExport, Files.readAllBytes(target));
        assertSame(appearance, body.getAppearance());
        try (var files = Files.list(directory)) {
            assertTrue(files.noneMatch(p -> p.getFileName().toString().startsWith(".openrocket-vanilla-")));
        }
    }
}
