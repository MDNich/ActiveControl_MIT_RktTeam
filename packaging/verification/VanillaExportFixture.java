import info.openrocket.core.document.OpenRocketDocument;
import info.openrocket.core.file.GeneralRocketLoader;
import info.openrocket.core.file.openrocket.VanillaOpenRocketExporter;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.ensemble.EnsembleSettings;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.simulation.motorbatch.*;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.UUID;

/** MIT-side producer: first argument is the output directory, remaining arguments are .ork inputs. */
public final class VanillaExportFixture {
    public static void main(String[] args) {
        try {
            Vanilla24CompatibilityProbe.initialize();
            Path output = Path.of(args[0]);
            Files.createDirectories(output);
            var exporter = new VanillaOpenRocketExporter();
            for (int i = 1; i < args.length; i++) {
                File input = new File(args[i]);
                var loader = new GeneralRocketLoader(input);
                var document = loader.load();
                System.out.println("MIT_SOURCE_LOAD " + input + " warnings=" + loader.getWarnings());
                exporter.export(document, output.resolve(input.getName().replaceFirst("\\.ork$", "")
                        + "-vanilla-24.12.ork").toFile());
            }
            var resource = VanillaExportFixture.class.getResource("/datafiles/examples/A simple model rocket.ork");
            OpenRocketDocument document;
            try (var stream = resource.openStream()) {
                document = new GeneralRocketLoader(resource).load(stream, "Mixed features");
            }
            TrapezoidFinSet original = null;
            BodyTube body = null;
            for (var c : document.getRocket()) {
                if (c instanceof TrapezoidFinSet fins) original = fins;
                if (c instanceof BodyTube tube && body == null) body = tube;
            }
            if (original == null || body == null) throw new AssertionError("Example geometry changed");
            var fins = new TabControlledTrapezoidFinSet(original);
            fins.setCrossSection(FinSet.CrossSection.TRIANGULAR);
            fins.setLeadingEdgeAngle(Math.toRadians(20));
            fins.setTabAngle(Math.toRadians(45));
            var brakes = new AirbrakeSet();
            body.addChild(brakes);
            brakes.setIndivAirbrakeMass(.0001);
            for (var simulation : document.getSimulations()) {
                simulation.getSimulationExtensions().add(new ZephyrusFlightComputer());
                simulation.getOptions().setInitialRollRate(2);
                simulation.getOptions().setEnsembleSettings(new EnsembleSettings(true, 5, 5, .05, 2, .01, 1, 42));
            }
            document.putMotorBatch(new MotorBatchReport("fixture", "verification",
                    MotorBatchSettings.defaults(UUID.randomUUID(), .018, .07),
                    jakarta.json.Json.createObjectBuilder().build(), List.of(), 0));
            exporter.export(document, output.resolve("mixed-features-vanilla-24.12.ork").toFile());
            System.out.println("MIT_EXPORT_FIXTURES=PASS");
            System.exit(0);
        } catch (Throwable failure) {
            failure.printStackTrace();
            System.exit(1);
        }
    }
}
