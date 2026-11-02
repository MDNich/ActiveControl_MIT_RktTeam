import com.google.inject.AbstractModule;
import com.google.inject.Guice;
import com.google.inject.util.Modules;
import info.openrocket.core.database.motor.MotorDatabase;
import info.openrocket.core.database.motor.ThrustCurveMotorSetDatabase;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.file.GeneralRocketLoader;
import info.openrocket.core.file.GeneralRocketSaver;
import info.openrocket.core.motor.ThrustCurveMotor;
import info.openrocket.core.plugin.PluginModule;
import info.openrocket.core.rocketcomponent.AirbrakeSet;
import info.openrocket.core.rocketcomponent.BodyTube;
import info.openrocket.core.simulation.FlightEvent;
import info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener;
import info.openrocket.core.startup.Application;
import info.openrocket.swing.gui.plot.Trajectory3DPanel;
import info.openrocket.swing.gui.plot.TrajectoryData;
import info.openrocket.swing.startup.GuiModule;
import java.awt.image.BufferedImage;
import java.io.ObjectInputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.SwingUtilities;
import javax.tools.ToolProvider;

/** Exercises simulation, FC output and the actual 3D panel in the packaged Linux runtime. */
public final class LinuxPackageProbe {
    private static void initialize() throws Exception {
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
        try (var stream = new ObjectInputStream(LinuxPackageProbe.class
                .getResourceAsStream("/datafiles/thrustcurves/thrustcurves.ser"))) {
            for (Object motor : (List<?>) stream.readObject()) database.addMotor((ThrustCurveMotor) motor);
        }
    }

    public static void main(String[] args) {
        try {
            if (!System.getProperty("os.name").equals("Linux")
                    || !System.getProperty("os.arch").equals("amd64"))
                throw new AssertionError("Expected Linux amd64 runtime");
            Path evidence = Path.of(args[1]);
            Files.createDirectories(evidence);
            initialize();
            var loader = new GeneralRocketLoader(Path.of(args[0]).toFile());
            var document = loader.load();
            if (!loader.getWarnings().isEmpty()) throw new AssertionError(loader.getWarnings());
            var simulation = document.getSimulation(0);
            simulation.getOptions().setMaxSimulationTime(90);
            simulation.simulate();
            var data = simulation.getSimulatedData();
            var branch = data.getBranch(0);
            if (simulation.hasErrors() || !Double.isFinite(data.getMaxAltitude())
                    || data.getMaxAltitude() <= 0
                    || branch.getFirstEvent(FlightEvent.Type.APOGEE) == null
                    || branch.getFirstEvent(FlightEvent.Type.GROUND_HIT) == null)
                throw new AssertionError("Nominal flight did not complete: " + simulation.getStatus());
            System.out.println("LINUX_FLIGHT=PASS apogee_m=" + data.getMaxAltitude()
                    + " flight_s=" + data.getFlightTime() + " samples=" + branch.getLength());
            Path saved = evidence.resolve("simulated-example.ork");
            new GeneralRocketSaver().save(saved.toFile(), document);
            var reopened = new GeneralRocketLoader(saved.toFile()).load();
            if (!document.getRocket().getIds().equals(reopened.getRocket().getIds())
                    || document.getSimulationCount() != reopened.getSimulationCount())
                throw new AssertionError("Save/reopen changed model/configurations");
            System.out.println("LINUX_SAVE_REOPEN=PASS");

            var compiler = ToolProvider.getSystemJavaCompiler();
            if (compiler == null) throw new AssertionError("Bundled runtime lacks Java compiler for FC programs");
            Path compilerSource = evidence.resolve("CompilerSmoke.java");
            Files.writeString(compilerSource, "public class CompilerSmoke { public static int value() { return 17; } }\n");
            if (compiler.run(null, null, null, "-encoding", "UTF-8", compilerSource.toString()) != 0)
                throw new AssertionError("Bundled runtime compilation failed");
            System.out.println("LINUX_BUNDLED_COMPILER=PASS");

            Path telemetry = evidence.resolve("fc");
            Files.createDirectories(telemetry);
            System.setProperty("openrocket.fc.telemetryDir", telemetry.toString());
            // Keep the nominal example unchanged. FC binding requires physical airbrakes;
            // attach a small synthetic set to the separately reopened verification model.
            BodyTube fcBody = null;
            for (var component : reopened.getRocket()) {
                if (component instanceof BodyTube body) { fcBody = body; break; }
            }
            if (fcBody == null) throw new AssertionError("Example has no body for the FC fixture");
            var brakes = new AirbrakeSet(0.01, 0.005, 0.001, 0.008, 0.0001, 4);
            brakes.setName("Synthetic packaging verification airbrakes");
            fcBody.addChild(brakes);
            brakes.setFracExposed(0);
            var fcSimulation = new Simulation(reopened, reopened.getRocket());
            fcSimulation.setFlightConfigurationId(simulation.getFlightConfigurationId());
            fcSimulation.getOptions().setMaxSimulationTime(3);
            var listener = new FlightControllerSimulatorListener(s -> {}, 0.0025, false);
            fcSimulation.simulate(listener);
            var engine = listener.getFlightComputer().telemetry;
            byte[] bytes = Files.readAllBytes(engine.getPacketPath());
            if (engine.getGeneratedCount() < 2 || bytes.length == 0 || bytes.length % 128 != 0)
                throw new AssertionError("FC packet output missing/incomplete");
            for (int offset = 0; offset < bytes.length; offset += 128) {
                int sum = 0;
                for (int i = 0; i < 127; i++) sum += Byte.toUnsignedInt(bytes[offset + i]);
                if ((sum & 255) != Byte.toUnsignedInt(bytes[offset + 127]))
                    throw new AssertionError("Bad FC packet checksum");
            }
            System.out.println("LINUX_FC=PASS generated=" + engine.getGeneratedCount()
                    + " received=" + engine.getReceivedCount() + " bytes=" + bytes.length);

            var failure = new AtomicReference<String>();
            var panelRef = new AtomicReference<Trajectory3DPanel>();
            var frameRef = new AtomicReference<JFrame>();
            var screenshot = new AtomicReference<BufferedImage>();
            var captured = new CountDownLatch(1);
            SwingUtilities.invokeAndWait(() -> {
                var panel = new Trajectory3DPanel(failure::set);
                var frame = new JFrame("OpenRocket MIT - Linux x86_64 trajectory verification");
                panelRef.set(panel);
                frameRef.set(frame);
                panel.setData(new TrajectoryData(branch));
                panel.setTime(branch.getFirstEvent(FlightEvent.Type.APOGEE).getTime());
                frame.add(panel);
                frame.pack();
                frame.setVisible(true);
                panel.snapshot(image -> { screenshot.set(image); captured.countDown(); });
            });
            if (!captured.await(120, TimeUnit.SECONDS))
                throw new AssertionError("3D capture timeout: " + failure.get());
            if (failure.get() != null || !panelRef.get().isAvailable())
                throw new AssertionError("3D failure: " + failure.get());
            ImageIO.write(screenshot.get(), "png", evidence.resolve("linux-trajectory-3d.png").toFile());
            System.out.println("LINUX_TRAJECTORY_RENDER=PASS frames=" + panelRef.get().getRenderedFrames());
            SwingUtilities.invokeAndWait(() -> { panelRef.get().close(); frameRef.get().dispose(); });
            System.out.println("LINUX_PACKAGE_PROBE=PASS");
            System.exit(0);
        } catch (Throwable failure) {
            failure.printStackTrace();
            System.exit(1);
        }
    }
}
