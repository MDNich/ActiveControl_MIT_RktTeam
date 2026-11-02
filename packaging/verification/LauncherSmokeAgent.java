import java.awt.Frame;
import java.lang.instrument.Instrumentation;
import java.util.concurrent.atomic.AtomicReference;
import javax.swing.SwingUtilities;

/** A disposable launch test: observe this test JVM's main window, then exit it. */
public final class LauncherSmokeAgent {
    /**
     * Installs this packaged-application verification agent before the application starts.
     *
     * @param expectedArchitecture expected architecture
     * @param instrumentation instrumentation
     * @throws AssertionError when rejected: Wrong bundled runtime:
     */
    public static void premain(String expectedArchitecture, Instrumentation instrumentation) {
        Thread observer =
                new Thread(
                        () -> {
                            try {
                                if (!expectedArchitecture.equals(System.getProperty("os.arch")))
                                    throw new AssertionError(
                                            "Wrong bundled runtime: "
                                                    + System.getProperty("os.arch"));
                                System.out.println(
                                        "LAUNCHER_JVM_ARCH=" + System.getProperty("os.arch"));
                                System.out.println(
                                        "LAUNCHER_JAVA_HOME=" + System.getProperty("java.home"));
                                AtomicReference<String> title = new AtomicReference<>();
                                int waitSeconds = Integer.getInteger(
                                        "openrocket.verification.launchTimeoutSeconds", 60);
                                if (waitSeconds <= 0)
                                    throw new AssertionError("Launch timeout must be positive");
                                for (int attempt = 0; attempt < waitSeconds; attempt++) {
                                    Thread.sleep(1000);
                                    // Do not create Swing's first event thread from this agent.
                                    // OpenRocket sets up its own context class loader first;
                                    // initializing AWT earlier can break startup on slow hosts.
                                    if (Thread.getAllStackTraces().keySet().stream()
                                            .noneMatch(thread -> thread.getName()
                                                    .startsWith("AWT-EventQueue-")))
                                        continue;
                                    SwingUtilities.invokeAndWait(
                                            () -> {
                                                for (Frame frame : Frame.getFrames()) {
                                                    if (frame.isShowing()
                                                            && frame.getClass()
                                                                    .getName()
                                                                    .equals(
                                                                            "info.openrocket.swing.gui.main.BasicFrame"))
                                                        title.set(frame.getTitle());
                                                }
                                            });
                                    if (title.get() != null) {
                                        System.out.println("LAUNCHER_WINDOW=" + title.get());
                                        System.out.println("LAUNCHER_SMOKE_CHECK=PASS");
                                        System.exit(0);
                                    }
                                }
                                throw new AssertionError("No main application window appeared after "
                                        + waitSeconds + " observation attempts");
                            } catch (Throwable error) {
                                // The application redirects stderr into its logger during startup.
                                error.printStackTrace(System.out);
                                System.exit(1);
                            }
                        },
                        "packaged-launch-verification");
        observer.setDaemon(true);
        observer.start();
    }
}
