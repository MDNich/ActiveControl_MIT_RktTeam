import java.awt.Frame;
import java.lang.instrument.Instrumentation;
import java.util.concurrent.atomic.AtomicReference;
import javax.swing.SwingUtilities;

/** A disposable launch test: observe this test JVM's main window, then exit it. */
public final class LauncherSmokeAgent {
    public static void premain(String expectedArchitecture, Instrumentation instrumentation) {
        Thread observer = new Thread(() -> {
            try {
                if (!expectedArchitecture.equals(System.getProperty("os.arch")))
                    throw new AssertionError("Wrong bundled runtime: " + System.getProperty("os.arch"));
                System.out.println("LAUNCHER_JVM_ARCH=" + System.getProperty("os.arch"));
                System.out.println("LAUNCHER_JAVA_HOME=" + System.getProperty("java.home"));
                AtomicReference<String> title = new AtomicReference<>();
                for (int attempt = 0; attempt < 60; attempt++) {
                    Thread.sleep(1000);
                    SwingUtilities.invokeAndWait(() -> {
                        for (Frame frame : Frame.getFrames()) {
                            if (frame.isShowing() && frame.getClass().getName().equals("info.openrocket.swing.gui.main.BasicFrame"))
                                title.set(frame.getTitle());
                        }
                    });
                    if (title.get() != null) {
                        System.out.println("LAUNCHER_WINDOW=" + title.get());
                        System.out.println("LAUNCHER_SMOKE_CHECK=PASS");
                        System.exit(0);
                    }
                }
                throw new AssertionError("No main application window appeared");
            } catch (Throwable error) {
                error.printStackTrace();
                System.exit(1);
            }
        }, "packaged-launch-verification");
        observer.setDaemon(true);
        observer.start();
    }
}
