import java.awt.Frame;
import java.io.IOException;
import java.lang.instrument.Instrumentation;
import java.net.Proxy;
import java.net.ProxySelector;
import java.net.URI;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.util.List;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.concurrent.atomic.AtomicBoolean;
import javax.swing.SwingUtilities;

/** Observes a disposable update restart without contacting any release service. */
public class UpdateRestartAgent {
    public static void premain(String unused, Instrumentation instrumentation) {
        try {
            System.setOut(new java.io.PrintStream(System.getenv("OR_UPDATE_TEST_MARKER") + ".log", java.nio.charset.StandardCharsets.UTF_8));
            System.setErr(System.out);
            System.setProperty("openrocket.log.stdout", "INFO");
        } catch (Exception failure) { throw new RuntimeException(failure); }
        ProxySelector.setDefault(new ProxySelector() {
            public List<Proxy> select(URI uri) {
                return List.of(uri.getScheme().equals("https")
                    ? new Proxy(Proxy.Type.HTTP, new InetSocketAddress("127.0.0.1", 9)) : Proxy.NO_PROXY);
            }
            public void connectFailed(URI uri, SocketAddress address, IOException failure) { }
        });
        // The initial process is a console harness. Only watch the restart via -jar or the launcher.
        if (System.getProperty("sun.java.command", "").startsWith("WindowsUpdateProbe")) return;
        new Thread(() -> {
            Path marker = Path.of(System.getenv("OR_UPDATE_TEST_MARKER"));
            try {
                AtomicBoolean found = new AtomicBoolean();
                for (int attempt = 0; attempt < 60; attempt++) {
                    Thread.sleep(1000);
                    SwingUtilities.invokeAndWait(() -> {
                        for (Frame frame : Frame.getFrames()) {
                            if (frame.isShowing() && frame.getClass().getName().equals("info.openrocket.swing.gui.main.BasicFrame"))
                                found.set(true);
                        }
                    });
                    if (found.get()) {
                        Files.writeString(marker, "PASS: restarted application displayed its main window; java.home="
                            + System.getProperty("java.home"));
                        System.exit(0);
                    }
                }
                SwingUtilities.invokeAndWait(() -> {
                    for (java.awt.Window window : java.awt.Window.getWindows()) {
                        if (window.isShowing()) System.out.println("Visible window: " + window);
                    }
                });
                throw new AssertionError("Restarted application did not display its main window");
            } catch (Throwable failure) {
                try { Files.writeString(marker, "FAIL: " + failure); } catch (Exception ignored) { }
                System.exit(1);
            }
        }, "update-restart-observer").start();
    }
}
