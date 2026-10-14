import java.awt.Frame;
import java.lang.instrument.Instrumentation;
import java.util.concurrent.atomic.AtomicReference;
import javax.swing.JFrame;
import javax.swing.JMenu;
import javax.swing.JMenuItem;
import javax.swing.SwingUtilities;

/** Observes only this test JVM, checks the loaded main window, then exits it. */
public final class VanillaExportGuiAgent {
    public static void premain(String mode, Instrumentation instrumentation) {
        Thread observer = new Thread(() -> {
            try {
                for (int attempt = 0; attempt < 60; attempt++) {
                    Thread.sleep(1000);
                    AtomicReference<String> found = new AtomicReference<>();
                    SwingUtilities.invokeAndWait(() -> {
                        for (Frame frame : Frame.getFrames()) {
                            if (!frame.isShowing() || !frame.getClass().getName()
                                    .equals("info.openrocket.swing.gui.main.BasicFrame")) continue;
                            if (!frame.getTitle().contains("vanilla-24.12")) continue;
                            if (mode.equals("mit")) {
                                boolean present = false;
                                var bar = ((JFrame) frame).getJMenuBar();
                                for (int i = 0; i < bar.getMenuCount(); i++) present |= containsExport(bar.getMenu(i));
                                if (!present) throw new AssertionError("Vanilla export menu missing or disabled");
                            }
                            found.set(frame.getTitle());
                        }
                    });
                    if (found.get() != null) {
                        System.out.println("VANILLA_EXPORT_GUI_PASS mode=" + mode + " window=" + found.get());
                        System.exit(0);
                    }
                }
                throw new AssertionError("Expected exported rocket window did not appear");
            } catch (Throwable failure) {
                failure.printStackTrace();
                System.exit(1);
            }
        }, "vanilla-export-gui-verification");
        observer.setDaemon(true);
        observer.start();
    }

    private static boolean containsExport(JMenu menu) {
        if (menu == null) return false;
        for (int i = 0; i < menu.getItemCount(); i++) {
            JMenuItem item = menu.getItem(i);
            if (item != null && "Vanilla OpenRocket 24.12 (.ork)…".equals(item.getText()) && item.isEnabled()) return true;
            if (item instanceof JMenu nested && containsExport(nested)) return true;
        }
        return false;
    }
}
