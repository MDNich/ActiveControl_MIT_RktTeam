import java.awt.*;
import java.awt.image.BufferedImage;
import java.lang.instrument.Instrumentation;
import java.nio.file.*;
import java.nio.file.attribute.PosixFilePermissions;
import java.util.*;
import java.util.prefs.*;
import javax.imageio.ImageIO;
import javax.swing.*;

/** Disposable agent for testing the native launcher, real windows, menu and file choosers.
 * No production hooks or replacement Mission host. Never load this in a user's session. */
public final class MissionDesktopSmokeAgent {
    private static Path root;
    private static boolean welcomed, exampleRequested, missionRequested, connected;
    private static long welcomedAt;
    private static ClassLoader applicationLoader;

    public static void premain(String directory, Instrumentation instrumentation) throws Exception {
        root = Path.of(directory).toAbsolutePath();
        Files.createDirectories(root.resolve("home"));
        System.setProperty("user.home", root.resolve("home").toString());
        // macOS ignores java.util.prefs.userRoot: isolate all preferences explicitly.
        System.setProperty("java.util.prefs.PreferencesFactory", MemoryPreferencesFactory.class.getName());
        System.setProperty("openrocket.mission.root", root.resolve("missions").toString());
        Thread observer = new Thread(() -> {
            try {
                long deadline = System.nanoTime() + 600_000_000_000L;
                while (System.nanoTime() < deadline) {
                    Thread.sleep(250);
                    if (Thread.getAllStackTraces().keySet().stream()
                            .noneMatch(t -> t.getName().startsWith("AWT-EventQueue-"))) continue;
                    SwingUtilities.invokeAndWait(() -> {
                        try { observe(); } catch (Throwable failure) { fail(failure); }
                    });
                    if (connected && Files.exists(root.resolve("browser-passed"))) {
                        Files.writeString(root.resolve("desktop-passed"),
                                "Native launcher, welcome, example rocket, Mission menu and browser acceptance passed.\n");
                        System.out.println("NATIVE_DESKTOP_MISSION=PASS");
                        System.exit(0);
                    }
                }
                throw new AssertionError("Desktop acceptance timed out");
            } catch (Throwable failure) { fail(failure); }
        }, "mission-desktop-verification");
        observer.setDaemon(true);
        observer.start();
    }

    private static void observe() throws Exception {
        JFrame main = null;
        for (Window window : Window.getWindows()) {
            if (!window.isShowing()) continue;
            String name = window.getClass().getName();
            if (name.contains("BugReport") || name.contains("ExceptionDialog"))
                throw new AssertionError("Application error dialog: " + name);
            if (name.endsWith("WelcomeDialog")) {
                capture(window, "welcome.png");
                welcomed = true;
                welcomedAt = System.nanoTime();
                window.dispose();
            }
            if (name.equals("info.openrocket.swing.gui.main.BasicFrame")) main = (JFrame) window;
            JFileChooser chooser = findChooser(window);
            if (chooser != null && Files.exists(root.resolve("chooser-next-path"))) {
                String path = Files.readString(root.resolve("chooser-next-path")).strip();
                if (path.startsWith("{")) {
                    Object json = applicationLoader.loadClass("info.openrocket.core.simulation.mission.MissionJson")
                            .getMethod("parse", byte[].class).invoke(null, (Object) path.getBytes(java.nio.charset.StandardCharsets.UTF_8));
                    path = (String) applicationLoader.loadClass("jakarta.json.JsonObject")
                            .getMethod("getString", String.class).invoke(json, chooser.getDialogTitle());
                }
                // Test selections are constrained to this disposable acceptance directory.
                Path selected = Path.of(path).toAbsolutePath().normalize();
                if (!selected.startsWith(root)) throw new AssertionError("Chooser path outside test directory");
                chooser.setSelectedFile(selected.toFile());
                chooser.approveSelection();
            }
        }
        if (!welcomed || main == null || System.nanoTime() - welcomedAt < 1_000_000_000L) return;
        applicationLoader = main.getClass().getClassLoader();
        if (!exampleRequested) {
            exampleRequested = true;
            JFrame parent = main;
            var example = applicationLoader.getResource("datafiles/examples/A simple model rocket.ork");
            if (example == null) throw new AssertionError("Missing packaged example");
            // Use the application's normal example loader, after normal no-argument startup.
            SwingUtilities.invokeLater(() -> {
                try { parent.getClass().getMethod("open", java.net.URL.class, parent.getClass()).invoke(null, example, parent); }
                catch (Throwable failure) { fail(failure); }
            });
            return;
        }
        if (!missionRequested && main.getTitle().contains("simple model")) {
            capture(main, "rocket.png");
            JMenuItem item = findMenu(main.getJMenuBar());
            if (item == null) throw new AssertionError("Mission menu is missing");
            missionRequested = true;
            SwingUtilities.invokeLater(item::doClick);
            return;
        }
        if (missionRequested && !connected) {
            var workspace = applicationLoader.loadClass("info.openrocket.swing.gui.mission.MissionWorkspace");
            var field = workspace.getDeclaredField("api"); field.setAccessible(true);
            Object api = field.get(null);
            if (api == null) return;
            Path drafts = root.resolve("missions/drafts");
            try (var files = Files.list(drafts)) {
                var draft = files.filter(p -> p.toString().endsWith(".json")).findFirst().orElse(null);
                if (draft == null) return;
                String id = draft.getFileName().toString().replace(".json", "");
                String address = api.getClass().getMethod("uri", String.class).invoke(api, id).toString();
                Path destination = root.resolve("workspace-url");
                Files.createFile(destination, PosixFilePermissions.asFileAttribute(PosixFilePermissions.fromString("rw-------")));
                Files.writeString(destination, address);
            }
            connected = true;
            System.out.println("NATIVE_DESKTOP_MISSION=READY");
        }
    }

    private static JFileChooser findChooser(Container parent) {
        if (parent instanceof JFileChooser chooser) return chooser;
        for (Component child : parent.getComponents()) if (child instanceof Container container) {
            JFileChooser found = findChooser(container); if (found != null) return found;
        }
        return null;
    }
    private static JMenuItem findMenu(MenuElement parent) {
        if (parent instanceof JMenuItem item && "New Mission Simulation...".equals(item.getText())) return item;
        for (MenuElement child : parent.getSubElements()) {
            JMenuItem found = findMenu(child); if (found != null) return found;
        }
        return null;
    }
    private static void capture(Window window, String filename) throws Exception {
        var image = new BufferedImage(window.getWidth(), window.getHeight(), BufferedImage.TYPE_INT_RGB);
        var graphics = image.createGraphics(); window.paint(graphics); graphics.dispose();
        ImageIO.write(image, "png", root.resolve(filename).toFile());
    }
    private static void fail(Throwable failure) {
        failure.printStackTrace(System.out);
        try { Files.writeString(root.resolve("desktop-failed"), failure.toString()); } catch (Exception ignored) { }
        System.exit(1);
    }

    /** In-memory test preferences keep the real user's macOS preference domain untouched. */
    public static final class MemoryPreferencesFactory implements PreferencesFactory {
        private final Preferences user = new MemoryPreferences(null, "");
        private final Preferences system = new MemoryPreferences(null, "");
        public Preferences userRoot() { return user; }
        public Preferences systemRoot() { return system; }
    }
    private static final class MemoryPreferences extends AbstractPreferences {
        private final Map<String, String> values = new HashMap<>();
        private final Map<String, MemoryPreferences> children = new HashMap<>();
        MemoryPreferences(AbstractPreferences parent, String name) { super(parent, name); }
        protected void putSpi(String key, String value) { values.put(key, value); }
        protected String getSpi(String key) { return values.get(key); }
        protected void removeSpi(String key) { values.remove(key); }
        protected void removeNodeSpi() { values.clear(); if (parent() instanceof MemoryPreferences p) p.children.remove(name()); }
        protected String[] keysSpi() { return values.keySet().toArray(String[]::new); }
        protected String[] childrenNamesSpi() { return children.keySet().toArray(String[]::new); }
        protected AbstractPreferences childSpi(String name) { return children.computeIfAbsent(name, n -> new MemoryPreferences(this, n)); }
        protected void syncSpi() { }
        protected void flushSpi() { }
    }
}
