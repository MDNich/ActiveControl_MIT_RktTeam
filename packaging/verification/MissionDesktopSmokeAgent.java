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
    private static boolean exampleRequested, missionRequested, connected;
    private static long mainWindowSeenAt;
    private static ClassLoader applicationLoader;
    private static int step, site;
    private static long stepAt;
    private static JFrame missionWindow;
    private static boolean paused;


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
            if (name.endsWith("WelcomeDialog"))
                throw new AssertionError("Welcome dialog must not appear on launch");
            if (name.equals("info.openrocket.swing.gui.main.BasicFrame")) main = (JFrame) window;
            if(window instanceof JDialog dialog && "Mission Simulation".equals(dialog.getTitle())) {
                capture(window,"mission-error.png");
                throw new AssertionError("Native Mission error dialog at step "+step+": "+collectText(window));
            }
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
        if (main == null) return;
        if (mainWindowSeenAt == 0) mainWindowSeenAt = System.nanoTime();
        if (System.nanoTime() - mainWindowSeenAt < 1_000_000_000L) return;
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
        if (missionRequested) nativeMission();
    }

    private static Object field(Object object, String name) throws Exception {
        var f=object.getClass().getDeclaredField(name);f.setAccessible(true);return f.get(object);
    }
    private static Component named(Container parent,String name) {
        if(name.equals(parent.getName()))return parent;
        for(Component child:parent.getComponents()) {
            if(name.equals(child.getName()))return child;
            if(child instanceof Container container){Component found=named(container,name);if(found!=null)return found;}
        }
        return null;
    }
    private static void click(String name) {
        Component value=named(missionWindow,name);
        if(!(value instanceof AbstractButton button))throw new AssertionError("Missing native button "+name);
        SwingUtilities.invokeLater(button::doClick);
    }
    private static void text(String name,String value) {
        Component field=named(missionWindow,name);
        if(!(field instanceof JTextField input))throw new AssertionError("Missing field "+name);
        input.setText(value);
    }
    private static String jsonString(Object json,String key)throws Exception {
        return (String)applicationLoader.loadClass("jakarta.json.JsonObject").getMethod("getString",String.class).invoke(json,key);
    }
    private static void advance(){step++;stepAt=System.nanoTime();}
    private static void nativeMission()throws Exception {
        for(Window w:Window.getWindows())if(w.isShowing()&&"mission-window".equals(w.getName()))missionWindow=(JFrame)w;
        if(missionWindow==null || Boolean.TRUE.equals(field(missionWindow,"busy")))return;
        if(stepAt!=0 && System.nanoTime()-stepAt<400_000_000L)return;
        JTabbedPane tabs=(JTabbedPane)field(missionWindow,"tabs");
        switch(step){
            case 0:
                capture(missionWindow,"mission-sites.png");
                if(tabs.getTabCount()!=6)throw new AssertionError("Expected six native mission pages");
                // All locations remain explicit; test heights are never application defaults.
                text("/sites/0/heightM","250");
                site=1;advance();break;
            case 1:
                if(site<6){
                    JComboBox<?> selector=(JComboBox<?>)named(missionWindow,"mission-site");
                    selector.setSelectedIndex(site);
                    text("/sites/"+site+"/heightM","250");site++;
                }else {tabs.setSelectedIndex(1);click("mission-current-rocket");advance();}
                break;
            case 2:
                if(((JComboBox<?>)named(missionWindow,"mission-simulation-choice")).getItemCount()==0)return;
                click("mission-select-flight");advance();break;
            case 3:
                if(named(missionWindow,"/flight/editor/options/maximumTimeS")==null)return;
                text("/flight/editor/options/maximumTimeS","6");
                capture(missionWindow,"mission-flight.png");tabs.setSelectedIndex(2);advance();break;
            case 4:
                capture(missionWindow,"mission-telemetry.png");tabs.setSelectedIndex(3);advance();break;
            case 5:
                ((JCheckBox)named(missionWindow,"/outputs/recordEncodedVideo")).setSelected(true);capture(missionWindow,"mission-video.png");tabs.setSelectedIndex(4);click("mission-network-start");advance();break;
            case 6:
                capture(missionWindow,"mission-network.png");
                Files.writeString(root.resolve("network-services.txt"),((JTextArea)named(missionWindow,"mission-network-endpoints")).getText());
                tabs.setSelectedIndex(5);click("mission-validate");advance();break;
            case 7:
                String findings=((JTextArea)named(missionWindow,"mission-findings")).getText();
                if(!findings.startsWith("Configuration valid"))throw new AssertionError("Native validation failed: "+findings);
                click("mission-start");advance();break;
            case 8:
                Object job=field(missionWindow,"job");if(job==null)return;
                String state=jsonString(job,"status");
                if(state.equals("failed"))throw new AssertionError("Mission failed: "+job);
                if(!paused && state.equals("running")){click("mission-pause");paused=true;advance();}
                break;
            case 9:
                Object p=field(missionWindow,"job");if(!jsonString(p,"status").equals("paused"))return;
                capture(missionWindow,"mission-paused.png");click("mission-resume");advance();break;
            case 10:
                Object result=field(missionWindow,"job");String terminal=jsonString(result,"status");
                if(terminal.equals("failed"))throw new AssertionError("Mission failed: "+result);
                if(!terminal.equals("complete"))return;
                Path archive=Path.of(jsonString(result,"result"));if(!Files.isRegularFile(archive))throw new AssertionError("Missing result bundle");
                try(var zip=new java.util.zip.ZipFile(archive.toFile())){if(zip.stream().noneMatch(e->e.getName().contains("/video/encoded/")&&e.getName().endsWith(".mkv")))throw new AssertionError("No encoded video in mission bundle");}
                Files.writeString(root.resolve("job.json"),result.toString());capture(missionWindow,"mission-complete.png");
                // Use the application's real save chooser.
                Files.writeString(root.resolve("chooser-next-path"),root.resolve("saved.mission.json").toString());click("mission-save");advance();break;
            case 11:
                if(!Files.isRegularFile(root.resolve("saved.mission.json")))return;
                Files.deleteIfExists(root.resolve("chooser-next-path"));
                tabs.setSelectedIndex(4);click("mission-network-stop");advance();break;
            case 12:
                tabs.setSelectedIndex(5);click("mission-result-current");advance();break;
            case 13:
                if(field(missionWindow,"opened")==null)return;
                click("mission-result-read");advance();break;
            case 14:
                JTable records=(JTable)named(missionWindow,"mission-result-records");
                if(records==null||records.getRowCount()==0)throw new AssertionError("No indexed result rows");
                capture(missionWindow,"mission-results.png");click("mission-result-derive");advance();break;
            case 15:
                if(!field(missionWindow,"mission").toString().contains("recordedFlight"))throw new AssertionError("Recorded experiment was not created");
                tabs.setSelectedIndex(0);advance();break;
            case 16:
                capture(missionWindow,"mission-map-loaded.png");
                Files.writeString(root.resolve("desktop-passed"),"Native launcher, six Swing mission pages, location/flight editing, network advertising, validation, paced run, pause/resume, result bundle and native Save chooser passed.\n");
                System.out.println("NATIVE_DESKTOP_MISSION=PASS");System.exit(0);break;
            default:throw new AssertionError("Unexpected step");
        }
    }

    private static String collectText(Container parent){StringBuilder text=new StringBuilder();for(Component c:parent.getComponents()){if(c instanceof JLabel label)text.append(label.getText()).append(" ");if(c instanceof Container container)text.append(collectText(container));}return text.toString();}
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
