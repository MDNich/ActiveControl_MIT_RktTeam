package info.openrocket.swing.gui.mission;

import info.openrocket.core.document.OpenRocketDocument;
import info.openrocket.core.simulation.mission.*;
import info.openrocket.core.startup.Application;
import info.openrocket.swing.gui.util.URLUtil;
import java.awt.Component;
import java.nio.file.*;
import java.util.concurrent.FutureTask;
import javax.swing.*;

/** One Java-owned mission service for this application; the browser can reconnect without losing jobs. */
public final class MissionWorkspace {
    private static MissionApi api;
    private static MissionStore store;
    private static Component owner;
    private static OpenRocketDocument document;
    private MissionWorkspace() { }

    public static void open(Component parent, OpenRocketDocument current) {
        owner = parent; document = current;
        try {
            if (api == null) {
                Path root = Path.of(System.getProperty("openrocket.mission.root",
                        Path.of(System.getProperty("user.home"), "OpenRocket", "Missions").toString()));
                store = new MissionStore(root, digest -> Application.getMotorSetDatabase()
                        .findMotors(digest, null, null, null, Double.NaN, Double.NaN).stream().findFirst().orElse(null));
                api = new MissionApi(store, new MissionApi.Host() {
                    public Path choose(String purpose, boolean save) throws Exception {
                        return onEdt(() -> {
                            var chooser = new JFileChooser(); chooser.setDialogTitle(purpose);
                            if (purpose.contains("project folder")) chooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
                            if (save) chooser.setSelectedFile(new java.io.File(purpose.contains(".rktmission-setup") ? "mission.rktmission-setup" : "mission.rktmission-config.json"));
                            int result = save ? chooser.showSaveDialog(owner) : chooser.showOpenDialog(owner);
                            if (result != JFileChooser.APPROVE_OPTION) return null;
                            Path selected = chooser.getSelectedFile().toPath();
                            if (save && Files.exists(selected) && JOptionPane.showConfirmDialog(owner,
                                    "Replace " + selected.getFileName() + "?", "Save mission", JOptionPane.YES_NO_OPTION) != JOptionPane.YES_OPTION) return null;
                            return selected;
                        });
                    }
                    public OpenRocketDocument currentDocument() throws Exception {
                        return onEdt(() -> {
                            if (document == null) return null;
                            var copy = document.copy(); copy.setFile(document.getFile()); return copy;
                        });
                    }
                }, null);
                Runtime.getRuntime().addShutdownHook(new Thread(() -> api.close(), "mission-shutdown"));
            }
            var draft = store.create("URRG mission");
            if (!URLUtil.openWebpage(api.uri(MissionStore.id(draft)))) {
                var address = new JTextField(api.uri(MissionStore.id(draft)).toString(), 60); address.setEditable(false);
                JOptionPane.showMessageDialog(parent, address, "Open this workspace address in your browser", JOptionPane.INFORMATION_MESSAGE);
            }
        } catch (Exception e) { JOptionPane.showMessageDialog(parent, e.getMessage(), "Mission workspace could not open", JOptionPane.ERROR_MESSAGE); }
    }
    private static <T> T onEdt(java.util.concurrent.Callable<T> action) throws Exception {
        if (SwingUtilities.isEventDispatchThread()) return action.call();
        var task = new FutureTask<>(action); SwingUtilities.invokeAndWait(task); return task.get();
    }
}
