package info.openrocket.swing.gui.mission;

import info.openrocket.core.document.OpenRocketDocument;
import info.openrocket.core.simulation.mission.*;
import info.openrocket.core.startup.Application;
import java.awt.Component;
import java.nio.file.*;
import java.util.concurrent.FutureTask;
import javax.swing.*;

/**
 * One native mission workspace per OpenRocket process. Closing its window preserves an active run.
 */
public final class MissionWorkspace {
  private static MissionWindow window;
  private static MissionService service;

  private MissionWorkspace() {}

  public static void open(Component parent, OpenRocketDocument current) {
    try {
      if (window == null) {
        Path root =
            Path.of(
                System.getProperty(
                    "openrocket.mission.root",
                    Path.of(System.getProperty("user.home"), "OpenRocket", "Missions").toString()));
        var store =
            new MissionStore(
                root,
                digest ->
                    Application.getMotorSetDatabase()
                        .findMotors(digest, null, null, null, Double.NaN, Double.NaN)
                        .stream()
                        .findFirst()
                        .orElse(null));
        service =
            new MissionService(
                store,
                new MissionService.Host() {
                  public Path choose(String purpose, boolean save) throws Exception {
                    return onEdt(
                        () -> {
                          var chooser = new JFileChooser();
                          chooser.setDialogTitle(purpose);
                          if (purpose.contains("folder"))
                            chooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
                          if (save)
                            chooser.setSelectedFile(
                                new java.io.File(
                                    purpose.contains(".rktmission-setup")
                                        ? "mission.rktmission-setup"
                                        : purpose.contains(".rktflight")
                                            ? "station.rktflight"
                                            : purpose.contains(".rktmission")
                                                ? "mission.rktmission"
                                                : "mission.rktmission-config.json"));
                          int result =
                              save
                                  ? chooser.showSaveDialog(window)
                                  : chooser.showOpenDialog(window);
                          if (result != JFileChooser.APPROVE_OPTION) return null;
                          Path selected = chooser.getSelectedFile().toPath();
                          if (save
                              && Files.exists(selected)
                              && JOptionPane.showConfirmDialog(
                                      window,
                                      "Replace " + selected.getFileName() + "?",
                                      "Save mission",
                                      JOptionPane.YES_NO_OPTION)
                                  != JOptionPane.YES_OPTION) return null;
                          return selected;
                        });
                  }

                  public OpenRocketDocument currentDocument() throws Exception {
                    return onEdt(() -> window.documentCopy());
                  }
                },
                new MissionJobs(root));
        window = new MissionWindow(service, current, store.create("URRG mission"));
        Runtime.getRuntime().addShutdownHook(new Thread(service::close, "mission-shutdown"));
      } else window.currentDocument(current);
      window.setVisible(true);
      window.toFront();
    } catch (Exception e) {
      JOptionPane.showMessageDialog(
          parent, e.getMessage(), "Mission workspace could not open", JOptionPane.ERROR_MESSAGE);
    }
  }

  private static <T> T onEdt(java.util.concurrent.Callable<T> action) throws Exception {
    if (SwingUtilities.isEventDispatchThread()) return action.call();
    var task = new FutureTask<>(action);
    SwingUtilities.invokeAndWait(task);
    return task.get();
  }
}
