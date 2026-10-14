package info.openrocket.swing.gui.dialogs;

import java.awt.Dimension;
import java.awt.Window;
import java.io.File;
import java.io.IOException;
import java.util.concurrent.ExecutionException;

import javax.swing.JFileChooser;
import javax.swing.JOptionPane;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.SwingWorker;

import info.openrocket.core.document.OpenRocketDocument;
import info.openrocket.core.file.openrocket.VanillaOpenRocketExporter;
import info.openrocket.core.startup.Application;
import info.openrocket.swing.gui.util.FileHelper;
import info.openrocket.swing.gui.util.SwingPreferences;
import info.openrocket.swing.gui.widgets.SaveFileChooser;

/** Separate-copy export; never changes the open document's path or saved state. */
public final class VanillaExportDialog {
    public static final String MENU_LABEL = "Vanilla OpenRocket 24.12 (.ork)…";
    private static final String TITLE = "Export for vanilla OpenRocket 24.12";

    private VanillaExportDialog() { }

    public static File suggestedFile(OpenRocketDocument document, File defaultDirectory) {
        File source = document.getFileNoExtension();
        if (source == null) return new File(defaultDirectory, "rocket-vanilla-24.12.ork");
        return new File(source.getParentFile(), source.getName() + "-vanilla-24.12.ork");
    }

    public static void show(Window parent, OpenRocketDocument document) {
        String details = "Create a separate copy for unmodified OpenRocket 24.12.\n"
                + "Your MIT design stays open and unchanged.\n\n"
                + String.join("\n\n", VanillaOpenRocketExporter.describeChanges(document));
        JTextArea text = new JTextArea(details);
        text.setEditable(false);
        text.setLineWrap(true);
        text.setWrapStyleWord(true);
        text.setCaretPosition(0);
        JScrollPane scroll = new JScrollPane(text);
        scroll.setPreferredSize(new Dimension(640, 330));
        if (JOptionPane.showConfirmDialog(parent, scroll, TITLE, JOptionPane.OK_CANCEL_OPTION,
                JOptionPane.INFORMATION_MESSAGE) != JOptionPane.OK_OPTION) return;

        SwingPreferences preferences = (SwingPreferences) Application.getPreferences();
        SaveFileChooser chooser = new SaveFileChooser();
        chooser.setDialogTitle(TITLE);
        chooser.setAcceptAllFileFilterUsed(false);
        chooser.setFileFilter(FileHelper.OPENROCKET_DESIGN_FILTER);
        chooser.setCurrentDirectory(preferences.getDefaultDirectory());
        chooser.setSelectedFile(suggestedFile(document, preferences.getDefaultDirectory()));
        if (chooser.showSaveDialog(parent) != JFileChooser.APPROVE_OPTION) return;
        File file = FileHelper.forceExtension(chooser.getSelectedFile(), "ork");
        try {
            if (VanillaOpenRocketExporter.isSourceFile(document, file)) {
                JOptionPane.showMessageDialog(parent,
                        "Choose a different filename to keep your original MIT design.", TITLE,
                        JOptionPane.WARNING_MESSAGE);
                return;
            }
        } catch (IOException e) {
            showError(parent, e);
            return;
        }
        if (!FileHelper.confirmWrite(file, parent)) return;
        preferences.setDefaultDirectory(chooser.getCurrentDirectory());
        SwingWorker<Void, Void> worker = new SwingWorker<>() {
            @Override
            protected Void doInBackground() throws Exception {
                new VanillaOpenRocketExporter().export(document, file);
                return null;
            }
        };
        if (!SwingWorkerDialog.runWorker(parent, TITLE, "Writing " + file.getName() + "…", worker)) return;
        try {
            worker.get();
            JOptionPane.showMessageDialog(parent, "Exported to:\n" + file.getAbsolutePath()
                    + "\n\nOpen this copy in OpenRocket 24.12 and rerun its simulations.", TITLE,
                    JOptionPane.INFORMATION_MESSAGE);
        } catch (ExecutionException e) {
            showError(parent, e.getCause());
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            showError(parent, e);
        }
    }

    private static void showError(Window parent, Throwable cause) {
        JOptionPane.showMessageDialog(parent, "Export failed: " + cause.getMessage(), TITLE,
                JOptionPane.ERROR_MESSAGE);
    }
}
