package info.openrocket.swing.gui.dialogs;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import javax.swing.JEditorPane;
import javax.swing.SwingUtilities;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.google.inject.Guice;
import info.openrocket.core.plugin.PluginModule;
import info.openrocket.core.startup.Application;
import info.openrocket.swing.startup.GuiModule;

class BugReportDialogTest {

    @BeforeAll
    static void setup() {
        Application.setInjector(Guice.createInjector(new GuiModule(), new PluginModule()));
    }

    @Test
    void copiedAndSavedReportIncludesEditsAndCompletePlainTextDiagnostics() throws Exception {
        SwingUtilities.invokeAndWait(() -> {
            try {
                String logs = "diagnostic entry ä &lt;frame&gt;<br>".repeat(4000);
                JEditorPane editor = new JEditorPane("text/html",
                        "<html><p>Error: &lt;boom&gt; &amp; detail</p><p>" + logs + "FINAL ENTRY</p></html>");
                editor.getDocument().insertString(0, "User description: opening my rocket\n", null);
                String report = BugReportDialog.reportText(editor);
                assertTrue(report.contains("User description: opening my rocket"));
                assertTrue(report.contains("Error: <boom> & detail"));
                assertTrue(report.contains("diagnostic entry ä <frame>"));
                assertTrue(report.contains("FINAL ENTRY"));
                assertTrue(report.length() > 100000, "Reports must not be truncated to fit mailto links");
                assertFalse(report.contains("<html>"));
            } catch (Exception failure) {
                throw new AssertionError(failure);
            }
        });
    }
}
