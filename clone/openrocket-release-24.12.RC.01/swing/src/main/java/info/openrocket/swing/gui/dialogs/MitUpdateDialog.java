package info.openrocket.swing.gui.dialogs;

import java.awt.Color;
import java.awt.Cursor;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.util.ArrayList;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.JScrollPane;
import javax.swing.JTextPane;
import javax.swing.SwingUtilities;
import javax.swing.SwingWorker;
import javax.swing.event.HyperlinkEvent;

import info.openrocket.core.communication.MitReleaseInfo;
import info.openrocket.core.communication.MitUpdateInfo;
import info.openrocket.core.startup.Application;
import info.openrocket.core.util.BuildProperties;
import info.openrocket.core.util.MarkdownUtil;
import info.openrocket.swing.gui.components.StyledLabel;
import info.openrocket.swing.gui.theme.UITheme;
import info.openrocket.swing.gui.util.GUIUtil;
import info.openrocket.swing.gui.util.Icons;
import info.openrocket.swing.gui.util.MitUpdateInstaller;
import info.openrocket.swing.gui.util.URLUtil;

import net.miginfocom.swing.MigLayout;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Dialog shown when an MIT-edition update is available.
 */
public class MitUpdateDialog extends JDialog {
	private static final Logger log = LoggerFactory.getLogger(MitUpdateDialog.class);

	private final JLabel progressLabel = new JLabel(" ");
	private final JProgressBar progressBar = new JProgressBar(0, 100);

	private static Color textColor;

	static {
		initColors();
	}

	public MitUpdateDialog(MitUpdateInfo info) {
		super(null, "MIT edition update available", ModalityType.APPLICATION_MODAL);

		MitReleaseInfo release = info.getLatestRelease();
		JPanel panel = new JPanel(new MigLayout("insets n n 8px n, fill"));

		panel.add(new JLabel(Icons.loadImageIcon("pix/icon/icon-128.png", "OpenRocket")),
				"spany, top, gapright 20px, cell 0 0");

		panel.add(new StyledLabel("OpenRocket MIT edition " + release.getReleaseVersion() + " is available",
				8, StyledLabel.Style.BOLD), "spanx, wrap");

		panel.add(new StyledLabel(String.format("Your MIT edition version is %s.",
				BuildProperties.getMitVersion()), -1, StyledLabel.Style.PLAIN), "skip 1, spanx, wrap para");

		panel.add(new StyledLabel("Release notes", 1, StyledLabel.Style.BOLD), "spanx, wrap");

		final JTextPane textPane = new JTextPane();
		textPane.setBorder(BorderFactory.createCompoundBorder(
				BorderFactory.createLineBorder(textColor),
				BorderFactory.createEmptyBorder(0, 10, 10, 10)
		));
		textPane.setEditable(false);
		textPane.setContentType("text/html");
		textPane.putClientProperty(JTextPane.HONOR_DISPLAY_PROPERTIES, true);
		textPane.setText(buildReleaseHtml(release));
		textPane.setCaretPosition(0);
		textPane.addHyperlinkListener(e -> {
			if (e.getEventType().equals(HyperlinkEvent.EventType.ACTIVATED) && e.getURL() != null) {
				URLUtil.openWebpage(e.getURL().toString());
			}
		});

		panel.add(new JScrollPane(textPane), "skip 1, left, spanx, grow, push, gapbottom 6px, wrap");

		progressLabel.setVisible(false);
		progressBar.setStringPainted(true);
		progressBar.setVisible(false);
		panel.add(progressLabel, "skip 1, spanx, growx, wrap");
		panel.add(progressBar, "skip 1, spanx, growx, wrap para");

		JButton btnLater = new JButton("Later");
		btnLater.addActionListener(e -> MitUpdateDialog.this.dispose());
		panel.add(btnLater, "skip 1, split 4");

		JButton btnSkip = new JButton("Skip this version");
		btnSkip.addActionListener(e -> {
			List<String> ignoredVersions = new ArrayList<>(Application.getPreferences().getIgnoreMitUpdateVersions());
			String version = release.getReleaseVersion();
			if (!ignoredVersions.contains(version)) {
				ignoredVersions.add(version);
				Application.getPreferences().setIgnoreMitUpdateVersions(ignoredVersions);
			}
			MitUpdateDialog.this.dispose();
		});
		panel.add(btnSkip);

		JButton btnReleasePage = new JButton("Open release page");
		btnReleasePage.addActionListener(e -> URLUtil.openWebpage(release.getReleaseURL()));
		btnReleasePage.setEnabled(release.getReleaseURL() != null && !release.getReleaseURL().isBlank());
		panel.add(btnReleasePage);

		JButton btnInstall = new JButton("Download and Install");
		btnInstall.addActionListener((ActionEvent e) -> runInstallWorker(info, btnInstall));
		panel.add(btnInstall, "wrap");

		panel.setPreferredSize(new Dimension(850, 650));

		this.add(panel);
		this.pack();
		this.setLocationRelativeTo(null);
		GUIUtil.setDisposableDialogOptions(this, btnLater);
	}

	private void runInstallWorker(MitUpdateInfo info, JButton btnInstall) {
		btnInstall.setEnabled(false);
		setCursor(Cursor.getPredefinedCursor(Cursor.WAIT_CURSOR));
		updateProgress("Starting download", -1);
		progressLabel.setVisible(true);
		progressBar.setVisible(true);
		revalidate();
		pack();

		SwingWorker<Void, Void> worker = new SwingWorker<>() {
			@Override
			protected Void doInBackground() throws Exception {
				MitUpdateInstaller.downloadVerifyAndLaunchInstaller(info,
						(message, percent) -> SwingUtilities.invokeLater(() -> updateProgress(message, percent)));
				return null;
			}

			@Override
			protected void done() {
				setCursor(Cursor.getDefaultCursor());
				try {
					get();
					JOptionPane.showMessageDialog(MitUpdateDialog.this,
							"OpenRocket MIT edition will quit now and finish installing the update.",
							"MIT edition update",
							JOptionPane.INFORMATION_MESSAGE);
					System.exit(0);
				} catch (Exception ex) {
					log.warn("MIT edition update installation failed", ex);
					btnInstall.setEnabled(true);
					updateProgress("Update failed", 0);
					JOptionPane.showMessageDialog(MitUpdateDialog.this,
							errorMessage(ex),
							"MIT edition update failed",
							JOptionPane.ERROR_MESSAGE);
				}
			}
		};
		worker.execute();
	}

	private void updateProgress(String message, int percent) {
		progressLabel.setText(message);
		if (percent < 0) {
			progressBar.setIndeterminate(true);
			progressBar.setString(message);
		} else {
			progressBar.setIndeterminate(false);
			progressBar.setValue(percent);
			progressBar.setString(percent + "%");
		}
	}

	private static String buildReleaseHtml(MitReleaseInfo release) {
		StringBuilder sb = new StringBuilder();
		sb.append("<html>");
		String releaseNotes = release.getReleaseNotes();
		if (releaseNotes == null || releaseNotes.isBlank()) {
			sb.append("<p>No release notes were provided for this release.</p>");
		} else {
			sb.append(MarkdownUtil.toHtml(releaseNotes));
		}
		if (release.getReleaseURL() != null && !release.getReleaseURL().isBlank()) {
			sb.append("<br><br><a href='").append(release.getReleaseURL()).append("'>Open release page</a>");
		}
		sb.append("</html>");
		return sb.toString();
	}

	private static String errorMessage(Exception ex) {
		Throwable cause = ex.getCause() != null ? ex.getCause() : ex;
		String message = cause.getMessage();
		if (message == null || message.isBlank()) {
			message = cause.toString();
		}
		return "The MIT edition update could not be installed.\n\n" + message;
	}

	private static void initColors() {
		updateColors();
		UITheme.Theme.addUIThemeChangeListener(MitUpdateDialog::updateColors);
	}

	public static void updateColors() {
		textColor = GUIUtil.getUITheme().getTextColor();
	}
}
