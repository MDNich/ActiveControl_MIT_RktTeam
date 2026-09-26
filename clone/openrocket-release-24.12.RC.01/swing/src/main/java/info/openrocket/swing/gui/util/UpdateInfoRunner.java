package info.openrocket.swing.gui.util;

import info.openrocket.core.communication.ReleaseInfo;
import info.openrocket.core.communication.MitUpdateInfo;
import info.openrocket.core.communication.MitUpdateInfoRetriever;
import info.openrocket.core.communication.UpdateInfo;
import info.openrocket.core.communication.UpdateInfoRetriever;
import info.openrocket.core.l10n.Translator;
import info.openrocket.core.startup.Application;
import info.openrocket.core.util.BuildProperties;
import info.openrocket.swing.gui.dialogs.UpdateInfoDialog;
import info.openrocket.swing.gui.dialogs.MitUpdateDialog;
import net.miginfocom.swing.MigLayout;

import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.SwingWorker;
import java.awt.Dialog;
import java.awt.Window;

/**
 * Helper class for checking for updates.
 *
 * @author Sibo Van Gool <sibo.vangool@hotmail.com>
 */
public abstract class UpdateInfoRunner {
	private static final Translator trans = Application.getTranslator();
	private static final SwingPreferences preferences = (SwingPreferences) Application.getPreferences();

	public static void checkForUpdates(Window parent) {
		if (BuildProperties.isMitEdition()) {
			checkForMitUpdates(parent);
			return;
		}
		final UpdateInfoRetriever retriever = new UpdateInfoRetriever();
		retriever.startFetchUpdateInfo();

		final JDialog dialog1 = new JDialog(parent, Dialog.ModalityType.MODELESS); // Make non-modal
		JPanel panel = new JPanel(new MigLayout());

		panel.add(new JLabel(trans.get("pref.dlg.lbl.Checkingupdates")), "wrap");

		JProgressBar bar = new JProgressBar();
		bar.setIndeterminate(true);
		panel.add(bar, "growx, wrap para");

		JButton cancel = new JButton(trans.get("dlg.but.cancel"));
		cancel.addActionListener(e -> {
			retriever.cancel(); // Add way to cancel retriever
			dialog1.dispose();
		});
		panel.add(cancel, "right");
		dialog1.add(panel);

		GUIUtil.setDisposableDialogOptions(dialog1, cancel);

		SwingWorker<UpdateInfo, Void> worker = new SwingWorker<>() {
			@Override
			protected UpdateInfo doInBackground() {
				long startTime = System.currentTimeMillis();
				while (retriever.isRunning() && System.currentTimeMillis() - startTime < 10000) {
					try {
						Thread.sleep(100);
					} catch (InterruptedException e) {
						break;
					}
				}
				return retriever.getUpdateInfo();
			}

			@Override
			protected void done() {
				dialog1.dispose();
				try {
					handleUpdateResult(parent, get(), retriever);
				} catch (Exception e) {
					handleError(parent, e);
				}
			}
		};

		worker.execute();
		dialog1.setVisible(true);
	}

	private static void checkForMitUpdates(Window parent) {
		MitUpdateInfoRetriever retriever = new MitUpdateInfoRetriever();
		retriever.startFetchUpdateInfo();
		JDialog progress = new JDialog(parent, "Checking for MIT edition updates", Dialog.ModalityType.MODELESS);
		JPanel panel = new JPanel(new MigLayout("fill"));
		panel.add(new JLabel("Checking the MIT edition's GitHub releases..."), "wrap para");
		JProgressBar bar = new JProgressBar();
		bar.setIndeterminate(true);
		panel.add(bar, "growx, wrap para");
		JButton cancel = new JButton(trans.get("dlg.but.cancel"));
		panel.add(cancel, "right");
		progress.add(panel);
		GUIUtil.setDisposableDialogOptions(progress, cancel);
		SwingWorker<MitUpdateInfo, Void> worker = new SwingWorker<>() {
			@Override
			protected MitUpdateInfo doInBackground() throws InterruptedException {
				long deadline = System.nanoTime() + java.util.concurrent.TimeUnit.SECONDS.toNanos(60);
				while (retriever.isRunning() && System.nanoTime() < deadline && !isCancelled()) {
					Thread.sleep(100);
				}
				if (retriever.isRunning()) {
					retriever.cancel();
					return new MitUpdateInfo(new java.io.IOException("The MIT update check timed out."));
				}
				return retriever.getUpdateInfo();
			}

			@Override
			protected void done() {
				boolean dismissed = !progress.isDisplayable();
				progress.dispose();
				if (isCancelled() || dismissed) {
					return;
				}
				try {
					MitUpdateInfo info = get();
					if (info == null) {
						throw new java.io.IOException("No MIT update information was received.");
					}
					if (info.getException() != null) {
						throw info.getException();
					}
					if (info.isUpdateAvailable()) {
						// A manual check includes versions previously skipped at startup.
						new MitUpdateDialog(info).setVisible(true);
					} else {
						JOptionPane.showMessageDialog(parent, "OpenRocket MIT " + BuildProperties.getMitVersion()
								+ " is up to date.", "MIT edition updates", JOptionPane.INFORMATION_MESSAGE);
					}
				} catch (Exception e) {
					JOptionPane.showMessageDialog(parent, "Could not check the MIT GitHub releases.\n\n" + e.getMessage(),
							"MIT edition updates", JOptionPane.WARNING_MESSAGE);
				}
			}
		};
		cancel.addActionListener(e -> {
			retriever.cancel();
			worker.cancel(true);
			progress.dispose();
		});
		progress.addWindowListener(new java.awt.event.WindowAdapter() {
			@Override
			public void windowClosed(java.awt.event.WindowEvent e) {
				if (!worker.isDone()) {
					retriever.cancel();
					worker.cancel(true);
				}
			}
		});
		progress.pack();
		progress.setLocationRelativeTo(parent);
		worker.execute();
		progress.setVisible(true);
	}

	private static void handleUpdateResult(Window parent, UpdateInfo info, UpdateInfoRetriever retriever) {
		if (info == null) {
			if (!retriever.isCancelled()) {
				JOptionPane.showMessageDialog(parent,
						trans.get("update.dlg.error"),
						trans.get("update.dlg.error.title"),
						JOptionPane.WARNING_MESSAGE);
			}
			return;
		}

		if (info.getException() != null) {
			JOptionPane.showMessageDialog(parent,
					info.getException().getMessage(),
					trans.get("update.dlg.exception.title"),
					JOptionPane.WARNING_MESSAGE);
			return;
		}

		ReleaseInfo release = info.getLatestRelease();
		// Skip if version is in ignore list
		boolean checkAllUpdates = System.getProperty("openrocket.debug.checkAllVersionUpdates") != null;
		if (!checkAllUpdates && preferences.getIgnoreUpdateVersions().contains(release.getReleaseName())) {
			return;
		}

		switch (info.getReleaseStatus()) {
			case LATEST:
				JOptionPane.showMessageDialog(parent,
						String.format(trans.get("update.dlg.latestVersion"),
								BuildProperties.getVersion()),
						trans.get("update.dlg.latestVersion.title"),
						JOptionPane.INFORMATION_MESSAGE);
				break;
			case NEWER:
				JOptionPane.showMessageDialog(parent,
						String.format("<html><body><p style='width: %dpx'>%s", 400,
								String.format(trans.get("update.dlg.newerVersion"),
										BuildProperties.getVersion(), release.getReleaseName())),
						trans.get("update.dlg.newerVersion.title"),
						JOptionPane.INFORMATION_MESSAGE);
				break;
			case OLDER:
				new UpdateInfoDialog(info).setVisible(true);
				break;
		}
	}

	private static void handleError(Window parent, Exception e) {
		JOptionPane.showMessageDialog(parent,
				trans.get("update.dlg.error"),
				trans.get("update.dlg.error.title"),
				JOptionPane.WARNING_MESSAGE);
	}
}
