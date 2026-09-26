package info.openrocket.swing.gui.util;

import java.io.BufferedInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.nio.file.StandardOpenOption;
import java.security.DigestInputStream;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.HexFormat;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import info.openrocket.core.arch.SystemInfo;
import info.openrocket.core.communication.MitReleaseInfo;
import info.openrocket.core.communication.MitUpdateInfo;
import info.openrocket.core.util.JarUtil;

/**
 * Downloads and stages MIT-edition jar updates.
 */
public final class MitUpdateInstaller {
	private static final Pattern SHA256_PATTERN = Pattern.compile("\\b[0-9a-fA-F]{64}\\b");
	private static final int CONNECTION_TIMEOUT = 10000;
	private static final DateTimeFormatter LOG_TIMESTAMP = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss.SSS");

	private MitUpdateInstaller() {
	}

	public static void downloadVerifyAndLaunchInstaller(MitUpdateInfo info) throws UpdateInstallException {
		downloadVerifyAndLaunchInstaller(info, ProgressListener.NONE);
	}

	public static void downloadVerifyAndLaunchInstaller(MitUpdateInfo info, ProgressListener progressListener)
			throws UpdateInstallException {
		ProgressListener progress = progressListener != null ? progressListener : ProgressListener.NONE;
		Path logPath = getUpdateLogPath();
		logInstallerMessage(logPath, "Starting MIT edition update install");
		if (info == null || !info.isUpdateAvailable()) {
			logInstallerMessage(logPath, "No update is available");
			throw new UpdateInstallException("No MIT edition update is available");
		}

		MitReleaseInfo.Asset jarAsset = info.getJarAsset();
		MitReleaseInfo.Asset sha256Asset = info.getSha256Asset();
		if (jarAsset == null) {
			logInstallerMessage(logPath, "Release metadata is missing jar asset");
			throw new UpdateInstallException("MIT edition update is missing jar asset");
		}
		boolean needsSha256Asset = info.getExpectedSha256() == null || info.getExpectedSha256().isBlank();
		if (needsSha256Asset && sha256Asset == null) {
			logInstallerMessage(logPath, "Release metadata is missing SHA-256 asset");
			throw new UpdateInstallException("MIT edition update is missing SHA-256 asset");
		}

		Path tempDir;
		try {
			tempDir = Files.createTempDirectory("openrocket-mit-update-");
			logInstallerMessage(logPath, "Created temp directory: " + tempDir);
		} catch (IOException e) {
			logInstallerMessage(logPath, "Could not create temporary update directory: " + e.getMessage());
			throw new UpdateInstallException("Could not create temporary update directory", e);
		}

		Path downloadedJar = tempDir.resolve(sanitizeFilename(jarAsset.name()));
		logInstallerMessage(logPath, "Downloading jar asset " + jarAsset.name() + " to " + downloadedJar);
		downloadAsset(jarAsset.browserDownloadUrl(), downloadedJar, progress, "Downloading update", 0, 90);
		logInstallerMessage(logPath, "Downloaded jar asset. Size: " + fileSize(downloadedJar));
		String expectedSha256 = info.getExpectedSha256();
		if (needsSha256Asset) {
			Path downloadedSha256 = tempDir.resolve(sanitizeFilename(sha256Asset.name()));
			logInstallerMessage(logPath, "Downloading checksum asset " + sha256Asset.name() + " to " + downloadedSha256);
			downloadAsset(sha256Asset.browserDownloadUrl(), downloadedSha256, progress, "Downloading checksum", 90, 95);
			expectedSha256 = readExpectedSha256(downloadedSha256);
			logInstallerMessage(logPath, "Read checksum from checksum asset: " + expectedSha256);
		} else {
			progress.onProgress("Using release checksum", 95);
			logInstallerMessage(logPath, "Using checksum from release metadata: " + expectedSha256);
		}

		progress.onProgress("Verifying checksum", 96);
		String actualSha256 = sha256(downloadedJar);
		logInstallerMessage(logPath, "Actual downloaded jar checksum: " + actualSha256);
		if (!expectedSha256.equalsIgnoreCase(actualSha256)) {
			logInstallerMessage(logPath, "Checksum mismatch. Expected " + expectedSha256 + " but got " + actualSha256);
			throw new UpdateInstallException(String.format(
					"MIT edition update checksum mismatch. Expected %s but downloaded %s.",
					expectedSha256, actualSha256));
		}
		logInstallerMessage(logPath, "Checksum verified");

		Path currentJar = getCurrentJarPath();
		logInstallerMessage(logPath, "Current jar path: " + currentJar);
		progress.onProgress("Preparing installer", 99);
		if (SystemInfo.getPlatform() == SystemInfo.Platform.WINDOWS) {
			WindowsUpdateHelper.launch(downloadedJar, currentJar, logPath, actualSha256);
		} else {
			if (currentJar.getParent() == null || !Files.isWritable(currentJar.getParent())) {
				logInstallerMessage(logPath, "Current jar directory is not writable: " + currentJar.getParent());
				throw new UpdateInstallException("Current jar directory is not writable: " + currentJar.getParent());
			}
			logInstallerMessage(logPath, "Launching Unix/macOS replacement script");
			launchReplacementScript(downloadedJar, currentJar);
		}
		logInstallerMessage(logPath, "Installer helper launched");
		progress.onProgress("Ready to install", 100);
	}

	private static Path getCurrentJarPath() throws UpdateInstallException {
		if (JarUtil.getCurrentJarFile() == null) {
			throw new UpdateInstallException("Could not locate the currently running jar. Automatic update requires a packaged jar/app run.");
		}
		Path path = JarUtil.getCurrentJarFile().toPath().toAbsolutePath().normalize();
		if (!Files.isRegularFile(path)) {
			throw new UpdateInstallException("Current jar path is not a regular file: " + path);
		}
		return path;
	}

	private static void downloadAsset(String url, Path destination, ProgressListener progress, String status,
			int startPercent, int endPercent) throws UpdateInstallException {
		HttpURLConnection connection = null;
		try {
			connection = (HttpURLConnection) new URL(url).openConnection();
			connection.setRequestMethod("GET");
			connection.setUseCaches(false);
			connection.setAllowUserInteraction(false);
			connection.setConnectTimeout(CONNECTION_TIMEOUT);
			connection.setReadTimeout(CONNECTION_TIMEOUT);
			connection.connect();

			int statusCode = connection.getResponseCode();
			if (statusCode != HttpURLConnection.HTTP_OK) {
				throw new UpdateInstallException("Download failed with HTTP status " + statusCode + ": " + url);
			}

			long contentLength = connection.getContentLengthLong();
			progress.onProgress(status, contentLength > 0 ? startPercent : -1);
			try (InputStream in = new BufferedInputStream(connection.getInputStream())) {
				copyWithProgress(in, destination, contentLength, progress, status, startPercent, endPercent);
			}
		} catch (IOException e) {
			throw new UpdateInstallException("Could not download MIT edition update asset: " + url, e);
		} finally {
			if (connection != null) {
				connection.disconnect();
			}
		}
	}

	private static void copyWithProgress(InputStream in, Path destination, long contentLength, ProgressListener progress,
			String status, int startPercent, int endPercent) throws IOException {
		Path tempDestination = destination.resolveSibling(destination.getFileName() + ".part");
		long totalRead = 0;
		int lastProgress = -1;
		try (var out = Files.newOutputStream(tempDestination)) {
			byte[] buffer = new byte[8192];
			int read;
			while ((read = in.read(buffer)) != -1) {
				out.write(buffer, 0, read);
				totalRead += read;
				if (contentLength > 0) {
					int currentProgress = startPercent +
							(int) Math.min(endPercent - startPercent,
									((totalRead * (endPercent - startPercent)) / contentLength));
					if (currentProgress != lastProgress) {
						progress.onProgress(status, currentProgress);
						lastProgress = currentProgress;
					}
				}
			}
		}
		Files.move(tempDestination, destination, StandardCopyOption.REPLACE_EXISTING);
		progress.onProgress(status, endPercent);
	}

	private static String readExpectedSha256(Path sha256File) throws UpdateInstallException {
		try {
			String text = Files.readString(sha256File, StandardCharsets.UTF_8);
			Matcher matcher = SHA256_PATTERN.matcher(text);
			if (!matcher.find()) {
				throw new UpdateInstallException("SHA-256 asset does not contain a 64-character checksum");
			}
			return matcher.group().toLowerCase(Locale.ROOT);
		} catch (IOException e) {
			throw new UpdateInstallException("Could not read SHA-256 asset", e);
		}
	}

	private static String sha256(Path file) throws UpdateInstallException {
		try {
			MessageDigest digest = MessageDigest.getInstance("SHA-256");
			try (InputStream in = Files.newInputStream(file);
				 DigestInputStream digestInputStream = new DigestInputStream(in, digest)) {
				byte[] buffer = new byte[8192];
				while (digestInputStream.read(buffer) != -1) {
					// Drain stream through digest.
				}
			}
			return HexFormat.of().formatHex(digest.digest());
		} catch (IOException e) {
			throw new UpdateInstallException("Could not compute downloaded jar checksum", e);
		} catch (NoSuchAlgorithmException e) {
			throw new UpdateInstallException("SHA-256 is not available in this Java runtime", e);
		}
	}

	private static void launchReplacementScript(Path downloadedJar, Path currentJar) throws UpdateInstallException {
		Path appBundle = findAppBundle(currentJar);
		String script = """
				#!/bin/sh
				set -eu

				PID="$1"
				SOURCE="$2"
				TARGET="$3"
				APP_BUNDLE="$4"
				BACKUP="${TARGET}.bak"

				while kill -0 "$PID" 2>/dev/null; do
					sleep 1
				done

				rm -f "$BACKUP"
				cp -p "$TARGET" "$BACKUP" 2>/dev/null || true
				rm -f "$TARGET"
				cp "$SOURCE" "$TARGET"
				rm -f "$SOURCE"

				if [ -n "$APP_BUNDLE" ]; then
					open "$APP_BUNDLE"
				else
					java -jar "$TARGET" >/dev/null 2>&1 &
				fi

				rm -f "$0"
				""";

		try {
			Path scriptPath = Files.createTempFile("openrocket-mit-update-", ".sh");
			Files.writeString(scriptPath, script, StandardCharsets.UTF_8);
			scriptPath.toFile().setExecutable(true);

			String pid = Long.toString(ProcessHandle.current().pid());
			String appBundleArg = appBundle != null ? appBundle.toString() : "";
			new ProcessBuilder("sh", scriptPath.toString(), pid, downloadedJar.toString(), currentJar.toString(), appBundleArg)
					.redirectErrorStream(true)
					.start();
		} catch (IOException e) {
			throw new UpdateInstallException("Could not launch MIT edition updater helper script", e);
		}
	}


	private static Path findAppBundle(Path path) {
		Path current = path.toAbsolutePath().normalize();
		while (current != null) {
			Path fileName = current.getFileName();
			if (fileName != null && fileName.toString().endsWith(".app")) {
				return current;
			}
			current = current.getParent();
		}
		return null;
	}

	private static String sanitizeFilename(String name) {
		return name.replaceAll("[^A-Za-z0-9._-]", "_");
	}


	private static Path getUpdateLogPath() {
		Path home = Path.of(System.getProperty("user.home", "."));
		Path downloads = home.resolve("Downloads");
		try {
			Files.createDirectories(downloads);
			return downloads.resolve("openrocket-mit-update.log");
		} catch (IOException e) {
			return home.resolve("openrocket-mit-update.log");
		}
	}

	private static void logInstallerMessage(Path logPath, String message) {
		try {
			Files.writeString(logPath,
					"[" + LocalDateTime.now().format(LOG_TIMESTAMP) + "] [java] " + message + System.lineSeparator(),
					StandardCharsets.UTF_8,
					StandardOpenOption.CREATE, StandardOpenOption.APPEND);
		} catch (IOException e) {
			// Update logging must not prevent installation.
		}
	}

	private static String fileSize(Path file) {
		try {
			return Long.toString(Files.size(file));
		} catch (IOException e) {
			return "<unknown>";
		}
	}

	public static class UpdateInstallException extends Exception {
		public UpdateInstallException(String message) {
			super(message);
		}

		public UpdateInstallException(String message, Throwable cause) {
			super(message, cause);
		}
	}

	public interface ProgressListener {
		ProgressListener NONE = (message, percent) -> {
		};

		/**
		 * @param message progress status
		 * @param percent progress from 0 to 100, or -1 for indeterminate progress
		 */
		void onProgress(String message, int percent);
	}
}
