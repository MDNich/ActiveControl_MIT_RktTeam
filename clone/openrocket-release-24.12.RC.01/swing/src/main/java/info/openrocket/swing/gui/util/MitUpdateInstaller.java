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
import java.security.DigestInputStream;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
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

	private MitUpdateInstaller() {
	}

	public static void downloadVerifyAndLaunchInstaller(MitUpdateInfo info) throws UpdateInstallException {
		if (info == null || !info.isUpdateAvailable()) {
			throw new UpdateInstallException("No MIT edition update is available");
		}

		MitReleaseInfo.Asset jarAsset = info.getJarAsset();
		MitReleaseInfo.Asset sha256Asset = info.getSha256Asset();
		if (jarAsset == null || sha256Asset == null) {
			throw new UpdateInstallException("MIT edition update is missing jar or SHA-256 asset");
		}

		Path tempDir;
		try {
			tempDir = Files.createTempDirectory("openrocket-mit-update-");
		} catch (IOException e) {
			throw new UpdateInstallException("Could not create temporary update directory", e);
		}

		Path downloadedJar = tempDir.resolve(sanitizeFilename(jarAsset.name()));
		Path downloadedSha256 = tempDir.resolve(sanitizeFilename(sha256Asset.name()));
		downloadAsset(jarAsset.browserDownloadUrl(), downloadedJar);
		downloadAsset(sha256Asset.browserDownloadUrl(), downloadedSha256);

		String expectedSha256 = readExpectedSha256(downloadedSha256);
		String actualSha256 = sha256(downloadedJar);
		if (!expectedSha256.equalsIgnoreCase(actualSha256)) {
			throw new UpdateInstallException(String.format(
					"MIT edition update checksum mismatch. Expected %s but downloaded %s.",
					expectedSha256, actualSha256));
		}

		Path currentJar = getCurrentJarPath();
		if (SystemInfo.getPlatform() == SystemInfo.Platform.WINDOWS) {
			throw new UpdateInstallException("Automatic MIT edition jar replacement is not implemented for Windows");
		}
		if (currentJar.getParent() == null || !Files.isWritable(currentJar.getParent())) {
			throw new UpdateInstallException("Current jar directory is not writable: " + currentJar.getParent());
		}

		launchReplacementScript(downloadedJar, currentJar);
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

	private static void downloadAsset(String url, Path destination) throws UpdateInstallException {
		HttpURLConnection connection = null;
		try {
			connection = (HttpURLConnection) new URL(url).openConnection();
			connection.setRequestMethod("GET");
			connection.setUseCaches(false);
			connection.setAllowUserInteraction(false);
			connection.setConnectTimeout(CONNECTION_TIMEOUT);
			connection.setReadTimeout(CONNECTION_TIMEOUT);
			connection.connect();

			int status = connection.getResponseCode();
			if (status != HttpURLConnection.HTTP_OK) {
				throw new UpdateInstallException("Download failed with HTTP status " + status + ": " + url);
			}

			try (InputStream in = new BufferedInputStream(connection.getInputStream())) {
				Files.copy(in, destination, StandardCopyOption.REPLACE_EXISTING);
			}
		} catch (IOException e) {
			throw new UpdateInstallException("Could not download MIT edition update asset: " + url, e);
		} finally {
			if (connection != null) {
				connection.disconnect();
			}
		}
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

				cp "$TARGET" "$BACKUP" 2>/dev/null || true
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

	public static class UpdateInstallException extends Exception {
		public UpdateInstallException(String message) {
			super(message);
		}

		public UpdateInstallException(String message, Throwable cause) {
			super(message, cause);
		}
	}
}
