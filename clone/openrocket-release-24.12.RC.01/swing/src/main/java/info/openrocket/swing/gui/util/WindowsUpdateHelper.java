package info.openrocket.swing.gui.util;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Base64;
import java.util.concurrent.TimeUnit;

import info.openrocket.swing.gui.util.MitUpdateInstaller.UpdateInstallException;

/** Stages the Windows helper and waits until it is ready before allowing the app to quit. */
final class WindowsUpdateHelper {

	private WindowsUpdateHelper() {
	}

	static void launch(Path source, Path target, Path logFile, String checksum) throws UpdateInstallException {
		Path state = null;
		try {
			Path directory = source.getParent();
			Path script = directory.resolve("install-update.ps1");
			state = directory.resolve("installer-state.txt");
			try (InputStream resource = WindowsUpdateHelper.class.getResourceAsStream("/updates/windows-update.ps1")) {
				if (resource == null) {
					throw new IOException("Windows update helper is missing from the application");
				}
				Files.copy(resource, script);
			}
			Path launcher = findLauncher(target);
			Path java = findJava(Path.of(System.getProperty("java.home")));
			String command = "& " + quote(script.toString())
					+ " -ProcessIdToWait " + ProcessHandle.current().pid()
					+ " -Source " + quote(source.toString())
					+ " -Target " + quote(target.toString())
					+ " -Launcher " + quote(launcher == null ? "" : launcher.toString())
					+ " -JavaPath " + quote(java.toString())
					+ " -LogFile " + quote(logFile.toString())
					+ " -StateFile " + quote(state.toString())
					+ " -ExpectedSha256 " + quote(checksum)
					+ (requiresElevation(target) ? " -Elevate" : "");
			String systemRoot = System.getenv("SystemRoot");
			Path powershell = Path.of(systemRoot == null ? "C:\\Windows" : systemRoot,
					"System32", "WindowsPowerShell", "v1.0", "powershell.exe");
			Process broker = new ProcessBuilder(powershell.toString(), "-NoProfile", "-NonInteractive",
					"-ExecutionPolicy", "Bypass", "-EncodedCommand", encode(command))
					.redirectErrorStream(true)
					.redirectOutput(directory.resolve("helper-output.log").toFile()).start();
			long deadline = System.nanoTime() + TimeUnit.MINUTES.toNanos(3);
			while (System.nanoTime() < deadline) {
				String status = Files.exists(state) ? Files.readString(state, StandardCharsets.UTF_8).trim() : "";
				if (status.equals("READY")) {
					return;
				}
				if (status.startsWith("FAILED:")) {
					throw new IOException(status.substring("FAILED:".length()).trim());
				}
				if (!broker.isAlive()) {
					throw new IOException("Windows update helper exited before becoming ready. See " + logFile);
				}
				Thread.sleep(100);
			}
			throw new IOException("Windows update helper did not become ready. See " + logFile);
		} catch (IOException | InterruptedException e) {
			// A late UAC response must never leave an update armed after this method fails.
			if (state != null) {
				try {
					Files.writeString(Path.of(state + ".cancel"), "cancel", StandardCharsets.UTF_8);
				} catch (IOException ignored) {
				}
			}
			if (e instanceof InterruptedException) {
				Thread.currentThread().interrupt();
			}
			throw new UpdateInstallException("Could not prepare the Windows update: " + e.getMessage(), e);
		}
	}

	static Path findLauncher(Path jar) {
		// jpackage uses app/OpenRocket.jar; older install4j bundles put it in lib/ or the root.
		Path directory = jar.toAbsolutePath().normalize().getParent();
		for (int level = 0; directory != null && level < 3; level++, directory = directory.getParent()) {
			for (String name : new String[] { "OpenRocket MIT.exe", "OpenRocket_MIT.exe", "OpenRocket.exe" }) {
				Path launcher = directory.resolve(name);
				if (Files.isRegularFile(launcher)) {
					return launcher;
				}
			}
		}
		return null;
	}

	static Path findJava(Path javaHome) throws IOException {
		for (String name : new String[] { "javaw.exe", "java.exe" }) {
			Path java = javaHome.resolve("bin").resolve(name);
			if (Files.isRegularFile(java)) {
				return java;
			}
		}
		throw new IOException("Cannot find the current Windows Java runtime in " + javaHome);
	}

	static boolean requiresElevation(Path target) {
		if (!Files.isWritable(target)) {
			return true;
		}
		try {
			Path probe = Files.createTempFile(target.getParent(), ".openrocket-update-", ".tmp");
			Files.delete(probe);
			return false;
		} catch (IOException e) {
			return true;
		}
	}

	static String quote(String value) {
		return "'" + value.replace("'", "''") + "'";
	}

	static String encode(String command) {
		return Base64.getEncoder().encodeToString(command.getBytes(StandardCharsets.UTF_16LE));
	}
}
