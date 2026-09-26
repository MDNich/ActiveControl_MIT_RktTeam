package info.openrocket.swing.gui.util;

import static org.junit.jupiter.api.Assertions.*;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Base64;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class WindowsUpdateHelperTest {

	@TempDir
	Path temp;

	@Test
	void findsMitLauncherBeforeLegacyAliasWithSpacesAndUnicode() throws Exception {
		Path app = Files.createDirectories(temp.resolve("D'Angelo & MIT – rocket/app"));
		Path jar = Files.writeString(app.resolve("OpenRocket.jar"), "test");
		Files.createFile(app.getParent().resolve("OpenRocket.exe"));
		Path expected = Files.createFile(app.getParent().resolve("OpenRocket MIT.exe"));
		assertEquals(expected, WindowsUpdateHelper.findLauncher(jar));
		assertFalse(WindowsUpdateHelper.requiresElevation(jar));
	}

	@Test
	void standaloneJarUsesCurrentJavaRatherThanPath() throws Exception {
		Path jar = Files.writeString(temp.resolve("OpenRocket MIT.jar"), "test");
		assertNull(WindowsUpdateHelper.findLauncher(jar));
		Path bin = Files.createDirectories(temp.resolve("bundled runtime/bin"));
		Path console = Files.createFile(bin.resolve("java.exe"));
		assertEquals(console, WindowsUpdateHelper.findJava(bin.getParent()));
		Path gui = Files.createFile(bin.resolve("javaw.exe"));
		assertEquals(gui, WindowsUpdateHelper.findJava(bin.getParent()));
	}

	@Test
	void encodedCommandPreservesLiteralWindowsPaths() {
		String path = "C:\\Users\\D'Angelo & $(echo wrong)\\é\\OpenRocket MIT.exe";
		String quoted = WindowsUpdateHelper.quote(path);
		assertEquals("'C:\\Users\\D''Angelo & $(echo wrong)\\é\\OpenRocket MIT.exe'", quoted);
		assertEquals(quoted, new String(Base64.getDecoder().decode(WindowsUpdateHelper.encode(quoted)),
				StandardCharsets.UTF_16LE));
	}
}
