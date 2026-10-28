package info.openrocket.core.communication;

import info.openrocket.core.logging.WarningSet;
import info.openrocket.core.file.simplesax.SimpleSAX;
import info.openrocket.core.util.BuildProperties;
import org.xml.sax.InputSource;
import org.xml.sax.SAXException;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.Files;
import java.nio.charset.StandardCharsets;

public abstract class WelcomeInfoRetriever {
    private static final String RELEASE_NOTES_FILENAME = "ReleaseNotes.md";

    /**
     * Retrieves the release notes of the current build version from the
     * ReleaseNotes.md file.
     *
     * @param version the build version to search the release notes for (e.g. "22.02")
     * @return the release notes of the current build version
     * @throws IOException if the file could not be read
     */
    public static String retrieveWelcomeInfo(String version) throws IOException {
        // First, try to load from resources (for packaged application)
        InputStream inputStream = WelcomeInfoRetriever.class.getClassLoader().getResourceAsStream(RELEASE_NOTES_FILENAME);

        // If not found in resources, try to load from project root directory
        if (inputStream == null) {
            inputStream = openProjectReleaseNotes(Paths.get("").toAbsolutePath());
        }

        try (InputStreamReader reader = new InputStreamReader(inputStream, StandardCharsets.UTF_8)) {
            InputSource source = new InputSource(reader);
            ReleaseNotesHandler handler = new ReleaseNotesHandler(version);
            WarningSet warnings = new WarningSet();

            SimpleSAX.readXML(source, handler, warnings);
            return handler.getReleaseNotes();
        } catch (SAXException e) {
            throw new IOException("Failed to parse release notes: " + e.getMessage(), e);
        }
    }

    static InputStream openProjectReleaseNotes(Path currentPath) throws IOException {
        Path file = currentPath.resolve(RELEASE_NOTES_FILENAME);
        // Finder can start the application at the filesystem root, which has no parent.
        if (!Files.isRegularFile(file) && currentPath.getParent() != null) {
            file = currentPath.getParent().resolve(RELEASE_NOTES_FILENAME);
        }
        if (!Files.isRegularFile(file)) {
            throw new FileNotFoundException("ReleaseNotes.md not found in resources or project root: " + file);
        }
        return Files.newInputStream(file);
    }

    /**
     * Retrieves the release notes of the current build version from the
     * ReleaseNotes.md file.
     * 
     * @return the release notes of the current build version
     * @throws IOException if the file could not be read
     */
    public static String retrieveWelcomeInfo() throws IOException {
        return retrieveWelcomeInfo(BuildProperties.getVersion());
    }
}
