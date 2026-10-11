package info.openrocket.swing.gui.flightcomputer;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;

/** An editing copy, never an executable input or a replacement for the saved .fc source. */
final class JavaBoardExternalSource {
    private final Path file;
    private String synchronizedText;

    /**
     * Builds the java board external source and connects its controls to the supplied model.
     *
     * @param directory directory
     * @param source source text, model or measured-channel selection
     * @throws IOException if the requested operation cannot be completed
     */
    JavaBoardExternalSource(Path directory, String source) throws IOException {
        Files.createDirectories(directory);
        file = Files.createTempDirectory(directory, "board-").resolve("BoardProgram.java");
        write(source);
    }

    /**
     * Returns the per-editor temporary Java working-copy path.
     *
     * @return the per-editor temporary Java working-copy path
     */
    Path file() {
        return file;
    }

    /**
     * Records the editor's current source revision for external-change detection.
     *
     * @param source source text, model or measured-channel selection
     * @return true when the condition described above holds
     */
    boolean editorChanged(String source) {
        return !source.equals(synchronizedText);
    }

    /**
     * Tests whether the external working copy changed since the last acknowledged revision.
     *
     * @return true when the condition described above holds
     * @throws IOException if the requested operation cannot be completed
     */
    boolean externallyChanged() throws IOException {
        return !read().equals(synchronizedText);
    }

    /**
     * Reads the current external Java working copy as UTF-8.
     *
     * @return formatted or resolved text
     * @throws IOException if the requested operation cannot be completed
     */
    String read() throws IOException {
        if (Files.size(file) > 400_000)
            throw new IOException("External source exceeds 100,000 characters");
        String text = Files.readString(file, StandardCharsets.UTF_8);
        check(text);
        return text;
    }

    /**
     * Marks the current external revision as synchronized with the in-app editor.
     *
     * @param source source text, model or measured-channel selection
     */
    void acknowledge(String source) {
        synchronizedText = source;
    }

    /**
     * Writes the current editor source to the external working copy.
     *
     * @param source source text, model or measured-channel selection
     * @throws IOException if the requested operation cannot be completed
     */
    void write(String source) throws IOException {
        check(source);
        Files.writeString(file, source, StandardCharsets.UTF_8);
        synchronizedText = source;
    }

    /**
     * Rejects conflicting local and external edits before synchronizing the working copy.
     *
     * @param source source text, model or measured-channel selection
     * @throws IOException if the requested operation cannot be completed
     */
    private static void check(String source) throws IOException {
        if (source.length() > 100_000)
            throw new IOException("External source exceeds 100,000 characters");
    }
}
