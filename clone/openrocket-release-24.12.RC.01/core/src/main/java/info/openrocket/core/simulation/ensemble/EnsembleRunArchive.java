package info.openrocket.core.simulation.ensemble;

import info.openrocket.core.document.OpenRocketDocument;
import info.openrocket.core.file.openrocket.OpenRocketSaver;
import info.openrocket.core.file.openrocket.importt.EnsembleRunLoader;
import info.openrocket.core.simulation.FlightData;
import java.io.*;
import java.lang.ref.Cleaner;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import java.util.zip.*;

/**
 * Compressed, disk-backed full flights. Saving embeds them in the .ork; no external sidecar is
 * needed.
 */
public final class EnsembleRunArchive {
    private static final Cleaner CLEANER = Cleaner.create();
    private final Storage storage;
    private final List<EnsembleRunParameters> parameters;
    private final long compressedSize;

    /**
     * Creates a EnsembleRunArchive with the supplied dependencies and per-instance state.
     *
     * @param storage initial storage
     * @param parameters parameters
     * @throws IOException if the requested operation cannot be completed
     */
    private EnsembleRunArchive(Storage storage, List<EnsembleRunParameters> parameters)
            throws IOException {
        this.storage = storage;
        this.parameters = List.copyOf(parameters);
        compressedSize = Files.size(storage.path);
    }

    /**
     * Returns the number of member runs stored in this archive.
     *
     * @return the number of member runs stored in this archive
     */
    public int size() {
        return parameters.size();
    }

    /**
     * Returns the size of the compressed member-flight archive in bytes.
     *
     * @return the size of the compressed member-flight archive in bytes
     */
    public long compressedSize() {
        return compressedSize;
    }

    /**
     * Returns the saved random realization parameters for an archived member.
     *
     * @param index zero-based element index
     * @return the saved random realization parameters for an archived member
     */
    public EnsembleRunParameters parameters(int index) {
        return parameters.get(index);
    }

    /** Load one full flight at a time, resolving its component references against the document. */
    public FlightData read(int index, OpenRocketDocument document) throws IOException {
        Objects.checkIndex(index, size());
        try (var zip = new ZipFile(storage.path.toFile())) {
            var entry = zip.getEntry(entryName(index));
            if (entry == null)
                throw new IOException("Missing archived ensemble run " + (index + 1));
            try (var input = zip.getInputStream(entry)) {
                return EnsembleRunLoader.load(input, document);
            }
        }
    }

    /**
     * Copies the compressed member-flight archive to the supplied output stream.
     *
     * @param output output snapshot or destination
     * @throws IOException if the requested operation cannot be completed
     * @throws InterruptedIOException when rejected: Saving ensemble runs cancelled
     */
    public void writeTo(Writer output) throws IOException {
        try (var zip = new ZipFile(storage.path.toFile())) {
            for (int i = 0; i < size(); i++) {
                if (Thread.currentThread().isInterrupted())
                    throw new InterruptedIOException("Saving ensemble runs cancelled");
                var entry = zip.getEntry(entryName(i));
                if (entry == null)
                    throw new IOException("Missing archived ensemble run " + (i + 1));
                try (var input =
                        new InputStreamReader(zip.getInputStream(entry), StandardCharsets.UTF_8)) {
                    input.transferTo(output);
                }
            }
        }
    }

    /**
     * Builds the deterministic archive entry name for a member index.
     *
     * @param index zero-based element index
     * @return formatted or resolved text
     */
    private static String entryName(int index) {
        return "run-" + (index + 1) + ".xml";
    }

    private static final class Storage {
        final Path path;
        final Cleaner.Cleanable cleanable;

        /**
         * Creates a Storage value from the supplied .
         *
         * @throws IOException if the requested operation cannot be completed
         */
        Storage() throws IOException {
            path = Files.createTempFile("openrocket-ensemble-", ".zip");
            path.toFile().deleteOnExit();
            cleanable = CLEANER.register(this, new Cleanup(path));
        }
    }

    private record Cleanup(Path path) implements Runnable {
        /** Loads one archived member flight on demand. */
        @Override
        public void run() {
            try {
                Files.deleteIfExists(path);
            } catch (IOException ignored) {
                /* Retry at JVM exit. */
            }
        }
    }

    public static final class Builder implements AutoCloseable {
        private final Storage storage = new Storage();
        private final ZipOutputStream output;
        private final List<EnsembleRunParameters> parameters = new ArrayList<>();
        private boolean closed, transferred;

        /**
         * Creates a Builder value from the supplied .
         *
         * @throws IOException if the requested operation cannot be completed
         */
        public Builder() throws IOException {
            try {
                output =
                        new ZipOutputStream(
                                new BufferedOutputStream(Files.newOutputStream(storage.path)));
            } catch (IOException e) {
                storage.cleanable.clean();
                throw e;
            }
        }

        /**
         * Returns the number of member runs stored in this archive.
         *
         * @return the number of member runs stored in this archive
         */
        public int size() {
            return parameters.size();
        }

        /**
         * Serializes and compresses one member flight into the archive builder.
         *
         * @param inputs inputs
         * @param flight flight
         * @throws IOException if the requested operation cannot be completed
         * @throws IllegalStateException when rejected: Archive already closed
         * @throws IllegalArgumentException when rejected: Missing, duplicate or unordered ensemble
         *     run
         */
        public void add(EnsembleRunParameters inputs, FlightData flight) throws IOException {
            if (closed) throw new IllegalStateException("Archive already closed");
            if (inputs.number() != size() + 1)
                throw new IllegalArgumentException("Missing, duplicate or unordered ensemble run");
            if (flight == null
                    || flight.getBranchCount() == 0
                    || flight.getEnsembleResult() != null)
                throw new IllegalArgumentException("Expected a complete individual flight");
            output.putNextEntry(new ZipEntry(entryName(size())));
            OpenRocketSaver.writeEnsembleRun(output, inputs, flight);
            output.closeEntry();
            parameters.add(inputs);
        }

        /**
         * Finalizes the compressed archive and returns a reader for the completed members.
         *
         * @return the resulting EnsembleRunArchive
         * @throws IOException if the requested operation cannot be completed
         * @throws IllegalStateException when rejected: Archive already closed
         */
        public EnsembleRunArchive finish() throws IOException {
            if (closed) throw new IllegalStateException("Archive already closed");
            output.close();
            closed = true;
            var archive = new EnsembleRunArchive(storage, parameters);
            transferred = true;
            return archive;
        }

        /**
         * Closes archive-builder resources after completion or failure.
         *
         * @throws IOException if the requested operation cannot be completed
         */
        @Override
        public void close() throws IOException {
            try {
                if (!closed) {
                    closed = true;
                    output.close();
                }
            } finally {
                if (!transferred) storage.cleanable.clean();
            }
        }
    }
}
