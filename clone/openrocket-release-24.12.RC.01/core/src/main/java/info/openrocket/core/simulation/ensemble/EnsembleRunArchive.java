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

/** Compressed, disk-backed full flights. Saving embeds them in the .ork; no external sidecar is needed. */
public final class EnsembleRunArchive {
    private static final Cleaner CLEANER = Cleaner.create();
    private final Storage storage;
    private final List<EnsembleRunParameters> parameters;
    private final long compressedSize;

    private EnsembleRunArchive(Storage storage, List<EnsembleRunParameters> parameters) throws IOException {
        this.storage = storage;
        this.parameters = List.copyOf(parameters);
        compressedSize = Files.size(storage.path);
    }
    public int size() { return parameters.size(); }
    public long compressedSize() { return compressedSize; }
    public EnsembleRunParameters parameters(int index) { return parameters.get(index); }

    /** Load one full flight at a time, resolving its component references against the document. */
    public FlightData read(int index, OpenRocketDocument document) throws IOException {
        Objects.checkIndex(index, size());
        try (var zip = new ZipFile(storage.path.toFile())) {
            var entry = zip.getEntry(entryName(index));
            if (entry == null) throw new IOException("Missing archived ensemble run " + (index+1));
            try (var input = zip.getInputStream(entry)) { return EnsembleRunLoader.load(input, document); }
        }
    }
    public void writeTo(Writer output) throws IOException {
        try (var zip = new ZipFile(storage.path.toFile())) {
            for (int i = 0; i < size(); i++) {
                if (Thread.currentThread().isInterrupted()) throw new InterruptedIOException("Saving ensemble runs cancelled");
                var entry = zip.getEntry(entryName(i));
                if (entry == null) throw new IOException("Missing archived ensemble run " + (i+1));
                try (var input = new InputStreamReader(zip.getInputStream(entry), StandardCharsets.UTF_8)) {
                    input.transferTo(output);
                }
            }
        }
    }
    private static String entryName(int index) { return "run-" + (index+1) + ".xml"; }

    private static final class Storage {
        final Path path;
        final Cleaner.Cleanable cleanable;
        Storage() throws IOException {
            path = Files.createTempFile("openrocket-ensemble-", ".zip");
            path.toFile().deleteOnExit();
            cleanable = CLEANER.register(this, new Cleanup(path));
        }
    }
    private record Cleanup(Path path) implements Runnable {
        @Override public void run() {
            try { Files.deleteIfExists(path); } catch (IOException ignored) { /* Retry at JVM exit. */ }
        }
    }
    public static final class Builder implements AutoCloseable {
        private final Storage storage = new Storage();
        private final ZipOutputStream output;
        private final List<EnsembleRunParameters> parameters = new ArrayList<>();
        private boolean closed, transferred;
        public Builder() throws IOException {
            try { output = new ZipOutputStream(new BufferedOutputStream(Files.newOutputStream(storage.path))); }
            catch (IOException e) { storage.cleanable.clean(); throw e; }
        }
        public int size() { return parameters.size(); }
        public void add(EnsembleRunParameters inputs, FlightData flight) throws IOException {
            if (closed) throw new IllegalStateException("Archive already closed");
            if (inputs.number() != size()+1) throw new IllegalArgumentException("Missing, duplicate or unordered ensemble run");
            if (flight == null || flight.getBranchCount() == 0 || flight.getEnsembleResult() != null)
                throw new IllegalArgumentException("Expected a complete individual flight");
            output.putNextEntry(new ZipEntry(entryName(size())));
            OpenRocketSaver.writeEnsembleRun(output, inputs, flight);
            output.closeEntry();
            parameters.add(inputs);
        }
        public EnsembleRunArchive finish() throws IOException {
            if (closed) throw new IllegalStateException("Archive already closed");
            output.close(); closed = true;
            var archive = new EnsembleRunArchive(storage, parameters);
            transferred = true;
            return archive;
        }
        @Override public void close() throws IOException {
            try { if (!closed) { closed = true; output.close(); } }
            finally { if (!transferred) storage.cleanable.clean(); }
        }
    }
}
