package info.openrocket.core.simulation.mission;

import edu.mit.rocket_team.RTFCVerificationRocket;
import info.openrocket.core.util.BaseTestCase;
import jakarta.json.*;
import org.apache.commons.compress.archivers.zip.*;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import java.util.zip.*;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import static org.junit.jupiter.api.Assertions.*;

class MissionArchiveTest extends BaseTestCase {
    @TempDir Path temp;
    @Test void portableSetupReopensWithNoOriginalMachinePaths() throws Exception {
        var store = new MissionStore(temp.resolve("machine-a"), digest -> null);
        var draft = store.create("Portable"); String id = MissionStore.id(draft);
        var simulation = RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket()); simulation.getOptions().setTimeStep(.05);
        draft = store.bindFlight(id, 0, simulation, "original.ork");
        Path setup = temp.resolve("portable.rktmission-setup"); store.exportSetup(id, setup);
        try (var paths = Files.walk(store.root())) { for (var p : paths.sorted(Comparator.reverseOrder()).toList()) Files.delete(p); }
        var second = new MissionStore(temp.resolve("machine-b"), digest -> null);
        var loaded = second.importSetup(setup); assertEquals(draft, loaded);
        assertEquals(MissionFlightEditor.capture(simulation), MissionFlightEditor.capture(second.editableFlight(id)));
        assertEquals(6, second.validate(id).size()); // Unresolved heights remain unresolved after transport.
    }

    @Test void verifiedReadsAreLazyAndCorruptionCannotReplaceAnExistingResult() throws Exception {
        Path mission = temp.resolve("mission.json"), bytes = temp.resolve("bytes.bin"), archive = temp.resolve("result.rktmission");
        MissionDefinition.createUrrg("Archive").write(mission); Files.write(bytes, new byte[] {1, 2, 3});
        var files = Map.of("mission.json", mission, "runs/run1/tx.bin", bytes);
        MissionArchive.write(archive, MissionArchive.RESULT, "incomplete", files);
        byte[] original = Files.readAllBytes(archive);
        try (var reader = new MissionArchive(archive)) {
            assertEquals("incomplete", reader.manifest().getString("status"));
            assertArrayEquals(new byte[] {1, 2, 3}, reader.read("runs/run1/tx.bin", 3));
            assertThrows(IOException.class, () -> reader.read("runs/run1/tx.bin", 2));
        }
        assertThrows(IOException.class, () -> MissionArchive.write(archive, MissionArchive.RESULT, "complete", Map.of("mission.json", mission, "../escape", bytes)));
        assertArrayEquals(original, Files.readAllBytes(archive));
        // Repack with a valid ZIP CRC but a payload differing from its signed inventory.
        Path corrupt = temp.resolve("corrupt.zip");
        try (var input = new java.util.zip.ZipFile(archive.toFile()); var out = new ZipOutputStream(Files.newOutputStream(corrupt))) {
            for (var entry : Collections.list(input.entries())) {
                out.putNextEntry(new ZipEntry(entry.getName()));
                if (entry.getName().equals("runs/run1/tx.bin")) out.write(new byte[] {3, 2, 1});
                else try (var stream = input.getInputStream(entry)) { stream.transferTo(out); }
                out.closeEntry();
            }
        }
        try (var reader = new MissionArchive(corrupt)) {
            assertNotNull(reader.read("mission.json", MissionJson.MAX_BYTES));
            assertThrows(IOException.class, () -> reader.read("runs/run1/tx.bin", 3));
        }
    }

    @Test void traversalDuplicatesSymlinksAndTruncationAreRejectedBeforeExtraction() throws Exception {
        for (String name : List.of("../escape", "/absolute", "C:/absolute", "a/../b", "a\\b", "NUL.txt", "a/CON", "trailing.")) {
            Path file = temp.resolve(UUID.randomUUID() + ".zip");
            try (var zip = new ZipArchiveOutputStream(file.toFile())) { zip.putArchiveEntry(new ZipArchiveEntry(name)); zip.write(1); zip.closeArchiveEntry(); }
            assertThrows(IOException.class, () -> new MissionArchive(file), name);
        }
        Path duplicate = temp.resolve("duplicate.zip");
        try (var zip = new ZipArchiveOutputStream(duplicate.toFile())) {
            for (String name : List.of("mission.json", "MISSION.json")) { zip.putArchiveEntry(new ZipArchiveEntry(name)); zip.write(1); zip.closeArchiveEntry(); }
        }
        assertThrows(IOException.class, () -> new MissionArchive(duplicate));
        Path symlink = temp.resolve("symlink.zip");
        try (var zip = new ZipArchiveOutputStream(symlink.toFile())) {
            var entry = new ZipArchiveEntry("link"); entry.setUnixMode(0120777); zip.putArchiveEntry(entry);
            zip.write("target".getBytes(StandardCharsets.UTF_8)); zip.closeArchiveEntry();
        }
        assertThrows(IOException.class, () -> new MissionArchive(symlink));
        Path truncated = temp.resolve("truncated.zip"); Files.write(truncated, Arrays.copyOf(Files.readAllBytes(duplicate), 20));
        assertThrows(IOException.class, () -> new MissionArchive(truncated));
    }
}
