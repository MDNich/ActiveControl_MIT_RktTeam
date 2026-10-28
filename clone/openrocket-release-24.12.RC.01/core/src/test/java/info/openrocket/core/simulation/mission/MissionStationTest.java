package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import static org.junit.jupiter.api.Assertions.*;

class MissionStationTest {
    @TempDir Path temp;
    static byte[] pointer(float az, float el, int op) {
        var b = ByteBuffer.allocate(11).order(ByteOrder.LITTLE_ENDIAN); b.put((byte) 0xaa).put((byte) op).putFloat(az).putFloat(el);
        int sum = 0; for (int i = 1; i < 10; i++) sum += b.array()[i] & 255; b.put((byte) sum); return b.array();
    }
    @Test void referenceZeroNeverTeleportsAndStaleDoesNotStopMotion() {
        var mount = new MissionPointer(Json.createObjectBuilder().add("azimuthRateDegS", 90).add("elevationRateDegS", 60).build());
        mount.command(pointer(90, 60, 0), 0, 500_000, 0); mount.advance(500_000);
        assertEquals(45, mount.azimuth()); assertEquals(30, mount.elevation());
        assertEquals("applied-zero", mount.command(pointer(0, 0, 5), 500_000, 1_000_000, 500_000));
        assertEquals(45, mount.azimuth()); assertEquals(30, mount.elevation());
        mount.advance(1_000_000); assertEquals(90, mount.azimuth()); assertEquals(60, mount.elevation());
        assertEquals("expired", mount.command(pointer(0, 0, 0), 0, 500_000, 1_000_000));
    }
    @Test void linkIndependenceLossCorruptionAndUartPacing() {
        var p = Json.createObjectBuilder().add("receiverId", "gs1").add("uartBaud", 115200).build();
        var link = new MissionLink(p, "42", 0);
        assertEquals(12_500, link.transmit(0, 0, new byte[128], 144, true, true).deliveryUs());
        assertEquals(25_000, link.transmit(0, 1, new byte[128], 144, true, true).deliveryUs());
        assertEquals("routing-excluded", link.transmit(0, 2, new byte[128], 144, false, true).outcome());
        var all = new MissionLink(Json.createObjectBuilder(p).add("packetLossFraction", 1).build(), "42", 0);
        assertEquals("packet-loss", all.transmit(0, 0, new byte[128], 144, true, true).outcome());
        var corrupt = new MissionLink(Json.createObjectBuilder(p).add("bitErrorProbability", 1).build(), "42", 0);
        var bytes = corrupt.transmit(0, 0, new byte[128], 144, true, true).payload();
        for (byte value : bytes) assertEquals((byte) 255, value);
        var a = new MissionLink(Json.createObjectBuilder(p).add("packetLossFraction", .5).build(), "42", 0);
        var b = new MissionLink(Json.createObjectBuilder(p).add("packetLossFraction", .5).build(), "42", 0);
        var other = new MissionLink(Json.createObjectBuilder(p).add("receiverId", "gs2").build(), "42", 0);
        for (int i = 0; i < 100; i++) {
            var expected = a.transmit(i * 50_000, i, new byte[128], 144, true, true);
            other.transmit(i * 50_000, i, new byte[128], 144, true, true);
            var actual = b.transmit(i * 50_000, i, new byte[128], 144, true, true);
            assertEquals(expected.outcome(), actual.outcome()); assertEquals(expected.deliveryUs(), actual.deliveryUs());
        }
    }
    @Test void bundledWorkerUsesRealDecoderAndPolicyOverAuthenticatedSocket() throws Exception {
        String python = System.getenv("MISSION_TEST_PYTHON");
        Assumptions.assumeTrue(python != null, "Set MISSION_TEST_PYTHON to exercise the packaged Python worker");
        System.setProperty("openrocket.mission.python", python);
        try (var worker = new MissionWorker(temp, "mission", "run", "member-0")) {
            assertFalse(Files.exists(temp.resolve("launch-private.json")));
            var station = Json.createObjectBuilder().add("id", "gs1").add("origin", Json.createArrayBuilder().add(42.70278).add(-77.17606).add(250))
                    .add("pointerEnabled", true).build();
            worker.configure(Json.createArrayBuilder().add(station).build());
            byte[] payload = new byte[128]; var data = ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN);
            payload[31] = 3; data.putInt(32, 427041678).putInt(36, -771902950).putFloat(59, 500).putInt(80, 1000);
            int sum = 0; for (int i = 0; i < 127; i++) sum += payload[i] & 255; payload[127] = (byte) sum;
            var delivery = Json.createObjectBuilder().add("data", Base64.getEncoder().encodeToString(LegacyTelemetryEnvelope.wrap(payload, new byte[14])))
                    .add("packetId", "tx-0").add("txTimeUs", "0");
            var batch = Json.createObjectBuilder().add("id", "gs1").add("deliveries", Json.createArrayBuilder().add(delivery))
                    .add("actions", Json.createArrayBuilder().add(Json.createObjectBuilder().add("kind", "connect"))
                            .add(Json.createObjectBuilder().add("kind", "start-tracking")));
            var result = worker.advance(0, Json.createArrayBuilder().add(batch).build()).getJsonArray("stations").getJsonObject(0);
            assertTrue(result.getBoolean("tracking")); assertEquals(1, result.getJsonArray("samples").size());
            byte[] command = Base64.getDecoder().decode(result.getJsonArray("writes").getJsonObject(0).getString("data"));
            assertEquals(11, command.length); assertEquals((byte) 0xaa, command[0]);
            assertEquals("SIMULATION", result.getJsonArray("samples").getJsonObject(0).getString("source"));
        } finally { System.clearProperty("openrocket.mission.python"); }
    }
}
