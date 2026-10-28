package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.security.MessageDigest;
import java.util.*;
import java.util.stream.Stream;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import static org.junit.jupiter.api.Assertions.*;

class MissionContractsTest {
    @TempDir Path temporary;

    @TestFactory
    Stream<DynamicTest> sharedConfigurationCases() {
        return MissionJson.resource("configuration-cases.json").getJsonArray("cases").stream().map(v -> {
            var c = v.asJsonObject();
            return DynamicTest.dynamicTest(c.getString("name"), () -> {
                JsonObject draft = Json.createPatch(c.getJsonArray("patch")).apply(MissionJson.resource("urrg-draft.json"));
                var expected = new TreeSet<String>();
                for (var entry : c.getJsonArray("expected")) {
                    var e = entry.asJsonArray();
                    expected.add(e.getString(0) + ":" + e.getString(1));
                }
                var actual = new TreeSet<String>();
                MissionDefinition.validate(draft).forEach(i -> actual.add(i.path() + ":" + i.code()));
                assertEquals(expected, actual);
            });
        });
    }

    @Test
    void draftRoundTripPreservesUnknownFieldsAndUnsetHeights() throws Exception {
        var first = MissionDefinition.createUrrg("My mission");
        var unknown = Json.createObjectBuilder().add("custom", Json.createArrayBuilder().add(1).add("preserve")).build();
        var draft = new MissionDefinition(Json.createObjectBuilder(first.json()).add("future", unknown).build());
        var path = temporary.resolve("mission.json");
        draft.write(path);
        assertEquals(draft.json(), MissionDefinition.read(path).json());
        for (var site : draft.json().getJsonArray("sites")) assertTrue(site.asJsonObject().isNull("heightM"));
        var second = MissionDefinition.createUrrg("My mission");
        assertNotEquals(first.json().getJsonObject("identity").getString("missionId"),
                second.json().getJsonObject("identity").getString("missionId"));
        try (var files = Files.list(temporary)) { assertEquals(1, files.count()); }
    }

    @Test
    void frozenUrrgCoordinatesMatchTemplateWithoutQuantization() {
        var draft = MissionDefinition.createUrrg("URRG").json();
        var sites = draft.getJsonArray("sites");
        var reference = MissionJson.resource("urrg-sites.json").getJsonArray("sites");
        assertEquals(6, sites.size());
        assertEquals(4, draft.getJsonArray("pointers").size());
        for (int i = 0; i < sites.size(); i++) {
            var a = sites.getJsonObject(i); var b = reference.getJsonObject(i);
            assertEquals(b.getString("id"), a.getString("id"));
            assertEquals(b.getJsonNumber("latitudeDeg"), a.getJsonNumber("latitudeDeg"));
            assertEquals(b.getJsonNumber("longitudeDeg"), a.getJsonNumber("longitudeDeg"));
            assertEquals(b.getString("mgrs"), a.getJsonObject("coordinateInput").getString("value"));
            assertEquals("grid-point", a.getJsonObject("coordinateInput").getString("reference"));
        }
    }

    @Test
    void strictJsonRejectsAmbiguityMalformedTextAndResourceExhaustion() {
        for (String invalid : List.of("[]", "{} {}", "{\"x\":1,\"x\":2}",
                "{\"x\":1,\"\\u0078\":2}", "{\"x\":NaN}", "{\"x\":1e999}", "{\"x\":\"\\ud800\"}",
                "{\"x\":" + "[".repeat(65) + "0" + "]".repeat(65) + "}"))
            assertThrows(IOException.class, () -> MissionJson.parse(invalid.getBytes(StandardCharsets.UTF_8)), invalid);
        assertThrows(IOException.class, () -> MissionJson.parse(new byte[] {(byte) 255}));
        assertThrows(IOException.class, () -> MissionJson.parse(new byte[MissionJson.MAX_BYTES + 1]));
        assertThrows(IllegalArgumentException.class, () -> MissionSchema.validate(
                Json.createObjectBuilder().add("$ref", "https://example.invalid/schema").build(), JsonValue.EMPTY_JSON_OBJECT));
    }

    @Test
    void failedAtomicWriteDoesNotDamageExistingDestination() throws Exception {
        var path = temporary.resolve("saved.json");
        Files.writeString(path, "keep previous");
        var oversized = Json.createObjectBuilder().add("large", "x".repeat(MissionJson.MAX_BYTES)).build();
        assertThrows(IOException.class, () -> MissionJson.writeAtomic(path, oversized));
        assertEquals("keep previous", Files.readString(path));
    }

    @Test
    void samplerMatchesIndependentHashAndNormalVectors() {
        var reference = MissionJson.resource("sampler-vectors.json");
        for (var entry : reference.getJsonArray("vectors")) {
            var v = entry.asJsonObject(); var f = v.getJsonArray("fields");
            String seed = f.getString(1), participant = f.getString(3), phenomenon = f.getString(4), model = f.getString(5);
            long member = Long.parseLong(f.getString(2)), event = Long.parseLong(f.getString(6)), draw = Long.parseLong(f.getString(7));
            assertEquals(v.getString("sha256"), HexFormat.of().formatHex(MissionRandom.digest(seed, member, participant, phenomenon, model, event, draw)));
            assertEquals(v.getJsonNumber("uniform").doubleValue(), MissionRandom.uniform(seed, member, participant, phenomenon, model, event, draw));
        }
        for (var entry : reference.getJsonArray("normalVectors")) {
            var v = entry.asJsonObject(); var f = v.getJsonArray("fields");
            assertEquals(v.getJsonNumber("normal").doubleValue(), MissionRandom.normal(f.getString(1), Long.parseLong(f.getString(2)),
                    f.getString(3), f.getString(4), f.getString(5), Long.parseLong(f.getString(6)), Long.parseLong(f.getString(7))), 1e-14);
        }
        double before = MissionRandom.uniform("1", 0, "gs1", "loss", "1", 23, 0);
        for (int i = 0; i < 100; i++) MissionRandom.uniform("1", 0, "gs4", "loss", "1", i, 0);
        assertEquals(before, MissionRandom.uniform("1", 0, "gs1", "loss", "1", 23, 0));
        for (String seed : List.of("-1", "01", "18446744073709551616"))
            assertThrows(IllegalArgumentException.class, () -> MissionRandom.uniform(seed, 0, "gs1", "loss", "1", 0, 0));
    }

    @Test
    void geodesyMatchesAnalyticAxesAndExplicitHeightReference() {
        var reference = MissionJson.resource("geodesy-vectors.json");
        for (var value : reference.getJsonArray("ecef")) {
            var v = value.asJsonObject();
            double[] actual = MissionGeodesy.ecef(v.getJsonNumber("lat").doubleValue(), v.getJsonNumber("lon").doubleValue(), v.getJsonNumber("h").doubleValue());
            for (int i = 0; i < 3; i++) assertEquals(v.getJsonArray("xyz").getJsonNumber(i).doubleValue(), actual[i], 1e-8);
        }
        assertEquals(-10.842, MissionGeodesy.ellipsoidalHeight(23, -33.842), 1e-12);
        assertThrows(IllegalArgumentException.class, () -> MissionGeodesy.ellipsoidalHeight(23, Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> MissionGeodesy.ecef(91, 0, 0));
    }

    @Test
    void byteFramingMatchesLiteralVectorAndHandlesFragmentation() throws Exception {
        var v = MissionJson.resource("ipc-vector.json");
        byte[] expected = HexFormat.of().parseHex(v.getString("frameHex"));
        var out = new ByteArrayOutputStream();
        MissionWire.write(out, v.getJsonObject("message"));
        assertArrayEquals(expected, out.toByteArray());
        MissionWire.write(out, v.getJsonObject("message"));
        var fragmented = new FilterInputStream(new ByteArrayInputStream(out.toByteArray())) {
            @Override public int read(byte[] b, int offset, int count) throws IOException { return super.read(b, offset, Math.min(count, 1)); }
        };
        assertEquals(v.getJsonObject("message"), MissionWire.read(fragmented));
        assertEquals(v.getJsonObject("message"), MissionWire.read(fragmented));
        assertNull(MissionWire.read(fragmented));
        for (int length : new int[] {1, 3, 4, expected.length - 1}) {
            byte[] truncated = Arrays.copyOf(expected, length);
            assertThrows(EOFException.class, () -> MissionWire.read(new ByteArrayInputStream(truncated)));
        }
        assertThrows(IOException.class, () -> MissionWire.read(new ByteArrayInputStream(new byte[] {-1, -1, -1, -1})));
    }

    @Test
    void wireRejectsNumericTimeOverflowAndNoncanonicalBase64() {
        var message = MissionJson.resource("ipc-vector.json").getJsonObject("message");
        assertThrows(IllegalArgumentException.class, () -> MissionWire.validate(Json.createObjectBuilder(message).add("missionTimeUs", 42).build()));
        assertThrows(IllegalArgumentException.class, () -> MissionWire.validate(Json.createObjectBuilder(message).add("missionTimeUs", "9223372036854775808").build()));
        for (String data : List.of("@@", "YQ", "YR==", "Y Q==", "x".repeat(100_000)))
            assertThrows(IllegalArgumentException.class, () -> MissionWire.validate(Json.createObjectBuilder(message)
                    .add("body", Json.createObjectBuilder().add("data", data)).build()));
        MissionWire.validate(Json.createObjectBuilder(message).add("missionTimeUs", "-9223372036854775808").build());
    }

    @Test
    void legacyWrappingPreservesCorruptionAndReceiverBytes() {
        var v = MissionJson.resource("legacy-vectors.json");
        byte[] payload = HexFormat.of().parseHex(v.getString("payloadHex"));
        byte[] frame = HexFormat.of().parseHex(v.getString("frameHex"));
        byte[] trailer = Arrays.copyOfRange(frame, 130, 144);
        assertTrue(LegacyTelemetryEnvelope.validPayload(payload));
        assertArrayEquals(frame, LegacyTelemetryEnvelope.wrap(payload, trailer));
        payload[50] ^= 1;
        assertFalse(LegacyTelemetryEnvelope.validPayload(payload));
        assertArrayEquals(payload, LegacyTelemetryEnvelope.payload(LegacyTelemetryEnvelope.wrap(payload, trailer)));
        assertThrows(IllegalArgumentException.class, () -> LegacyTelemetryEnvelope.wrap(new byte[127], trailer));
    }

    @Test
    void bundledContractInventoryIsVerified() throws Exception {
        var inventory = MissionJson.resource("manifest.json").getJsonObject("files");
        for (String name : inventory.keySet()) {
            try (var input = MissionJson.class.getResourceAsStream("/mission/v1/" + name)) {
                assertNotNull(input);
                assertEquals(inventory.getString(name), HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(input.readAllBytes())), name);
            }
        }
    }
}
