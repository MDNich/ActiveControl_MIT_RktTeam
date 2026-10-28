package info.openrocket.core.simulation.mission;

import edu.mit.rocket_team.FC.RTFC;
import info.openrocket.core.simulation.flightcomputer.*;
import info.openrocket.core.util.BaseTestCase;
import jakarta.json.*;
import java.nio.*;
import java.nio.charset.StandardCharsets;
import java.util.*;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class TelemetryCodecTest extends BaseTestCase {
    private JsonObject fixture() { return MissionJson.resource("codec-vectors.json"); }

    @Test void independentLayoutVectorAndUnavailableProvenance() {
        var v = fixture(); var codec = new TelemetryCodec(v.getJsonObject("codec"));
        var values = new HashMap<String, Double>(); v.getJsonObject("values").forEach((k,x) -> values.put(k, ((JsonNumber)x).doubleValue()));
        var encoded = codec.encode(values);
        assertEquals(v.getString("payloadHex"), HexFormat.of().formatHex(encoded.bytes()));
        assertEquals(Set.of("state"), encoded.unavailable());
        var decoded = codec.decode(encoded.bytes());
        v.getJsonObject("decoded").forEach((k,x) -> assertEquals(((JsonNumber)x).doubleValue(), decoded.get(k), 1e-12));
        byte[] damaged = encoded.bytes(); damaged[0] ^= 1;
        assertThrows(IllegalArgumentException.class, () -> codec.decode(damaged));
        assertEquals(v.getString("payloadHex"), HexFormat.of().formatHex(encoded.bytes()));
        assertEquals(30, codec.frame(encoded.bytes(), new byte[14]).length);
        assertThrows(IllegalArgumentException.class, () -> codec.frame(encoded.bytes(), new byte[0]));
    }

    @Test void publishedCrcCheckStringsForReflectedAndUnreflectedProfiles() {
        for (var check : fixture().getJsonArray("crcChecks")) {
            var c = check.asJsonObject();
            var checksum = Json.createObjectBuilder(c).remove("expected").add("algorithm", "crc16")
                    .add("byteOffset", 9).add("start", 0).add("endExclusive", 9).build();
            var definition = Json.createObjectBuilder(fixture().getJsonObject("codec")).add("payloadBytes", 11)
                    .add("fields", Json.createArrayBuilder()).add("checksum", checksum).build();
            var codec = new TelemetryCodec(definition);
            var packet = ByteBuffer.allocate(11).order(ByteOrder.LITTLE_ENDIAN).put("123456789".getBytes(StandardCharsets.US_ASCII)).putShort((short)c.getInt("expected")).array();
            assertTrue(codec.decode(packet).isEmpty());
            packet[9] ^= 1; assertThrows(IllegalArgumentException.class, () -> codec.decode(packet));
        }
    }

    @Test void declarationsAndValuesCannotEscapeFixedLayout() {
        JsonObject definition = fixture().getJsonObject("codec");
        for (String patch : List.of(
                "[{\"op\":\"replace\",\"path\":\"/fields/0/byteOffset\",\"value\":13}]",
                "[{\"op\":\"replace\",\"path\":\"/fields/1/byteOffset\",\"value\":1}]",
                "[{\"op\":\"replace\",\"path\":\"/fields/0/source\",\"value\":\"truth.altitude\"}]",
                "[{\"op\":\"replace\",\"path\":\"/fields/0/scale\",\"value\":0}]",
                "[{\"op\":\"replace\",\"path\":\"/checksum/endExclusive\",\"value\":14}]")) {
            try (var reader = Json.createReader(new java.io.StringReader(patch))) {
                var changed = Json.createPatch(reader.readArray()).apply(definition);
                assertThrows(IllegalArgumentException.class, () -> new TelemetryCodec(changed));
            }
        }
        var codec = new TelemetryCodec(definition);
        assertThrows(IllegalArgumentException.class, () -> codec.encode(Map.of()));
        assertThrows(IllegalArgumentException.class, () -> codec.encode(Map.of("gps.latitude_deg", 1e9, "gps.height_m", 0d, "uptime_ms", 0d)));
        assertThrows(IllegalArgumentException.class, () -> codec.encode(Map.of("gps.latitude_deg", 0d, "gps.height_m", 1e100, "uptime_ms", 0d)));
    }

    @Test void fcV3MigrationPreservesControllerVersionAndRequiresMissionCodecConsumer() {
        var original = FlightComputerDesign.zephyrus();
        original.requireRunnable();
        var v3 = new FlightComputerDesign(Json.createObjectBuilder(original.json()).add("version", 3)
                .add("telemetryFormat", fixture().getJsonObject("codec")).build());
        v3.requireRunnable();
        assertEquals("mission-fixture.v1", v3.telemetryCodec().id());
        var edited = FlightComputerController.replace(v3, v3.controllers());
        assertEquals(3, edited.json().getInt("version"));
        assertEquals(v3.json().getJsonObject("telemetryFormat"), edited.json().getJsonObject("telemetryFormat"));
        assertThrows(IllegalArgumentException.class, () -> new RTFC().configure(v3));
        assertThrows(IllegalArgumentException.class, () -> new FlightComputerDesign(Json.createObjectBuilder(v3.json()).add("version", 2).build()));
        assertNull(original.telemetryCodec());
        var fallback = new FlightComputerDesign(Json.createObjectBuilder(original.json()).add("version", 3).build());
        fallback.requireRunnable(); assertNull(fallback.telemetryCodec());
    }
}
