package edu.mit.rocket_team.zephyrus;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import java.nio.file.*;
import edu.mit.rocket_team.zephyrus.FC.RTFC;
import edu.mit.rocket_team.zephyrus.telemetry.RTTelemetryEngine;
import static edu.mit.rocket_team.zephyrus.RTTestInputs.*;
import static org.junit.jupiter.api.Assertions.*;
class RTTelemetryTest {
    @TempDir Path dir;
    @Test void exportedRowsDecodeTheTransmittedPackets() throws Exception {
        RTFC c=fresh(); c.telemetry.open(dir); for(int ms=0;ms<=180;ms+=10) tick(c,ms,9.8065f,100,3);
        Path csv=c.telemetry.getCsvPath(),raw=c.telemetry.getPacketPath(); c.telemetry.close();
        byte[] packets=Files.readAllBytes(raw); assertEquals(3*128,packets.length);
        assertEquals(4,Files.readAllLines(csv).size()); assertEquals(43,RTTelemetryEngine.CSV_HEADER.split(",").length);
        byte[] packet=java.util.Arrays.copyOfRange(packets,0,128); String[] row=RTTelemetryEngine.decode(packet,0.06);
        assertEquals(43,row.length); assertEquals("60",row[15]); assertEquals("state.GROUND_TESTING",row[19]); assertEquals("1",row[20]);
        assertEquals("[941, 1500, 1495, 1430]",row[2]);
        assertTrue(row[4].startsWith("[9.25"),"GS decoder omits FC X calibration: "+row[4]);
        assertEquals("[3.0, 3.3008, 5.0, 7.4, 8.4, 28.0]",row[32]);
        assertEquals("",row[21]); assertTrue(Files.readString(csv).contains("\"state.GROUND_TESTING\""));
        packet[7]++; assertThrows(IllegalArgumentException.class,()->RTTelemetryEngine.decode(packet,0));
    }
}
