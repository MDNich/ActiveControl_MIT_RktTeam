package edu.mit.rocket_team.zephyrus.telemetry;
import java.util.ArrayDeque;
import edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.Trace;
/** Deterministic byte queue in place of the FC's CC1200, no RF simulation. */
public class RTTelemetryEngine implements AutoCloseable {
    public static final String CSV_HEADER="timestamp,pyros,servos,servos_deg,accelerometer,barofilteredalt,temp,gyro,gps_fix,lat,lon,gpsalt,gps_horiz_prec,gps_vert_prec,gps_num_sat,flight_time,yaw_gyro_int,pitch_gyro_int,roll_gyro_int,state,pktnum,rssi,armed_pyros,fired_pyros,badpackets,rxrssi,accel_integrated_velo,baro_max_alt,gps_max_alt,pyro_resistances,cell_voltages,total_current,converter_voltages,converter_currents,bms_protections_enabled,bms_protection_status,bms_temp,enabled_status,angleFromVertical,gnd_lat,gnd_lon,gnd_fix,gnd_alt";
    private java.io.BufferedWriter csv, transmittedCsv;
    private java.io.OutputStream binary, transmittedBinary;
    private java.nio.file.Path csvPath, packetPath, metadataPath;
    private final double epochSeconds = Double.parseDouble(System.getProperty("openrocket.fc.epochSeconds", "0"));
    private final TelemetryLinkSettings settings;
    private final java.util.Random random;
    private record Delivery(long index, long txUs, long rxUs, byte[] packet) {}
    private final ArrayDeque<Delivery> pending = new ArrayDeque<>();
    private long generated, received, dropped;
    private boolean finished;

    public java.nio.file.Path getCsvPath() { return csvPath; }
    public java.nio.file.Path getPacketPath() { return packetPath; }
    public long getGeneratedCount() { return generated; }
    public long getReceivedCount() { return received; }
    public long getDroppedCount() { return dropped; }
    public int getPendingCount() { return pending.size(); }

    public void open(java.nio.file.Path parent) {
        if (csvPath != null || finished) throw new IllegalStateException("Telemetry already opened or closed");
        try {
            java.nio.file.Files.createDirectories(parent);
            java.nio.file.Path dir = java.nio.file.Files.createTempDirectory(parent, "zephyrus-").toAbsolutePath();
            csvPath = dir.resolve("telemetry.csv");
            packetPath = dir.resolve("packets.bin");
            metadataPath = dir.resolve("metadata.txt");
            csv = java.nio.file.Files.newBufferedWriter(csvPath, java.nio.file.StandardOpenOption.CREATE_NEW);
            binary = java.nio.file.Files.newOutputStream(packetPath, java.nio.file.StandardOpenOption.CREATE_NEW);
            transmittedCsv = java.nio.file.Files.newBufferedWriter(dir.resolve("transmitted-telemetry.csv"), java.nio.file.StandardOpenOption.CREATE_NEW);
            transmittedBinary = java.nio.file.Files.newOutputStream(dir.resolve("transmitted-packets.bin"), java.nio.file.StandardOpenOption.CREATE_NEW);
            for (var writer : new java.io.BufferedWriter[]{csv, transmittedCsv}) {
                writer.write(CSV_HEADER); writer.newLine(); writer.flush();
            }
            writeMetadata("running");
            trace.log("telemetry.files", "csv=" + csvPath + " packets=" + packetPath + " transmitted=" + dir.resolve("transmitted-telemetry.csv") + " metadata=" + metadataPath);
        } catch (java.io.IOException e) {
            try { finish(false); } catch (RuntimeException closeError) { e.addSuppressed(closeError); }
            throw new java.io.UncheckedIOException(e);
        }
    }

    private void writeMetadata(String completion) throws java.io.IOException {
        if (metadataPath == null) return;
        java.nio.file.Files.writeString(metadataPath,
            "Zephyrus Java FC telemetry; link format version=1\n" +
            "CSV schema: ZEPH_TEST_FLIGHT_GS1/2/3.csv (43 columns).\n" +
            "telemetry.csv / packets.bin: received packets in arrival order.\n" +
            "transmitted-telemetry.csv / transmitted-packets.bin: every generated packet.\n" +
            "Binary: consecutive 128-byte FC payloads; checksum at byte 127, no RF framing.\n" +
            "Received timestamp=epochSeconds+scheduled arrival boot microseconds/1e6; transmitted timestamp uses transmission time.\n" +
            "epochSeconds=" + epochSeconds + "\nflight_time retains firmware transmission-time boot milliseconds.\n" +
            "packetLossFraction=" + settings.packetLossFraction() + "\ndownlinkDelayMs=" + settings.delayMs() +
            "\nrandomSeed=" + settings.randomSeed() + "\nrandomAlgorithm=java.util.Random.nextDouble; one draw per generated packet\n" +
            "generated=" + generated + "\nreceived=" + received + "\ndropped=" + dropped + "\npending=" + pending.size() +
            "\ncompletion=" + completion + "\n" +
            "Delay/loss affect downlink only. No physical RF/CPU execution model.\n" +
            "RSSI and ground station location unavailable; GPS uncertainty/satellites and pyro resistance unmodeled zero.\n" +
            "Power nominal; currents zero. Physical roll off; pyros recorded only.\n" +
            "CSV decoded using historical ground station conventions. Accel X omits FC gain 1.060; gyro omits FC biases and negates Y.\n" +
            "Temperature decoder C5=0x91E3/C6=0x6FEC differs from FC C5=0x8405/C6=0x6D91.\n" +
            "Mounting sensor X=body Z,Y=body X,Z=body Y; ideal linkage; airbrake input time uses integer seconds.\n");
    }
    private static String vector(double... values) { return java.util.Arrays.toString(values); }
    private static int u16(java.nio.ByteBuffer b,int i) { return b.getShort(i)&65535; }
    private static long u32(java.nio.ByteBuffer b,int i) { return Integer.toUnsignedLong(b.getInt(i)); }
    private static int signed24(byte[] p,int i) { int n=(p[i]&255)|((p[i+1]&255)<<8)|((p[i+2]&255)<<16); return (n<<8)>>8; }
    public static String[] decode(byte[] p,double timestamp) {
        if(p.length!=128) throw new IllegalArgumentException("FC payload must have 128 bytes");
        int sum=0; for(int i=0;i<127;i++) sum+=p[i]&255;
        if((sum&255)!=(p[127]&255)) throw new IllegalArgumentException("Telemetry checksum mismatch");
        java.nio.ByteBuffer b=java.nio.ByteBuffer.wrap(p).order(java.nio.ByteOrder.LITTLE_ENDIAN);
        int[] pyros=new int[6],armed=new int[8],fired=new int[8],servos=new int[4];
        double[] resistances=new double[8],deg=new double[4],cells=new double[3],volts=new double[6],currents=new double[6];
        long servoBits=0; for(int i=0;i<6;i++) servoBits|=(p[10+i]&255L)<<(8*i);
        for(int i=0;i<6;i++) { pyros[i]=(u16(b,0)>>(2*i))&3; armed[i]=((p[2]&255)>>i)&1; fired[i]=((p[3]&255)>>i)&1; resistances[i]=(p[4+i]&255)/10.0; volts[i]=b.getShort(98+2*i)*0.0016; currents[i]=b.getShort(110+2*i)*0.000625; }
        for(int i=0;i<4;i++) { servos[i]=(int)((servoBits>>(12*i))&4095); deg[i]=Math.rint((servos[i]-1500)*(i<2?60.0:50.0)/500.0*100)/100.0; }
        for(int i=0;i<3;i++) cells[i]=b.getShort(87+2*i)/1000.0;
        int rawTemp=(p[56]&255)|((p[57]&255)<<8)|((p[58]&255)<<16);
        double temperature=(2000+(rawTemp-0x91E3*256.0)*0x6FEC/(1<<23))/100;
        float roll=b.getFloat(64),pitch=b.getFloat(68),yaw=b.getFloat(72);
        double angle=Math.acos(Math.cos(pitch*Math.PI/180)*Math.cos(yaw*Math.PI/180))*180/Math.PI;
        StringBuilder enabled=new StringBuilder("["); double[] nominal={3,3.3,5,7.4,8.4,28};
        for(int i=0;i<6;i++) { if(i>0) enabled.append(", "); enabled.append(Math.abs(volts[i]-nominal[i])<1?"True":"False"); } enabled.append(']');
        int state=p[63]&255;
        return new String[]{Double.toString(timestamp),java.util.Arrays.toString(pyros),java.util.Arrays.toString(servos),vector(deg),
            vector(signed24(p,16)/12800.0*9.80665,signed24(p,19)/12800.0*9.80665,signed24(p,22)/12800.0*9.80665),
            Float.toString(b.getFloat(59)),Double.toString(temperature),vector(b.getShort(25)*0.03051757812,b.getShort(27)*-0.03051757812,b.getShort(29)*0.03051757812),
            Integer.toString(p[31]&255),Double.toString(b.getInt(32)*1e-7),Double.toString(b.getInt(36)*1e-7),Float.toString(b.getFloat(40)),
            Double.toString(u32(b,44)/1000.0),Double.toString(u32(b,48)/1000.0),Integer.toString(p[52]&255),Long.toString(u32(b,80)),
            Float.toString(yaw),Float.toString(pitch),Float.toString(roll),"state."+edu.mit.rocket_team.zephyrus.util.RTRocketState.values()[state],
            Integer.toString(u16(b,84)),"",java.util.Arrays.toString(armed),java.util.Arrays.toString(fired),"0","",Float.toString(b.getFloat(122)),
            Integer.toString(u16(b,76)),Integer.toString(u16(b,78)),vector(resistances),vector(cells),Double.toString(b.getShort(93)/-1000.0),vector(volts),vector(currents),
            Integer.toString(p[97]&255),Integer.toString(p[96]&255),Double.toString((p[95]&255)/2.0),enabled.toString(),Double.toString(angle),"","","",""};
    }
    private void export(byte[] packet, long timeUs, java.io.BufferedWriter writer, java.io.OutputStream raw) {
        if (writer == null) return;
        try {
            raw.write(packet); raw.flush();
            String[] cells = decode(packet, epochSeconds + timeUs / 1e6);
            for (int i = 0; i < cells.length; i++) {
                if (i > 0) writer.write(',');
                writer.write('"'); writer.write(cells[i].replace("\"", "\"\"")); writer.write('"');
            }
            writer.newLine(); writer.flush();
        } catch (java.io.IOException e) { throw new java.io.UncheckedIOException(e); }
    }

    /** Export scheduled arrivals without changing the firmware clock. */
    public void deliverDue(long nowUs) {
        while (!pending.isEmpty() && pending.peek().rxUs() <= nowUs) {
            Delivery delivery = pending.peek();
            export(delivery.packet(), delivery.rxUs(), csv, binary);
            pending.remove(); received++;
            trace.log("telemetry.received", "index=" + delivery.index() + " tx_boot_us=" + delivery.txUs() + " rx_boot_us=" + delivery.rxUs());
        }
    }

    /** Only successful completion drains future arrivals; cancellation keeps them pending in metadata. */
    public void finish(boolean complete) {
        if (finished) return;
        finished = true;
        RuntimeException error = null;
        try { if (complete) deliverDue(Long.MAX_VALUE); }
        catch (RuntimeException e) { error = e; complete = false; }
        for (java.io.Closeable stream : new java.io.Closeable[]{csv, binary, transmittedCsv, transmittedBinary}) {
            if (stream == null) continue;
            try { stream.close(); }
            catch (java.io.IOException e) {
                complete = false;
                if (error == null) error = new java.io.UncheckedIOException(e); else error.addSuppressed(e);
            }
        }
        csv = null; binary = null; transmittedCsv = null; transmittedBinary = null;
        try { writeMetadata(complete ? "complete" : "incomplete"); }
        catch (java.io.IOException e) { if (error == null) error = new java.io.UncheckedIOException(e); else error.addSuppressed(e); }
        trace.log("telemetry.close", "csv=" + csvPath + " generated=" + generated + " received=" + received + " dropped=" + dropped + " pending=" + pending.size() + " complete=" + complete);
        if (error != null) throw error;
    }
    @Override public void close() { finish(true); }

    private final Trace trace;
    private final ArrayDeque<byte[]> commands = new ArrayDeque<>();
    private byte[] lastPacket = new byte[128];
    public RTTelemetryEngine() { this(new Trace()); }
    public RTTelemetryEngine(Trace trace) { this(trace, TelemetryLinkSettings.DEFAULT); }
    public RTTelemetryEngine(Trace trace, TelemetryLinkSettings settings) {
        this.trace = trace; this.settings = java.util.Objects.requireNonNull(settings);
        if (!Double.isFinite(epochSeconds)) throw new IllegalArgumentException("Telemetry epoch must be finite");
        random = new java.util.Random(settings.randomSeed());
    }
    public void setup() { trace.log("radio.setup", "loss_fraction=" + settings.packetLossFraction() + " delay_ms=" + settings.delayMs() + " seed=" + settings.randomSeed()); }
    public void enqueue(byte[] packet) { commands.add(packet.clone()); trace.log("command.queue", "bytes=" + packet.length); }
    public byte[] receive() { byte[] packet = commands.poll(); trace.log("radio.receive", "bytes=" + (packet == null ? 0 : packet.length)); return packet; }
    public void send(byte[] packet) {
        if (finished) throw new IllegalStateException("Telemetry closed");
        lastPacket = packet.clone();
        export(lastPacket, trace.bootUs(), transmittedCsv, transmittedBinary);
        generated++;
        trace.log("telemetry.send", "index=" + generated + " bytes=" + packet.length + " hex=" + java.util.HexFormat.of().formatHex(packet));
        if (random.nextDouble() < settings.packetLossFraction()) {
            dropped++;
            trace.log("telemetry.dropped", "index=" + generated + " tx_boot_us=" + trace.bootUs());
        } else {
            pending.add(new Delivery(generated, trace.bootUs(), trace.bootUs() + settings.delayMs() * 1000L, lastPacket));
        }
        deliverDue(trace.bootUs());
    }
    public byte[] getLastPacket() { return lastPacket.clone(); }
}
