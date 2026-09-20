package edu.mit.rocket_team.zephyrus.telemetry;
import java.util.ArrayDeque;
import edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.Trace;
/** Deterministic byte queue in place of the FC's CC1200, no RF simulation. */
public class RTTelemetryEngine implements AutoCloseable {
    public static final String CSV_HEADER="timestamp,pyros,servos,servos_deg,accelerometer,barofilteredalt,temp,gyro,gps_fix,lat,lon,gpsalt,gps_horiz_prec,gps_vert_prec,gps_num_sat,flight_time,yaw_gyro_int,pitch_gyro_int,roll_gyro_int,state,pktnum,rssi,armed_pyros,fired_pyros,badpackets,rxrssi,accel_integrated_velo,baro_max_alt,gps_max_alt,pyro_resistances,cell_voltages,total_current,converter_voltages,converter_currents,bms_protections_enabled,bms_protection_status,bms_temp,enabled_status,angleFromVertical,gnd_lat,gnd_lon,gnd_fix,gnd_alt";
    private java.io.BufferedWriter csv;
    private java.io.OutputStream binary;
    private java.nio.file.Path csvPath,packetPath;
    private final double epochSeconds=Double.parseDouble(System.getProperty("openrocket.fc.epochSeconds","0"));
    public java.nio.file.Path getCsvPath() { return csvPath; }
    public java.nio.file.Path getPacketPath() { return packetPath; }
    public void open(java.nio.file.Path parent) {
        if(csv!=null) throw new IllegalStateException("Telemetry already open");
        try {
            java.nio.file.Files.createDirectories(parent);
            java.nio.file.Path dir=java.nio.file.Files.createTempDirectory(parent,"zephyrus-").toAbsolutePath();
            csvPath=dir.resolve("telemetry.csv"); packetPath=dir.resolve("packets.bin");
            csv=java.nio.file.Files.newBufferedWriter(csvPath,java.nio.file.StandardOpenOption.CREATE_NEW);
            binary=java.nio.file.Files.newOutputStream(packetPath,java.nio.file.StandardOpenOption.CREATE_NEW);
            csv.write(CSV_HEADER); csv.newLine(); csv.flush();
            java.nio.file.Files.writeString(dir.resolve("metadata.txt"),
                "Zephyrus Java FC simulated telemetry\n"+
                "CSV schema: ZEPH_TEST_FLIGHT_GS1/2/3.csv (43 columns).\n"+
                "Binary: consecutive 128-byte FC payloads, including checksum at byte 127; no RF framing.\n"+
                "CSV values decoded from those payloads using RT_Python_Lib/ground_station/rocket.py conventions.\n"+
                "timestamp = epochSeconds + boot microseconds / 1e6; epochSeconds="+epochSeconds+"\n"+
                "flight_time is firmware boot milliseconds, as in the ground-station decoder; not seconds since liftoff.\n"+
                "RF delay/loss=0; RSSI and ground-station position/fix fields are unavailable (blank CSV cells).\n"+
                "GPS uncertainty/satellite count=unmodeled zero; decoded absolute height uses simulator datum.\n"+
                "Power=nominal simulated voltages, zero currents, 20 C BMS; pyro resistance=unmodeled zero.\n"+
                "Raw sensor counts synthesized from engineering readings with firmware calibration and sensor-range clamping.\n"+
                "CSV accelerometer X omits firmware factor 1.060, matching ground decoder.\n"+
                "CSV gyro omits firmware biases and negates Y, matching ground decoder.\n"+
                "CSV temp uses historical GS C5=0x91E3/C6=0x6FEC, while FC uses 0x8405/0x6D91; inspect FC console for engineering temperature.\n"+
                "FC sample mounting: sensor X=body Z,Y=body X,Z=body Y (assumed); ideal PWM linkage; physical roll off; pyros recorded only.\n"+
                "Firmware airbrake time is integer seconds; prediction patch 5046 m preserved.\n");
            trace.log("telemetry.files", "csv="+csvPath+" packets="+packetPath+" metadata="+dir.resolve("metadata.txt"));
        } catch(java.io.IOException e) { close(); throw new java.io.UncheckedIOException(e); }
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
    private void export(byte[] packet) {
        if(csv==null) return;
        try {
            binary.write(packet); binary.flush();
            String[] cells=decode(packet,epochSeconds+trace.bootUs()/1e6);
            for(int i=0;i<cells.length;i++) { if(i>0) csv.write(','); csv.write('"'); csv.write(cells[i].replace("\"","\"\"")); csv.write('"'); }
            csv.newLine(); csv.flush();
            trace.log("telemetry.write", "csv="+csvPath+" bytes=128");
        } catch(java.io.IOException e) { throw new java.io.UncheckedIOException(e); }
    }
    @Override public void close() {
        java.io.IOException error=null;
        try { if(csv!=null) csv.close(); } catch(java.io.IOException e) { error=e; } finally { csv=null; }
        try { if(binary!=null) binary.close(); } catch(java.io.IOException e) { error=e; } finally { binary=null; }
        if(csvPath!=null) trace.log("telemetry.close", "csv="+csvPath+" packets="+packetPath);
        if(error!=null) throw new java.io.UncheckedIOException(error);
    }

    private final Trace trace;
    private final ArrayDeque<byte[]> commands=new ArrayDeque<>();
    private byte[] lastPacket=new byte[128];
    public RTTelemetryEngine() { this(new Trace()); }
    public RTTelemetryEngine(Trace trace) { this.trace=trace; }
    public void setup() { trace.log("radio.setup", "mode=queued_bytes"); }
    public void enqueue(byte[] packet) { commands.add(packet.clone()); trace.log("command.queue", "bytes="+packet.length); }
    public byte[] receive() { byte[] packet=commands.poll(); trace.log("radio.receive", "bytes="+(packet==null?0:packet.length)); return packet; }
    public void send(byte[] packet) { lastPacket=packet.clone(); export(packet); trace.log("telemetry.send", "bytes="+packet.length+" hex="+java.util.HexFormat.of().formatHex(packet)); }
    public byte[] getLastPacket() { return lastPacket.clone(); }
}
