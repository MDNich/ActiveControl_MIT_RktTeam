package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;

/** Immutable, selected truth/TX history for a new ground-link experiment. Never runs an FC. */
public final class MissionRecordedSource implements AutoCloseable {
    public record Transmission(String vehicle, MissionTelemetrySource.Packet packet) { }
    private final MissionArchive archive;
    private final String prefix;
    private final PriorityQueue<Cursor> cursors = new PriorityQueue<>(Comparator.comparingLong((Cursor c) -> c.next.packet().timeUs()).thenComparing(c -> c.vehicle));
    public final MissionTruth truth;
    public final JsonObject original;
    private final class Cursor {
        final String vehicle;
        final Iterator<String> indexes;
        BufferedReader reader;
        byte[] raw;
        String rawName;
        Transmission next;
        long ordinal=-1, previous=Long.MIN_VALUE;
        Cursor(String vehicle, List<String> indexes) throws IOException { this.vehicle=vehicle;this.indexes=indexes.iterator();advance(); }
        void advance() throws IOException {
            while(true) {
                if(reader==null) {
                    if(!indexes.hasNext()){next=null;return;}
                    reader=new BufferedReader(new InputStreamReader(new ByteArrayInputStream(archive.read(indexes.next(),(int)MissionRecorder.CHUNK_BYTES)),StandardCharsets.UTF_8));
                }
                String line=reader.readLine();if(line==null){reader.close();reader=null;continue;}
                var metadata=MissionJson.parse(line.getBytes(StandardCharsets.UTF_8));
                long time=Long.parseLong(metadata.getString("timeUs"));
                if(time<previous || time< -1_000_000 || time>86_400_000_000L)throw new IOException("Recorded TX time is out of order or outside mission limits");
                previous=time;
                String name=metadata.getString("file");
                if(!name.startsWith(prefix+"transmitters/"+vehicle+"/tx/") || !name.endsWith(".bin"))throw new IOException("Recorded TX index references another stream");
                if(!name.equals(rawName)){raw=archive.read(name,(int)MissionRecorder.CHUNK_BYTES);rawName=name;}
                int offset=metadata.getInt("offset"),length=metadata.getInt("length");
                if(offset<0 || length<1 || length>65536 || (long)offset+length>raw.length)throw new IOException("Recorded TX index exceeds its chunk");
                byte[] payload=Arrays.copyOfRange(raw,offset,offset+length);
                if(!MissionWorker.digest(payload).equals(metadata.getString("sha256")) || !metadata.getString("transmitterId").equals(vehicle))throw new IOException("Recorded TX identity or hash mismatch");
                String packetId=metadata.getString("packetId");
                if(!packetId.startsWith(vehicle+"-"))throw new IOException("Invalid recorded packet identity");
                long sequence=Long.parseLong(packetId.substring(vehicle.length()+1));
                if(sequence<=ordinal || Long.parseLong(metadata.getString("txTimeUs"))!=time)throw new IOException("Duplicate or inconsistent recorded packet identity");
                ordinal=sequence;next=new Transmission(vehicle,new MissionTelemetrySource.Packet(time,ordinal,payload,metadata));return;
            }
        }
    }
    public MissionRecordedSource(Path file, JsonObject definition, Runnable checkpoint) throws IOException {
        archive=new MissionArchive(file);
        try {
            original=MissionJson.parse(archive.read("mission.json",MissionJson.MAX_BYTES));
            String member=definition.getString("member");if(!member.matches("member-[0-9]{1,5}"))throw new IOException("Invalid recorded member");
            prefix="runs/"+member+"/";
            truth=MissionTruth.read(archive,prefix,checkpoint);
            if(definition.getString("telemetrySource").equals("recorded-tx")) {
                for(var value:original.getJsonArray("vehicleBindings")) {
                    String vehicle=value.asJsonObject().getString("id");
                    var names=archive.names().stream().filter(n->n.startsWith(prefix+"transmitters/"+vehicle+"/tx/index-") && n.endsWith(".jsonl")).sorted().toList();
                    if(names.isEmpty())continue;
                    var cursor=new Cursor(vehicle,names);if(cursor.next!=null)cursors.add(cursor);
                }
                if(cursors.isEmpty())throw new IOException("This member has no saved transmissions; explicitly select the standard generator");
            }
        }catch(IOException | RuntimeException e){archive.close();throw e;}
    }
    public long next(){return cursors.isEmpty()?Long.MAX_VALUE:cursors.peek().next.packet().timeUs();}
    public Transmission take() throws IOException {var cursor=cursors.remove();var result=cursor.next;cursor.advance();if(cursor.next!=null)cursors.add(cursor);return result;}
    @Override public void close() throws IOException {archive.close();}
    public static JsonObject packetSettings(JsonObject mission) {
        var telemetry=mission.getJsonObject("telemetry");var noise=telemetry.getJsonObject("noise");
        if(noise==null)noise=JsonValue.EMPTY_JSON_OBJECT;
        var settings=Json.createObjectBuilder(telemetry).remove("links").remove("uplink")
                .add("gpsRateHz",MissionPointer.number(telemetry,"gpsRateHz",10)).add("barometerRateHz",MissionPointer.number(telemetry,"barometerRateHz",20));
        settings.add("noise",Json.createObjectBuilder(noise).add("gpsHorizontalSigmaM",MissionPointer.number(noise,"gpsHorizontalSigmaM",0))
                .add("gpsVerticalSigmaM",MissionPointer.number(noise,"gpsVerticalSigmaM",0)).add("barometerSigmaM",MissionPointer.number(noise,"barometerSigmaM",0)));
        return settings.build();
    }
    public static JsonObject launch(JsonObject mission) {
        var site=mission.getJsonArray("sites").stream().map(JsonValue::asJsonObject).filter(s->s.getString("id").equals("launch-site")).findFirst().orElseThrow();
        return Json.createObjectBuilder().add("latitudeDeg",site.get("latitudeDeg")).add("longitudeDeg",site.get("longitudeDeg"))
                .add("heightM",site.get("heightM")).add("heightDatum",site.get("heightDatum")).build();
    }
}
