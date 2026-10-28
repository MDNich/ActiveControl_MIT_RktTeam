package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.*;
import java.nio.file.*;
import java.util.*;
import javax.imageio.ImageIO;

/** Static per-run slates and deterministic logical video delivery, independent of wall-clock encoders. */
public final class MissionVideo {
    private final JsonObject mission, config;
    private final MissionRecorder recorder;
    private final String run;
    private final long member;
    private final Map<String, JsonObject> streams = new TreeMap<>(), assets = new TreeMap<>();
    private final Map<String, JsonObject> states = new TreeMap<>();
    private final PriorityQueue<JsonObject> pending = new PriorityQueue<>(Comparator.comparingLong((JsonObject o) -> Long.parseLong(o.getString("timeUs"))).thenComparing(o -> o.getString("streamId")).thenComparing(o -> o.getString("siteId")));
    private long ordinal, next = -1_000_000, end = Long.MAX_VALUE;
    private final double fps;
    private final Set<String> enabledSites = new HashSet<>();
    public MissionVideo(JsonObject mission, MissionRecorder recorder, String run, long member) throws IOException {
        this.mission = mission; this.recorder = recorder; this.run = run; this.member = member; config = mission.getJsonObject("video");
        validate(config); fps = number(config,"frameRateHz",30);
        mission.getJsonArray("sites").forEach(s -> { if(s.asJsonObject().getBoolean("enabled")) enabledSites.add(s.asJsonObject().getString("id")); });
        int digital=0, analog=0;
        for (var value : config.getJsonArray("streams")) {
            var stream=value.asJsonObject(); String id=stream.getString("id"); streams.put(id,stream);
            int n=stream.getString("kind").equals("digital") ? ++digital : ++analog;
            assets.put(id, slate(stream,n));
        }
    }
    public static void validate(JsonObject config) {
        if (config.getJsonArray("streams").size()>64) throw new IllegalArgumentException("At most 64 logical video streams");
        for (String key: java.util.List.of("lossFraction","analogNoiseFraction")) {
            double n=number(config,key,0); if(n<0 || n>1 || !Double.isFinite(n)) throw new IllegalArgumentException("Invalid video " + key);
        }
        double fps=number(config,"frameRateHz",30), delay=number(config,"delayMs",0);
        if(fps<1 || fps>60 || !Double.isFinite(fps) || delay<0 || delay>60_000 || !Double.isFinite(delay)) throw new IllegalArgumentException("Invalid video rate or delay");
        // The transport contract fixes dimensions; keep imported alternatives explicit.
        if(number(config,"width",960)!=960 || number(config,"height",540)!=540) throw new IllegalArgumentException("Mission video slates are 960 x 540 pixels");
        var ids=new HashSet<String>();

        for(var value:config.getJsonArray("streams")) {
            var stream=value.asJsonObject(); String id=stream.getString("id");
            if(!id.matches("[A-Za-z0-9][A-Za-z0-9._-]{0,63}") || !ids.add(id)) throw new IllegalArgumentException("Invalid or duplicate stream ID");
            var receivers=stream.getJsonArray("receivers");
            if(receivers.size()>4 || new HashSet<>(receivers).size()!=receivers.size()) throw new IllegalArgumentException("Video routes must contain up to four distinct Away receivers");
        }
        var receiverIds=new HashSet<String>();
        var profiles=config.getJsonArray("receiverProfiles");
        if(profiles!=null)for(var value:profiles){var p=value.asJsonObject();String id=p.getString("receiverId");
            if(!Set.of("gs1","gs2","gs3","gs4").contains(id) || !receiverIds.add(id))throw new IllegalArgumentException("Video receiver profiles must identify distinct Away stations");
            for(String key:java.util.List.of("lossFraction","analogNoiseFraction")){double n=number(p,key,0);if(!Double.isFinite(n)||n<0||n>1)throw new IllegalArgumentException("Invalid receiver video "+key);}
            double d=number(p,"delayMs",0);if(!Double.isFinite(d)||d<0||d>60_000)throw new IllegalArgumentException("Invalid receiver video delay");
            MissionLink.validate(Json.createObjectBuilder().add("receiverId",id).add("outages",p.getJsonArray("outages")==null?JsonValue.EMPTY_JSON_ARRAY:p.getJsonArray("outages")).build());
        }
        double queued=0;
        for(var value:config.getJsonArray("streams"))for(var site:value.asJsonObject().getJsonArray("receivers"))queued+=fps*(number(profile(config,((JsonString)site).getString()),"delayMs",0)/1000+1);
        if(queued > 100_000) throw new IllegalArgumentException("Video rate, routes and delay exceed the 100,000 pending-frame limit; reduce one of these settings");
        MissionLink.validate(Json.createObjectBuilder().add("receiverId","video").add("outages",config.getJsonArray("outages")==null?JsonValue.EMPTY_JSON_ARRAY:config.getJsonArray("outages")).build());
    }
    private static JsonObject profile(JsonObject config,String site) {
        if(config.getJsonArray("receiverProfiles")!=null)for(var value:config.getJsonArray("receiverProfiles")){
            var p=value.asJsonObject();if(p.getString("receiverId").equals(site) && p.getBoolean("enabled",false)){
                var merged=Json.createObjectBuilder(config);for(String key:java.util.List.of("lossFraction","delayMs","analogNoiseFraction","outages"))if(p.containsKey(key))merged.add(key,p.get(key));return merged.build();
            }
        }
        return config;
    }
    private JsonObject slate(JsonObject stream,int number) throws IOException {
        byte[] fontBytes; try(var in=getClass().getResourceAsStream("/mission/video/DejaVuSans.ttf")){fontBytes=in.readAllBytes();}
        Font font; try {font=Font.createFont(Font.TRUETYPE_FONT,new ByteArrayInputStream(fontBytes));} catch(FontFormatException e){throw new IOException(e);}
        var image=new BufferedImage(960,540,BufferedImage.TYPE_INT_RGB); var g=image.createGraphics();
        g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING,RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
        g.setColor(new Color(16,40,43)); g.fillRect(0,0,960,540); g.setColor(new Color(119,211,189));g.fillRect(0,0,960,12);
        text(g,font,"MISSION SIMULATION",25,69);g.setColor(Color.WHITE);
        text(g,font,stream.getString("kind").equals("digital") ? "Digital " + number : "Analog " + number,70,182);
        text(g,font,mission.getJsonObject("identity").getString("title"),36,260);g.setColor(new Color(179,203,197));
        text(g,font,stream.getString("vehicleId") + " / " + stream.getString("id"),25,318);
        text(g,font,"Run " + run + " / member " + member,25,391);
        text(g,font,mission.getJsonObject("flight").getString("filename","rocket.ork"),21,443);
        text(g,font,MissionStore.id(mission),16,496);g.dispose();
        Path file=Files.createTempFile("mission-slate-",".png");
        String name="runs/" + run + "/video/assets/" + stream.getString("id") + ".png";
        String sha;
        try {ImageIO.write(image,"png",file.toFile());sha=MissionFlightSnapshot.hash(file);recorder.asset(name,file);} finally {Files.deleteIfExists(file);}
        var metadata=Json.createObjectBuilder(stream).add("asset",name).add("sha256",sha).add("width",960).add("height",540)
                .add("template","mission-slate.v1").add("fontSha256",MissionWorker.digest(fontBytes)).add("ordinal",number).build();
        recorder.json("runs/" + run + "/video/assets/" + stream.getString("id") + ".json",metadata);return metadata;
    }
    private static void text(Graphics2D g,Font font,String value,float size,int y) {
        g.setFont(font.deriveFont(size)); int width=g.getFontMetrics().stringWidth(value);
        if(width>875)g.setFont(font.deriveFont(Math.max(8,size*875/width)));g.drawString(value,42,y);
    }
    public JsonArray applicationStreams() {
        var result=Json.createArrayBuilder(); assets.values().forEach(a -> result.add(Json.createObjectBuilder(a).add("assetPath",recorder.root().resolve(a.getString("asset")).toString())));return result.build();
    }
    public void end(long time){end=time;}
    public long next(){return Math.min(!streams.isEmpty() && next<=end ? next : Long.MAX_VALUE,pending.isEmpty()?Long.MAX_VALUE:Long.parseLong(pending.peek().getString("timeUs")));}
    public void advance(long time) throws IOException {
        if(!streams.isEmpty() && time==next && time<=end) {
            for(var stream:streams.values()) for(var receiver:stream.getJsonArray("receivers")) {
                String site=((JsonString)receiver).getString(),id=stream.getString("id");
                if(!enabledSites.contains(site))continue;
                var reception=profile(config,site);
                boolean outage=false;for(var value:reception.getJsonArray("outages")==null?JsonValue.EMPTY_JSON_ARRAY:reception.getJsonArray("outages")) {
                    var interval=value.asJsonObject();outage|=time>=Long.parseLong(interval.getString("startUs")) && time<Long.parseLong(interval.getString("endUs"));
                }
                boolean lost=outage || MissionRandom.uniform(mission.getJsonObject("execution").getString("seed"),member,id+":"+site,"video-loss","video.v1",ordinal,0)<number(reception,"lossFraction",0);
                var event=Json.createObjectBuilder().add("streamId",id).add("siteId",site).add("frame",Long.toString(ordinal)).add("captureTimeUs",Long.toString(time))
                        .add("timeUs",Long.toString(time)).add("outcome",outage?"outage":lost?"lost":"scheduled").add("illustrativeAnalogNoise",stream.getString("kind").equals("analog")?number(reception,"analogNoiseFraction",0):0).build();
                recorder.jsonLine("runs/"+run+"/video/events",event);
                if(!lost) pending.add(Json.createObjectBuilder(event).add("outcome","delivered").add("timeUs",Long.toString(time+Math.round(number(reception,"delayMs",0)*1000))).build());
            }
            ordinal++;next=-1_000_000+Math.round(ordinal*1e6/fps);
        }
        while(!pending.isEmpty() && Long.parseLong(pending.peek().getString("timeUs"))<=time) {
            var event=pending.remove();recorder.jsonLine("runs/"+run+"/video/events",event);states.put(event.getString("siteId")+":"+event.getString("streamId"),event);
        }
    }
    public JsonArray state(String site) {
        var result=Json.createArrayBuilder();for(var event:states.values())if(event.getString("siteId").equals(site))result.add(event);return result.build();
    }
    private static double number(JsonObject o,String k,double f){return MissionPointer.number(o,k,f);}
}
