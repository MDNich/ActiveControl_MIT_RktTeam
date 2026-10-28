package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.io.*;
import java.net.*;
import java.net.http.*;
import java.nio.file.*;
import java.time.Duration;

/** Private, locally chosen rendezvous; portable mission files never choose endpoints or credentials. */
public final class MissionAttachment {
    private MissionAttachment() { }
    public static JsonObject check(Path ticket) throws IOException { return exchange(ticket, null); }
    public static JsonObject attach(Path ticket, JsonObject manifest) throws IOException { return exchange(ticket, manifest); }
    private static JsonObject exchange(Path ticket, JsonObject manifest) throws IOException {
        JsonObject config;
        try(var in=Files.newInputStream(ticket)){config=MissionJson.read(in);}
        if(!config.getString("protocol").equals("rt-mission-attach.v1"))throw new IOException("Unsupported application connection file");
        URI uri=URI.create(config.getString("endpoint"));
        if(!"http".equals(uri.getScheme()) || !"127.0.0.1".equals(uri.getHost()) || uri.getPort()<1
                || uri.getUserInfo()!=null || uri.getRawQuery()!=null || uri.getFragment()!=null || !uri.getPath().isEmpty())
            throw new IOException("Application attachment must use a private loopback endpoint");
        var client=HttpClient.newBuilder().connectTimeout(Duration.ofSeconds(5)).followRedirects(HttpClient.Redirect.NEVER).build();
        var info=call(client,uri,config,"/api/v1/info",null);
        MissionWorker.verifyCapabilities(info);
        if(!info.getString("protocol").equals("rt-mission-attach.v1") || !info.getString("bootId").equals(config.getString("bootId"))
                || !info.getString("source_mode").equals("simulation") || !info.getBoolean("ready"))throw new IOException("Application connection is occupied, restarted or not in Mission waiting mode");
        for(String capability:new String[]{"mission-clock.v1","ownership-transfer.v1","declarative-codec.v1"})
            if(!info.getJsonArray("capabilities").contains(Json.createValue(capability)))throw new IOException("Application lacks required capability: "+capability);
        if(manifest!=null)call(client,uri,config,"/mission-api/v1/attach",manifest);
        return info;
    }
    private static JsonObject call(HttpClient client, URI base, JsonObject config, String path, JsonObject body) throws IOException {
        var builder=HttpRequest.newBuilder(base.resolve(path)).timeout(Duration.ofSeconds(10))
                .header("Authorization","Bearer "+config.getString("credential"));
        if(body!=null)builder.header("Content-Type","application/json").POST(HttpRequest.BodyPublishers.ofByteArray(MissionJson.bytes(body)));
        try {
            var response=client.send(builder.build(),HttpResponse.BodyHandlers.ofInputStream());
            try(var in=response.body()){
                var value=MissionJson.read(in);
                if(response.statusCode()!=200 && response.statusCode()!=202)throw new IOException("Application connection refused: "+value.getString("error","HTTP "+response.statusCode()));
                return value;
            }
        }catch(InterruptedException e){Thread.currentThread().interrupt();throw new IOException("Application attachment interrupted",e);}
    }
}
