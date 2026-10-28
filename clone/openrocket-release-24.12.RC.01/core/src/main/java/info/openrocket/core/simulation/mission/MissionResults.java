package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.TimeUnit;

/** Bundled stdlib SQLite utility and hash-verified committed-prefix recovery. */
public final class MissionResults {
    private MissionResults() { }
    public static JsonObject utility(Path directory, JsonObject request) throws IOException {
        Files.createDirectories(directory);Path invocation=Files.createTempDirectory(directory,"utility-");
        try { Files.setPosixFilePermissions(invocation,java.nio.file.attribute.PosixFilePermissions.fromString("rwx------")); } catch(UnsupportedOperationException ignored) { }
        Path bundle=invocation.resolve("worker.zip"),input=invocation.resolve("request.json"),output=invocation.resolve("response.json"),error=invocation.resolve("error.log");
        try {
            byte[] bytes;JsonObject metadata;
            try(var in=MissionResults.class.getResourceAsStream("/mission/worker/station-worker.zip")){bytes=in.readNBytes(4*1024*1024);}
            try(var in=MissionResults.class.getResourceAsStream("/mission/worker/worker.json")){metadata=MissionJson.read(in);}
            if(!MissionWorker.digest(bytes).equals(metadata.getString("sha256")))throw new IOException("Bundled results utility hash mismatch");
            Files.write(bundle,bytes);MissionJson.writeAtomic(input,request);
            var process=new ProcessBuilder(MissionWorker.interpreter().toString(),"-I","-c",
                    "import sys,runpy;sys.path.insert(0,sys.argv.pop(1));runpy.run_module('rocket_gnc_monitor.mission_results',run_name='__main__')",
                    bundle.toString(),input.toString(),output.toString()).redirectOutput(ProcessBuilder.Redirect.DISCARD).redirectError(error.toFile()).start();
            try {
                if(!process.waitFor(120,TimeUnit.SECONDS)){process.destroyForcibly();throw new IOException("Result operation timed out; staging remains intact");}
                if(process.exitValue()!=0){String detail=Files.readString(error);throw new IOException("Result operation failed: "+detail.substring(Math.max(0,detail.length()-1500)));}
            }catch(InterruptedException e){process.destroyForcibly();Thread.currentThread().interrupt();throw new IOException("Result operation interrupted",e);}
            try(var in=Files.newInputStream(output)){return MissionJson.read(in);}
        } finally {
            try(var files=Files.walk(invocation)){for(var file:files.sorted(Comparator.reverseOrder()).toList())Files.deleteIfExists(file);}
        }
    }
    public static void index(MissionRecorder recorder,Path directory) throws IOException {
        if (Files.isRegularFile(recorder.root().resolve("index.sqlite"))) return;
        recorder.checkpoint("incomplete");Path file=directory.resolve("index-"+UUID.randomUUID()+".sqlite");
        try {
            utility(directory,Json.createObjectBuilder().add("operation","index").add("root",recorder.root().toString()).add("destination",file.toString()).build());
            recorder.asset("index.sqlite",file);
        } finally {Files.deleteIfExists(file);}
    }
    public static JsonObject query(Path archive,Path directory,JsonObject query) throws IOException {
        try(var check=new MissionArchive(archive)){if(!check.names().contains("index.sqlite"))throw new IOException("This result has no index; recover its staging to build one");}
        return utility(directory,Json.createObjectBuilder().add("operation","query").add("archive",archive.toString()).add("query",query).build());
    }
    public static void recover(Path staging,Path destination,Path directory) throws IOException {
        if(Files.isSymbolicLink(staging))throw new IOException("Recovery staging cannot be a symbolic link");
        JsonObject checkpoint;
        try(var in=Files.newInputStream(staging.resolve("checkpoint.json"))){checkpoint=MissionJson.parse(in.readNBytes(MissionArchive.MAX_MANIFEST+1),MissionArchive.MAX_MANIFEST);}
        if(!checkpoint.getString("format").equals("openrocket.mission-checkpoint") || checkpoint.getInt("version")!=1)throw new IOException("Unsupported checkpoint");
        Path recovered=Files.createTempDirectory(directory,"recovered-");var files=new TreeMap<String,Path>();
        long total=0;
        for(var value:checkpoint.getJsonArray("files")) {
            var entry=value.asJsonObject();String name=entry.getString("path");MissionArchive.safeName(name);
            if(files.size()>=MissionArchive.MAX_ENTRIES-2 || files.containsKey(name))throw new IOException("Invalid recovery inventory");
            long length=entry.getJsonNumber("committedBytes").longValueExact();if(length<0 || length>MissionArchive.MAX_BYTES || (total+=length)>MissionArchive.MAX_BYTES)throw new IOException("Invalid recovery length");
            Path source=staging.resolve(name);for(Path p=source;p!=null && !p.equals(staging);p=p.getParent())if(Files.isSymbolicLink(p))throw new IOException("Linked recovery file");
            if(!Files.isRegularFile(source) || Files.size(source)<length)throw new IOException("Missing committed data: "+name);
            Path target=recovered.resolve(name);Files.createDirectories(target.getParent());
            var digest=sha256();try(var in=Files.newInputStream(source);var out=Files.newOutputStream(target,StandardOpenOption.CREATE_NEW)){
                byte[] buffer=new byte[65536];long remaining=length;while(remaining>0){int n=in.read(buffer,0,(int)Math.min(buffer.length,remaining));if(n<0)throw new EOFException();out.write(buffer,0,n);digest.update(buffer,0,n);remaining-=n;}
            }
            if(!HexFormat.of().formatHex(digest.digest()).equals(entry.getString("sha256")))throw new IOException("Committed prefix hash mismatch: "+name);
            if(!name.equals("index.sqlite"))files.put(name,target);
        }
        var committedFiles=Json.createArrayBuilder();checkpoint.getJsonArray("files").stream().map(JsonValue::asJsonObject).filter(e->!e.getString("path").equals("index.sqlite")).forEach(committedFiles::add);
        var recoveredCheckpoint=Json.createObjectBuilder(checkpoint).add("files",committedFiles).add("status","incomplete").add("recovery","Committed prefixes only; FC state is not resumable").build();
        MissionJson.writeAtomic(recovered.resolve("checkpoint.json"),recoveredCheckpoint,MissionArchive.MAX_MANIFEST);files.put("checkpoint.json",recovered.resolve("checkpoint.json"));
        Path index=recovered.resolve("index.sqlite");Files.deleteIfExists(index);
        utility(directory,Json.createObjectBuilder().add("operation","index").add("root",recovered.toString()).add("destination",index.toString()).build());files.put("index.sqlite",index);
        MissionArchive.write(destination,MissionArchive.RESULT,"incomplete",files);
        try(var paths=Files.walk(recovered)){for(var file:paths.sorted(Comparator.reverseOrder()).toList())Files.delete(file);}
    }
    private static java.security.MessageDigest sha256(){try{return java.security.MessageDigest.getInstance("SHA-256");}catch(java.security.NoSuchAlgorithmException e){throw new IllegalStateException(e);}}
}
