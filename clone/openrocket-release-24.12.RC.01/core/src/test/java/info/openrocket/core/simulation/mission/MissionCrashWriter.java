package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.nio.file.*;

/** Separate-process durability fixture. The parent terminates this process without close(). */
public final class MissionCrashWriter {
    public static void main(String[] args) throws Exception {
        Path root=Path.of(args[0]);String phase=args[1];
        var recorder=new MissionRecorder(root.resolve("staging"),MissionDefinition.createUrrg("Crash fixture").json());
        for(int i=0;i<100;i++)recorder.event("member-0",Json.createObjectBuilder().add("timeUs",Integer.toString(i)).add("kind","committed").build());
        recorder.checkpoint("incomplete");
        if(phase.equals("finalizing")) {
            Path large=root.resolve("large.bin");byte[] bytes=new byte[1024*1024];var random=new java.util.Random(42);
            try(var out=Files.newOutputStream(large)){for(int i=0;i<128;i++){random.nextBytes(bytes);out.write(bytes);}}
            recorder.asset("assets/durability.bin",large);recorder.checkpoint("incomplete");
            Files.writeString(root.resolve("ready"),phase);
            recorder.finish(root.resolve("result.rktmission"),"complete");
        } else {
            Files.writeString(root.resolve("ready"),phase);
            for(int i=100;;i++) {
                recorder.event("member-0",Json.createObjectBuilder().add("timeUs",Integer.toString(i)).add("kind","tail").build());
                if(phase.equals("checkpoint"))recorder.checkpoint("incomplete");
                Thread.sleep(2);
            }
        }
    }
}
