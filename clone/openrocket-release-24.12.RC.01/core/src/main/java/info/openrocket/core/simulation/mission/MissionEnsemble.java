package info.openrocket.core.simulation.mission;

import info.openrocket.core.simulation.ensemble.*;
import jakarta.json.*;
import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;

/** Bounded independent member execution, stable sampling order and explicit partial-failure results. */
public final class MissionEnsemble {
    public record Result(String status, JsonObject metrics) { }
    private MissionEnsemble() { }
    public static Result execute(MissionStore.Resolved resolved, MissionRecorder recorder, Path directory,
                                 MissionRun.Control control, JsonObject preferences) throws Exception {
        var baseline=MissionFlightSnapshot.restore(resolved.flightDirectory());
        var settings=baseline.getOptions().getEnsembleSettings();
        if(!settings.enabled()) return new Result("complete",new MissionRun(resolved,recorder,"member-0",0,control).applicationPreferences(preferences).execute(directory.resolve("member-0")));
        int count=settings.runs();
        boolean paced=resolved.mission().getJsonObject("execution").getString("clockMode","lockstep").equals("paced");
        int requested=resolved.mission().getJsonObject("execution").getInt("parallelMembers",settings.threads());
        if(requested<1 || requested>256)throw new IllegalArgumentException("Mission parallel member count must be 1..256");
        int workers=paced?1:Math.min(Math.min(requested,count),Math.max(1,Math.min(8,Runtime.getRuntime().availableProcessors()/2)));
        for(var extension:baseline.getSimulationExtensions()) if(workers>1 && !info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer.isFlightComputer(extension))
            throw new IllegalArgumentException("Parallel mission ensembles require isolated extensions; select one thread for " + extension.getName());
        boolean failFast=resolved.mission().getJsonObject("execution").getBoolean("failFast",false);
        var executor=Executors.newFixedThreadPool(workers,r->{var t=new Thread(r,"mission-member");t.setDaemon(true);return t;});
        var sampler=new EnsembleSampler(baseline);var pending=new ArrayDeque<Future<JsonObject>>();
        var results=Json.createArrayBuilder();int next=0,complete=0,failures=0;
        try {
            while(complete<count) {
                while(next<count && pending.size()<workers) {
                    control.check();int member=next++;EnsembleRunParameters inputs=null;String error=null;
                    try{inputs=sampler.next();}catch(IllegalArgumentException e){error=e.getMessage();}
                    final EnsembleRunParameters sampled=inputs;final String invalid=error;
                    pending.add(executor.submit(()->{
                        String id="member-"+member;
                        try {
                            if(invalid!=null)throw new IllegalArgumentException(invalid);
                            if(paced)control.beginMember(member);
                            var child=new MissionRun.Control(){
                                public void check(){control.check();}
                                public void progress(long t,long end,String phase){if(paced)control.progress(t,end,"Member "+(member+1)+"/"+count+": "+phase);}
                                public JsonArray drainActions(int max){return paced?control.drainActions(max):JsonValue.EMPTY_JSON_ARRAY;}
                                public void participants(MissionParticipants value){if(paced)control.participants(value);}
                            };
                            var metrics=new MissionRun(resolved,recorder,id,member,child).ensembleMember(sampled).applicationPreferences(preferences).execute(directory.resolve(id));
                            return Json.createObjectBuilder().add("member",member).add("status","complete").add("metrics",metrics).build();
                        } catch(CancellationException e){throw e;}
                        catch(Exception e){
                            var failure=Json.createObjectBuilder().add("member",member).add("status","failed").add("reason",e.getMessage()==null?e.getClass().getSimpleName():e.getMessage()).build();
                            recorder.json("runs/"+id+"/failure.json",failure);return failure;
                        }
                    }));
                }
                JsonObject result;
                try{result=pending.removeFirst().get();}catch(ExecutionException e){if(e.getCause() instanceof CancellationException c)throw c;throw e;}
                results.add(result);complete++;
                if(result.getString("status").equals("failed")){failures++;if(failFast)throw new IOException("Ensemble stopped after member failure: "+result.getString("reason"));}
                if(!paced)control.progress(complete,count,"Completed ensemble member "+complete+"/"+count);
            }
            String status=failures==0?"complete":failures==count?"failed":"complete-with-failures";
            var summary=Json.createObjectBuilder().add("status",status).add("members",count).add("completed",count-failures).add("failed",failures)
                    .add("workers",workers).add("sampling","native-ensemble.v1").add("results",results).build();
            recorder.json("ensemble.json",summary);return new Result(status,summary);
        } finally {
            pending.forEach(f->f.cancel(true));executor.shutdownNow();
            boolean interrupted=Thread.interrupted();while(!executor.isTerminated())try{executor.awaitTermination(1,TimeUnit.SECONDS);}catch(InterruptedException e){interrupted=true;}
            if(interrupted)Thread.currentThread().interrupt();
        }
    }
}
