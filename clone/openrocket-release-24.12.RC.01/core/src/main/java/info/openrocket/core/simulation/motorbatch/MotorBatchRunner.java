package info.openrocket.core.simulation.motorbatch;

import java.time.Instant;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;
import java.util.function.Consumer;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.models.wind.*;
import info.openrocket.core.motor.Motor;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.exception.*;
import info.openrocket.core.simulation.listeners.AbstractSimulationListener;
import static info.openrocket.core.simulation.motorbatch.MotorBatchReport.*;

/** Bounded candidate-level parallelism: no worker ever reads another worker's rocket or flight data. */
public final class MotorBatchRunner {
    public record Progress(MotorBatchReport report,int completedTrials,int totalTrials) { }
    private final MotorBatchSnapshot snapshot;
    private final MotorBatchSettings settings;
    private final List<MotorCandidateSelector.Candidate> candidates;
    private final List<List<Double>> delays=new ArrayList<>();
    private final AtomicReferenceArray<Row> rows;
    private final AtomicBoolean cancelled=new AtomicBoolean();
    private final Set<Thread> running=ConcurrentHashMap.newKeySet();
    private final AtomicInteger active=new AtomicInteger(),peak=new AtomicInteger(),completed=new AtomicInteger();
    private final String id=UUID.randomUUID().toString(),created=Instant.now().toString();
    private int totalTrials;
    private Consumer<Progress> observer=p->{};

    public MotorBatchRunner(MotorBatchSnapshot snapshot,MotorBatchSettings settings,List<MotorCandidateSelector.Candidate> candidates) {
        this.snapshot=snapshot;this.settings=settings;
        var baseline=snapshot.baseline();
        this.candidates=candidates.stream().map(c->MotorCandidateSelector.screen(c.id(),c.motor(),settings,baseline)).toList();
        rows=new AtomicReferenceArray<>(this.candidates.size());
        var mount=MotorBatchSnapshot.target(baseline,settings.mountId());
        double originalDelay=mount.getMotorConfig(baseline.getFlightConfigurationId()).getEjectionDelay();
        boolean eject=baseline.getActiveConfiguration().getActiveComponents().stream().anyMatch(c->c instanceof RecoveryDevice r &&
                r.getDeploymentConfigurations().get(baseline.getFlightConfigurationId()).getDeployEvent()==DeploymentConfiguration.DeployEvent.EJECTION);
        if(settings.delayPolicy()==MotorBatchSettings.DelayPolicy.BEST_LISTED && !eject)
            throw new IllegalArgumentException("Choose best listed delay requires motor-ejection recovery");
        boolean anotherCharge=baseline.getActiveConfiguration().getAllMotors().stream().anyMatch(m->
                !((RocketComponent)m.getMount()).getID().equals(settings.mountId()) && m.getMotor()!=null && Double.isFinite(m.getEjectionDelay()));
        if(settings.delayPolicy()==MotorBatchSettings.DelayPolicy.BEST_LISTED && anotherCharge)
            throw new IllegalArgumentException("Best-delay search requires the varied mount to be the only motor-ejection source. Use Keep baseline delay for this configuration.");
        for(int i=0;i<this.candidates.size();i++) {
            var candidate=this.candidates.get(i);List<Double> ds=List.of();String reason=candidate.exclusion();
            if(reason.isEmpty()) {
                if(settings.delayPolicy()==MotorBatchSettings.DelayPolicy.BEST_LISTED) {
                    ds=Arrays.stream(candidate.motor().getStandardDelays()).filter(x->Double.isFinite(x)&&x>=0).distinct().sorted().boxed().toList();
                    if(ds.isEmpty())reason="DELAY_UNKNOWN_OR_PLUGGED";
                } else if(originalDelay==Motor.PLUGGED_DELAY) {
                    if(eject&&!anotherCharge)reason="PLUGGED_WITH_MOTOR_EJECTION_RECOVERY";
                    else ds=List.of(originalDelay);
                } else if(Arrays.stream(candidate.motor().getStandardDelays()).anyMatch(d->d==originalDelay))ds=List.of(originalDelay);
                else reason="BASELINE_DELAY_NOT_LISTED";
            }
            delays.add(ds);totalTrials+=ds.size();
            rows.set(i,new Row(candidate,List.of(),ds.size(),empty(reason.isEmpty()?Status.NOT_RUN:Status.SCREENED_OUT,
                    reason.isEmpty()?"NOT_RUN":reason)));
        }
    }
    public int totalTrials(){return totalTrials;}
    public boolean isCancelled(){return cancelled.get();}
    public void cancel(){cancelled.set(true);running.forEach(Thread::interrupt);System.out.println("MOTOR_BATCH cancel batch="+id);}
    public MotorBatchReport report(){
        var copy=new ArrayList<Row>();for(int i=0;i<rows.length();i++)copy.add(rows.get(i));
        return new MotorBatchReport(id,created,settings,snapshot.input(),copy,peak.get());
    }
    private void publish(){observer.accept(new Progress(report(),completed.get(),totalTrials));}
    public MotorBatchReport run(Consumer<Progress> observer) throws InterruptedException {
        this.observer=Objects.requireNonNull(observer);
        int count=Math.min(settings.threads(),Math.max(1,candidates.size()));
        AtomicInteger threadNumber=new AtomicInteger();
        var executor=Executors.newFixedThreadPool(count,r->{var t=new Thread(r,"motor-batch-"+id.substring(0,8)+"-"+threadNumber.incrementAndGet());t.setDaemon(true);return t;});
        var completion=new ExecutorCompletionService<Void>(executor);
        System.out.println("MOTOR_BATCH start batch="+id+" threads="+settings.threads()+" candidates="+candidates.size()+" trials="+totalTrials+" settings="+MotorBatchIO.settings(settings));
        for(var row:report().rows()) {
            if(row.selected().status()==Status.SCREENED_OUT)
                System.out.println("MOTOR_BATCH excluded candidate="+row.candidate().id()+" reasons="+row.selected().reasons());
            if(!row.candidate().ignoredIntersections().isEmpty())
                System.out.println("MOTOR_BATCH intersection_override candidate="+row.candidate().id()+" ignored="+row.candidate().ignoredIntersections());
        }
        publish();
        int next=0,pending=0;
        try {
            while(next<candidates.size() || pending>0) {
                while(!cancelled.get() && pending<count && next<candidates.size()) {
                    int index=next++;
                    if(delays.get(index).isEmpty())continue;
                    // Copy on the coordinator; concurrent safety-mutex checks must never see a shared source.
                    var baseline=snapshot.newWorkerBaseline();
                    completion.submit(()->{runCandidate(index,baseline);return null;});pending++;
                }
                if(pending==0)break;
                try {completion.take().get();}
                catch(ExecutionException e) {
                    cancel();
                    if(e.getCause() instanceof Error error)throw error;
                    throw new IllegalStateException("Motor batch worker failed",e.getCause());
                }
                pending--;
            }
        } catch(InterruptedException e) {cancel();throw e;}
        finally {
            executor.shutdown();
            if(cancelled.get())running.forEach(Thread::interrupt);
            // Do not publish a finished batch until its files and listeners have been closed.
            boolean interrupted=false;
            while(!executor.isTerminated())try{executor.awaitTermination(1,TimeUnit.SECONDS);}catch(InterruptedException e){interrupted=true;cancel();}
            if(interrupted)Thread.currentThread().interrupt();
            publish();
        }
        System.out.println("MOTOR_BATCH finished batch="+id+" trials="+completed+" peak_threads="+peak+" cancelled="+cancelled);
        return report();
    }
    private void runCandidate(int index,Simulation baseline) {
        Thread thread=Thread.currentThread();running.add(thread);
        var candidate=candidates.get(index);var trials=new ArrayList<Trial>();
        try {
            for(double delay:delays.get(index)) {
                if(cancelled.get()||thread.isInterrupted())break;
                int trialNumber=trials.size()+1;
                String tag="batch-"+id+"-"+candidate.id()+"-trial-"+trialNumber;
                var sim=MotorBatchSnapshot.trial(baseline,settings,candidate.motor(),delay,tag);
                peak.accumulateAndGet(active.incrementAndGet(),Math::max);
                System.out.println("MOTOR_BATCH trial_start batch="+id+" candidate="+candidate.id()+" motor="+candidate.motor().getDesignation()+" delay="+delay+" thread="+thread.getName());
                Trial trial;
                try {
                    sim.simulate(new TrialListener(cancelled,sim.getOptions().getRandomSeed()));
                    var result=MotorBatchMetrics.evaluate(sim,settings);
                    trial=new Trial(trialNumber,delay,result.complete()?Status.COMPLETE:Status.INCOMPLETE,
                            result.reasons().isEmpty()?Assessment.MEETS_CRITERIA:Assessment.DOES_NOT_MEET_CRITERIA,
                            result.reasons(),result.values(),String.valueOf(sim.getSimulatedWarnings()),"",path(sim.getFlightComputerTelemetryPath()),
                            path(sim.getFlightComputerLogPath()),result.delayDeploymentSpeed(),result.delayApogeeDistance());
                } catch(SimulationCancelledException e) {
                    trial=new Trial(trialNumber,delay,Status.CANCELLED,Assessment.NEEDS_REVIEW,List.of("CANCELLED"),Map.of(),"","",
                            path(sim.getFlightComputerTelemetryPath()),path(sim.getFlightComputerLogPath()),Double.NaN,Double.NaN);
                } catch(Exception e) {
                    trial=new Trial(trialNumber,delay,Status.ERROR,Assessment.NEEDS_REVIEW,List.of("SIMULATION_ERROR"),Map.of(),"",e.toString(),
                            path(sim.getFlightComputerTelemetryPath()),path(sim.getFlightComputerLogPath()),Double.NaN,Double.NaN);
                } finally {active.decrementAndGet();}
                trials.add(trial);completed.incrementAndGet();
                Trial selected=select(trials);
                if(trials.size()<delays.get(index).size() || trials.stream().anyMatch(t->t.status()==Status.CANCELLED))
                    selected=empty(cancelled.get()?Status.CANCELLED:Status.INCOMPLETE,"DELAY_SEARCH_INCOMPLETE");
                rows.set(index,new Row(candidate,trials,delays.get(index).size(),selected));
                System.out.println("MOTOR_BATCH trial_end batch="+id+" candidate="+candidate.id()+" trial="+trialNumber+" status="+trial.status()+" assessment="+trial.assessment()+" reasons="+trial.reasons()+" csv="+trial.telemetryPath()+" log="+trial.logPath());
                publish();
            }
            if(trials.isEmpty()&&cancelled.get())rows.set(index,new Row(candidate,trials,delays.get(index).size(),empty(Status.CANCELLED,"CANCELLED")));
        } finally {running.remove(thread);Thread.interrupted();}
    }
    private static Trial select(List<Trial> trials) {
        return trials.stream().min(Comparator.comparing((Trial t)->t.assessment()!=Assessment.MEETS_CRITERIA)
                .thenComparing(t->t.status()!=Status.COMPLETE)
                .thenComparingDouble(t->Double.isFinite(t.deploymentRank())?t.deploymentRank():Double.POSITIVE_INFINITY)
                .thenComparingDouble(t->Double.isFinite(t.apogeeRank())?t.apogeeRank():Double.POSITIVE_INFINITY)
                .thenComparingDouble(Trial::delay)).orElseThrow();
    }
    private static String path(java.nio.file.Path p){return p==null?"":p.toString();}
    private static final class TrialListener extends AbstractSimulationListener {
        private final AtomicBoolean cancelled;private final int seed;
        TrialListener(AtomicBoolean cancelled,int seed){this.cancelled=cancelled;this.seed=seed;}
        @Override public boolean isSystemListener(){return true;}
        @Override public void startSimulation(SimulationStatus status) throws SimulationException {
            check();var c=status.getSimulationConditions();
            if(c.getWindModel() instanceof PinkNoiseWindModel p)c.setWindModel(p.withSeed(seed));
            else if(c.getWindModel() instanceof MultiLevelPinkNoiseWindModel p)c.setWindModel(p.withSeed(seed));
        }
        @Override public boolean preStep(SimulationStatus s) throws SimulationException {check();return true;}
        @Override public void postStep(SimulationStatus s) throws SimulationException {check();}
        private void check() throws SimulationCancelledException {
            if(cancelled.get()||Thread.currentThread().isInterrupted())throw new SimulationCancelledException();
        }
    }
}
