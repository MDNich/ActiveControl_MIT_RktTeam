package info.openrocket.core.simulation.ensemble;

import info.openrocket.core.document.OpenRocketDocumentFactory;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.core.simulation.FlightData;
import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.simulation.exception.*;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.simulation.listeners.AbstractSimulationListener;
import info.openrocket.core.simulation.listeners.SimulationListener;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicInteger;

/** Bounded parallel flights with deterministic sampling and accumulation in run order. */
public final class EnsembleRunner {
    private EnsembleRunner() {}

    public static FlightData run(Simulation parent, SimulationListener... listeners) throws SimulationException {
        var settings = parent.getOptions().getEnsembleSettings();
        int count = Math.min(settings.threads(), settings.runs());
        for (var extension : parent.getSimulationExtensions()) {
            if (count > 1 && !ZephyrusFlightComputer.isFlightComputer(extension))
                throw new SimulationException("Parallel ensembles cannot isolate extension '" + extension.getName()
                        + "'. Choose 1 thread or remove that extension.");
        }
        var accumulator = new EnsembleAccumulator(settings);
        var sampler = new EnsembleSampler(parent);
        String batch = UUID.randomUUID().toString().substring(0, 8);
        AtomicInteger threadNumber = new AtomicInteger();
        var executor = Executors.newFixedThreadPool(count, job -> {
            var thread = new Thread(job, "ensemble-" + batch + "-" + threadNumber.incrementAndGet());
            thread.setDaemon(true);
            return thread;
        });
        var pending = new ArrayDeque<Future<Member>>();
        System.out.println("ENSEMBLE start " + settings + "; workers=" + count
                + "; thrust=max(0,T(t)+e(t)); fixed per-flight mass/coefficient/wind-bearing offsets; fixed baseline turbulence");
        parent.setEnsembleRunNumber(1);
        try (var archive = new EnsembleRunArchive.Builder()) {
            int next = 0, completed = 0;
            while (completed < settings.runs()) {
                while (pending.size() < count && next < settings.runs()) {
                    if (Thread.currentThread().isInterrupted()) throw new SimulationCancelledException();
                    int number = ++next;
                    EnsembleRunParameters inputs;
                    try {
                        inputs = sampler.next();
                    } catch (IllegalArgumentException e) {
                        throw new SimulationException("Run " + number
                                + ": nonphysical atmosphere or nonpositive mass or coefficient multiplier drawn; "
                                + "reduce variation or change seed. No partial statistics were saved.", e);
                    }
                    // Copy only on the coordinator; workers never read the shared baseline.
                    var rocket = (Rocket) parent.getRocket().copyWithOriginalID();
                    var document = OpenRocketDocumentFactory.createDocumentFromRocket(rocket);
                    var child = parent.duplicateSimulation(document, rocket);
                    document.addSimulation(child);
                    child.setEnsembleRunTag("ensemble-" + batch + "-run-" + number);
                    EnsembleSampler.apply(child, settings, inputs);
                    List<SimulationListener> extra = new ArrayList<>();
                    for (var listener : listeners) extra.add(listener.clone());
                    extra.add(new EnsembleRunListener(settings, inputs));
                    extra.add(new AbstractSimulationListener() {
                        @Override public boolean isSystemListener() { return true; }
                        @Override public void postStep(SimulationStatus status) throws SimulationCancelledException {
                            if (Thread.currentThread().isInterrupted()) throw new SimulationCancelledException();
                        }
                    });
                    pending.addLast(executor.submit(() -> simulate(child, settings, inputs, extra)));
                }
                // Bound live flight histories to the worker count and retain bitwise-stable statistics.
                Member result = pending.removeFirst().get();
                accumulator.add(result.data());
                archive.add(result.inputs(), result.data());
                completed++;
                parent.setEnsembleRunNumber(completed + 1);
                System.out.println("ENSEMBLE saved run=" + completed + "/" + settings.runs());
            }
            var result = accumulator.finish(archive.finish());
            System.out.println("ENSEMBLE complete: " + result.getEnsembleResult().description()
                    + "; full_results_retained=" + settings.runs());
            return result;
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new SimulationCancelledException(e);
        } catch (ExecutionException e) {
            if (e.getCause() instanceof SimulationCancelledException cancelled) throw cancelled;
            if (e.getCause() instanceof SimulationException simulation) throw simulation;
            if (e.getCause() instanceof Error error) throw error;
            throw new SimulationException("Ensemble worker failed; previous results preserved", e.getCause());
        } catch (java.io.IOException e) {
            if (Thread.currentThread().isInterrupted()) throw new SimulationCancelledException(e);
            throw new SimulationException("Could not retain individual ensemble results; previous results preserved", e);
        } finally {
            pending.forEach(future -> future.cancel(true));
            executor.shutdownNow();
            // Close worker output files and listeners before reporting completion/cancellation.
            boolean interrupted = Thread.interrupted();
            while (!executor.isTerminated()) {
                try { executor.awaitTermination(1, TimeUnit.SECONDS); }
                catch (InterruptedException e) { interrupted = true; executor.shutdownNow(); }
            }
            parent.setEnsembleRunNumber(0);
            if (interrupted) Thread.currentThread().interrupt();
        }
    }

    private record Member(EnsembleRunParameters inputs, FlightData data) {}

    private static Member simulate(Simulation child, EnsembleSettings settings, EnsembleRunParameters inputs,
            List<SimulationListener> listeners) throws SimulationException {
        System.out.println("ENSEMBLE run " + inputs.number() + "/" + settings.runs() + " " + inputs);
        System.out.printf(Locale.ROOT, "ENSEMBLE run %d wind_direction_offset_deg=%.9f%n",
                inputs.number(), Math.toDegrees(inputs.windDirectionOffset()));
        try {
            child.simulate(listeners.toArray(SimulationListener[]::new));
        } catch (SimulationCancelledException e) {
            throw e;
        } catch (SimulationException | IllegalArgumentException e) {
            throw new SimulationException("Ensemble stopped at run " + inputs.number() + "/" + settings.runs()
                    + "; no partial statistics were saved. " + e.getMessage(), e);
        }
        System.out.println("ENSEMBLE complete run=" + inputs.number() + " apogee_m=" + child.getSimulatedData().getMaxAltitude());
        if (child.getFlightComputerTelemetryPath() != null)
            System.out.println("ENSEMBLE files run=" + inputs.number() + " csv=" + child.getFlightComputerTelemetryPath()
                    + " log=" + child.getFlightComputerLogPath());
        return new Member(inputs, child.getSimulatedData());
    }
}
