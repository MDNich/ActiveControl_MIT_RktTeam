package info.openrocket.core.simulation.ensemble;

import info.openrocket.core.document.Simulation;
import info.openrocket.core.simulation.FlightData;
import info.openrocket.core.simulation.exception.*;
import info.openrocket.core.simulation.listeners.SimulationListener;
import java.util.*;

public final class EnsembleRunner {
    private EnsembleRunner() { }
    public static FlightData run(Simulation parent, SimulationListener... listeners) throws SimulationException {
        var settings=parent.getOptions().getEnsembleSettings();
        var accumulator=new EnsembleAccumulator(settings);
        var random=new Random(settings.seed());
        String batch=java.util.UUID.randomUUID().toString().substring(0,8);
        var base=parent.getOptions().toSimulationConditions().getAtmosphericModel().getConditions(parent.getOptions().getLaunchAltitude());
        System.out.println("ENSEMBLE start " + settings + "; thrust=max(0,T(t)+e(t)); unchanged burn time/mass; fixed baseline turbulence");
        try (var archive = new EnsembleRunArchive.Builder()) {
            for (int i=0;i<settings.runs();i++) {
                if (Thread.currentThread().isInterrupted()) throw new SimulationCancelledException();
                parent.setEnsembleRunNumber(i+1);
                var child=parent.copy();
                child.setEnsembleRunTag("ensemble-"+batch+"-run-"+(i+1));
                child.getOptions().setEnsembleSettings(settings.disabled());
                child.getOptions().setRandomSeed(settings.seed());
                // Draw all sources even when sigma=0, keeping other sources identical when toggled.
                long motorSeed=random.nextLong();
                double temp=base.getTemperature()+settings.temperatureSigma()*random.nextGaussian();
                double pressure=base.getPressure()*(1+settings.pressureSigma()*random.nextGaussian());
                double east=settings.windSigma()*random.nextGaussian(), north=settings.windSigma()*random.nextGaussian();
                // Reject impossible atmosphere draws explicitly, never silently censor flight outcomes.
                if (temp<=0 || pressure<=0) throw new SimulationException("Run " +(i+1)+": nonphysical atmosphere draw; reduce variation or change seed");
                if (settings.temperatureSigma()>0 || settings.pressureSigma()>0) {
                    child.getOptions().setISAAtmosphere(false);
                    child.getOptions().setLaunchTemperature(temp);
                    child.getOptions().setLaunchPressure(pressure);
                }
                List<SimulationListener> extra=new ArrayList<>();
                for (var l:listeners) extra.add(l.clone());
                extra.add(new EnsembleRunListener(settings,motorSeed,east,north));
                System.out.printf(Locale.ROOT,"ENSEMBLE run %d/%d motor_seed=%d temperature_K=%.6f pressure_Pa=%.6f wind_E_mps=%.6f wind_N_mps=%.6f%n",i+1,settings.runs(),motorSeed,temp,pressure,east,north);
                try {
                    child.simulate(extra.toArray(SimulationListener[]::new));
                    accumulator.add(child.getSimulatedData());
                    archive.add(new EnsembleRunParameters(i+1, motorSeed, temp, pressure, east, north), child.getSimulatedData());
                } catch (SimulationCancelledException e) { throw e; }
                catch (SimulationException | IllegalArgumentException e) {
                    throw new SimulationException("Ensemble stopped at run "+(i+1)+"/"+settings.runs()+"; no partial statistics were saved. "+e.getMessage(),e);
                }
                System.out.println("ENSEMBLE complete run="+(i+1)+" apogee_m="+child.getSimulatedData().getMaxAltitude());
                if (child.getFlightComputerTelemetryPath() != null)
                    System.out.println("ENSEMBLE files run="+(i+1)+" csv="+child.getFlightComputerTelemetryPath()+" log="+child.getFlightComputerLogPath());
            }
            var result=accumulator.finish(archive.finish());
            System.out.println("ENSEMBLE complete: "+result.getEnsembleResult().description()+"; full_results_retained="+settings.runs());
            return result;
        } catch (java.io.IOException e) {
            if (Thread.currentThread().isInterrupted()) throw new SimulationCancelledException(e);
            throw new SimulationException("Could not retain individual ensemble results; the previous simulation result is preserved", e);
        } finally { parent.setEnsembleRunNumber(0); }
    }
}
