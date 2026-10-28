package info.openrocket.core.simulation.ensemble;

import info.openrocket.core.document.Simulation;
import java.util.Random;

/** The historical native ensemble draw order, shared with Mission Simulation. */
public final class EnsembleSampler {
    private final EnsembleSettings settings;
    private final Random atmosphere, coefficients, direction;
    private final double temperature, pressure;
    private int number;
    public EnsembleSampler(Simulation baseline) {
        settings=baseline.getOptions().getEnsembleSettings();
        atmosphere=new Random(settings.seed()); coefficients=new Random(((long)settings.seed())^0x6a09e667f3bcc909L);
        direction=new Random(((long)settings.seed())^0xbb67ae8584caa73bL);
        var base=baseline.getOptions().toSimulationConditions().getAtmosphericModel().getConditions(baseline.getOptions().getLaunchAltitude());
        temperature=base.getTemperature();pressure=base.getPressure();
    }
    public EnsembleRunParameters next() {
        // Draw the complete member even if validation rejects it, so subsequent members stay independent.
        return new EnsembleRunParameters(++number,atmosphere.nextLong(),
                temperature+settings.temperatureSigma()*atmosphere.nextGaussian(),pressure*(1+settings.pressureSigma()*atmosphere.nextGaussian()),
                settings.windSigma()*atmosphere.nextGaussian(),settings.windSigma()*atmosphere.nextGaussian(),
                1+settings.massSigma()*coefficients.nextGaussian(),1+settings.dragSigma()*coefficients.nextGaussian(),
                1+settings.rollForceSigma()*coefficients.nextGaussian(),1+settings.rollDampingSigma()*coefficients.nextGaussian(),
                settings.windDirectionSigma()*direction.nextGaussian());
    }
    public static void apply(Simulation flight,EnsembleSettings settings,EnsembleRunParameters inputs) {
        flight.getOptions().setEnsembleSettings(settings.disabled());flight.getOptions().setRandomSeed(settings.seed());
        if(settings.temperatureSigma()>0 || settings.pressureSigma()>0) {
            flight.getOptions().setISAAtmosphere(false);flight.getOptions().setLaunchTemperature(inputs.temperature());flight.getOptions().setLaunchPressure(inputs.pressure());
        }
    }
}
