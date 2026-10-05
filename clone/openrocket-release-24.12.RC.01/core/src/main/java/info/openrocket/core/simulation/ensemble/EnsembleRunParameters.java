package info.openrocket.core.simulation.ensemble;

import java.util.Map;

/** Inputs drawn for an individual flight; the parent ensemble records the noise model. */
public record EnsembleRunParameters(int number, long motorSeed, double temperature, double pressure,
                                    double windEast, double windNorth) {
    public EnsembleRunParameters {
        if (number < 1 || number > 10000 || !Double.isFinite(temperature) || temperature <= 0 ||
                !Double.isFinite(pressure) || pressure <= 0 || !Double.isFinite(windEast) || !Double.isFinite(windNorth))
            throw new IllegalArgumentException("Invalid individual run parameters");
    }
    public Map<String, String> attributes() {
        return Map.of("number", Integer.toString(number), "motorseed", Long.toString(motorSeed),
                "temperature", Double.toString(temperature), "pressure", Double.toString(pressure),
                "windeast", Double.toString(windEast), "windnorth", Double.toString(windNorth));
    }
    public static EnsembleRunParameters fromAttributes(Map<String, String> a) {
        return new EnsembleRunParameters(Integer.parseInt(a.get("number")), Long.parseLong(a.get("motorseed")),
                Double.parseDouble(a.get("temperature")), Double.parseDouble(a.get("pressure")),
                Double.parseDouble(a.get("windeast")), Double.parseDouble(a.get("windnorth")));
    }
}
