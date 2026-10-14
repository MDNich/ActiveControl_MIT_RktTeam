package info.openrocket.core.simulation.ensemble;

import java.util.Map;

/** Inputs drawn for an individual flight; the parent ensemble records the noise model. */
public record EnsembleRunParameters(
        int number,
        long motorSeed,
        double temperature,
        double pressure,
        double windEast,
        double windNorth,
        double massMultiplier,
        double dragMultiplier,
        double rollForceMultiplier,
        double rollDampingMultiplier,
        double windDirectionOffset) {
    /** Runs saved before coefficient uncertainty was introduced use nominal coefficients. */
    public EnsembleRunParameters(int number, long motorSeed, double temperature, double pressure,
            double windEast, double windNorth) {
        this(number, motorSeed, temperature, pressure, windEast, windNorth, 1, 1, 1, 1, 0);
    }

    public EnsembleRunParameters(int number, long motorSeed, double temperature, double pressure,
            double windEast, double windNorth, double massMultiplier, double dragMultiplier,
            double rollForceMultiplier, double rollDampingMultiplier) {
        this(number, motorSeed, temperature, pressure, windEast, windNorth, massMultiplier,
                dragMultiplier, rollForceMultiplier, rollDampingMultiplier, 0);
    }

    /**
     * Creates a EnsembleRunParameters after validating the supplied configuration.
     *
     * @param number number
     * @param motorSeed motor seed
     * @param temperature temperature
     * @param pressure pressure
     * @param windEast wind east
     * @param windNorth wind north
     * @param massMultiplier total mass and inertia scale
     * @param dragMultiplier total drag scale
     * @param rollForceMultiplier roll-forcing scale
     * @param rollDampingMultiplier roll-damping scale
     * @param windDirectionOffset clockwise wind-bearing rotation in radians
     * @throws IllegalArgumentException when rejected: Invalid individual run parameters
     */
    public EnsembleRunParameters {
        if (number < 1
                || number > 10000
                || !Double.isFinite(temperature)
                || temperature <= 0
                || !Double.isFinite(pressure)
                || pressure <= 0
                || !Double.isFinite(windEast)
                || !Double.isFinite(windNorth)
                || !positive(massMultiplier) || !positive(dragMultiplier)
                || !Double.isFinite(windDirectionOffset)
                || !positive(rollForceMultiplier) || !positive(rollDampingMultiplier))
            throw new IllegalArgumentException("Invalid individual run parameters");
    }

    private static boolean positive(double value) {
        return Double.isFinite(value) && value > 0;
    }

    /**
     * Serializes sampled atmosphere, mass/coefficient multipliers and thrust-noise seed.
     *
     * @return the resulting Map&lt;String, String&gt;
     */
    public Map<String, String> attributes() {
        var a = new java.util.LinkedHashMap<String, String>();
        a.put("number", Integer.toString(number));
        a.put("motorseed", Long.toString(motorSeed));
        a.put("temperature", Double.toString(temperature));
        a.put("pressure", Double.toString(pressure));
        a.put("windeast", Double.toString(windEast));
        a.put("windnorth", Double.toString(windNorth));
        a.put("massmultiplier", Double.toString(massMultiplier));
        a.put("dragmultiplier", Double.toString(dragMultiplier));
        a.put("rollforcemultiplier", Double.toString(rollForceMultiplier));
        a.put("rolldampingmultiplier", Double.toString(rollDampingMultiplier));
        a.put("winddirectionoffset", Double.toString(windDirectionOffset));
        return Map.copyOf(a);
    }

    /**
     * Reconstructs one member's random realization from saved attributes.
     *
     * @param a a
     * @return the resulting EnsembleRunParameters
     */
    public static EnsembleRunParameters fromAttributes(Map<String, String> a) {
        return new EnsembleRunParameters(
                Integer.parseInt(a.get("number")),
                Long.parseLong(a.get("motorseed")),
                Double.parseDouble(a.get("temperature")),
                Double.parseDouble(a.get("pressure")),
                Double.parseDouble(a.get("windeast")),
                Double.parseDouble(a.get("windnorth")),
                Double.parseDouble(a.getOrDefault("massmultiplier", "1")),
                Double.parseDouble(a.getOrDefault("dragmultiplier", "1")),
                Double.parseDouble(a.getOrDefault("rollforcemultiplier", "1")),
                Double.parseDouble(a.getOrDefault("rolldampingmultiplier", "1")),
                Double.parseDouble(a.getOrDefault("winddirectionoffset", "0")));
    }
}
