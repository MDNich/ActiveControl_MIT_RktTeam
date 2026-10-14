package info.openrocket.core.simulation.ensemble;

/**
 * Standard deviations are newtons for thrust, fractions for pressure, kelvin for temperature and
 * m/s for wind speed and radians for wind direction. Mass, drag and roll deviations are
 * fractions (0.1 means 10%).
 */
public record EnsembleSettings(
        boolean enabled,
        int runs,
        double motorSigma,
        double noiseInterval,
        double temperatureSigma,
        double pressureSigma,
        double windSigma,
        int seed,
        double massSigma,
        double dragSigma,
        double rollForceSigma,
        double rollDampingSigma,
        double windDirectionSigma,
        int threads) {
    public static final EnsembleSettings DEFAULT =
            new EnsembleSettings(false, 30, 5, .05, 2, .01, 1, 12345);

    /** Legacy settings have no mass or aerodynamic coefficient uncertainty. */
    public EnsembleSettings(boolean enabled, int runs, double motorSigma, double noiseInterval,
            double temperatureSigma, double pressureSigma, double windSigma, int seed) {
        this(enabled, runs, motorSigma, noiseInterval, temperatureSigma, pressureSigma, windSigma,
                seed, 0, 0, 0, 0, 0, 4);
    }

    public EnsembleSettings(boolean enabled, int runs, double motorSigma, double noiseInterval,
            double temperatureSigma, double pressureSigma, double windSigma, int seed,
            double massSigma, double dragSigma, double rollForceSigma, double rollDampingSigma) {
        this(enabled, runs, motorSigma, noiseInterval, temperatureSigma, pressureSigma, windSigma,
                seed, massSigma, dragSigma, rollForceSigma, rollDampingSigma, 0, 4);
    }

    /**
     * Creates a EnsembleSettings after validating the supplied configuration.
     *
     * @param enabled whether this capability is enabled
     * @param runs runs
     * @param motorSigma motor sigma
     * @param noiseInterval noise interval
     * @param temperatureSigma temperature sigma
     * @param pressureSigma pressure sigma
     * @param windSigma wind sigma
     * @param seed repeatable random seed
     * @param massSigma fractional total-mass standard deviation
     * @param dragSigma fractional overall-drag standard deviation
     * @param rollForceSigma fractional roll-forcing standard deviation
     * @param rollDampingSigma fractional roll-damping standard deviation
     * @param windDirectionSigma wind-bearing standard deviation in radians
     * @param threads maximum concurrent flights (1–256)
     * @throws IllegalArgumentException when rejected: Run count must be between 2 and 10000
     */
    public EnsembleSettings {
        if (runs < 2 || runs > 10000)
            throw new IllegalArgumentException("Run count must be between 2 and 10000");
        check(motorSigma, 1e6);
        if (!Double.isFinite(noiseInterval) || noiseInterval < .005 || noiseInterval > 10)
            throw new IllegalArgumentException("Noise interval must be 0.005–10 s");
        check(temperatureSigma, 50);
        check(pressureSigma, .30);
        check(windSigma, 100);
        check(massSigma, 1);
        check(dragSigma, 1);
        check(rollForceSigma, 1);
        check(rollDampingSigma, 1);
        check(windDirectionSigma, Math.PI);
        if (threads < 1 || threads > 256) throw new IllegalArgumentException("Thread count must be between 1 and 256");
    }

    /**
     * Rejects non-finite or out-of-range ensemble variation settings.
     *
     * @param value replacement or input value
     * @param max upper allowed bound
     * @throws IllegalArgumentException when rejected: Variation must be finite and between 0 and
     */
    private static void check(double value, double max) {
        if (!Double.isFinite(value) || value < 0 || value > max)
            throw new IllegalArgumentException("Variation must be finite and between 0 and " + max);
    }

    /**
     * Serializes ensemble configuration for .ork persistence.
     *
     * @return the resulting Map&lt;String, String&gt;
     */
    public java.util.Map<String, String> attributes() {
        var a = new java.util.LinkedHashMap<String, String>();
        a.put("enabled", Boolean.toString(enabled));
        a.put("runs", Integer.toString(runs));
        a.put("motorsigma", Double.toString(motorSigma));
        a.put("noiseinterval", Double.toString(noiseInterval));
        a.put("temperaturesigma", Double.toString(temperatureSigma));
        a.put("pressuresigma", Double.toString(pressureSigma));
        a.put("windsigma", Double.toString(windSigma));
        a.put("seed", Integer.toString(seed));
        a.put("winddirectionsigma", Double.toString(windDirectionSigma));
        a.put("threads", Integer.toString(threads));
        a.put("masssigma", Double.toString(massSigma));
        a.put("dragsigma", Double.toString(dragSigma));
        a.put("rollforcesigma", Double.toString(rollForceSigma));
        a.put("rolldampingsigma", Double.toString(rollDampingSigma));
        return java.util.Map.copyOf(a);
    }

    /**
     * Reads ensemble settings from saved attributes with compatible defaults.
     *
     * @param a a
     * @return the resulting EnsembleSettings
     * @throws IllegalArgumentException when rejected: Invalid ensemble enable flag
     */
    public static EnsembleSettings fromAttributes(java.util.Map<String, String> a) {
        if (!"true".equals(a.get("enabled")) && !"false".equals(a.get("enabled")))
            throw new IllegalArgumentException("Invalid ensemble enable flag");
        return new EnsembleSettings(
                Boolean.parseBoolean(a.get("enabled")),
                Integer.parseInt(a.get("runs")),
                Double.parseDouble(a.get("motorsigma")),
                Double.parseDouble(a.get("noiseinterval")),
                Double.parseDouble(a.get("temperaturesigma")),
                Double.parseDouble(a.get("pressuresigma")),
                Double.parseDouble(a.get("windsigma")),
                Integer.parseInt(a.get("seed")),
                Double.parseDouble(a.getOrDefault("masssigma", "0")),
                Double.parseDouble(a.getOrDefault("dragsigma", "0")),
                Double.parseDouble(a.getOrDefault("rollforcesigma", "0")),
                Double.parseDouble(a.getOrDefault("rolldampingsigma", "0")),
                Double.parseDouble(a.getOrDefault("winddirectionsigma", "0")),
                Integer.parseInt(a.getOrDefault("threads", "4")));
    }

    /**
     * Returns the default settings with ensemble execution disabled.
     *
     * @return the default settings with ensemble execution disabled
     */
    public EnsembleSettings disabled() {
        return new EnsembleSettings(
                false,
                runs,
                motorSigma,
                noiseInterval,
                temperatureSigma,
                pressureSigma,
                windSigma,
                seed, massSigma, dragSigma, rollForceSigma, rollDampingSigma, windDirectionSigma, threads);
    }

    /**
     * Names the enabled uncertainty sources contributing to the plotted band.
     *
     * @return formatted or resolved text
     */
    public String sourceLabel() {
        var sources = new java.util.ArrayList<String>();
        if (motorSigma > 0) sources.add("Motor");
        if (temperatureSigma > 0 || pressureSigma > 0 || windSigma > 0 || windDirectionSigma > 0) sources.add("Atmosphere");
        if (massSigma > 0) sources.add("Mass");
        if (dragSigma > 0) sources.add("Drag");
        if (rollForceSigma > 0 || rollDampingSigma > 0) sources.add("Roll");
        return sources.isEmpty() ? "No added variation" : String.join(" + ", sources);
    }
}
