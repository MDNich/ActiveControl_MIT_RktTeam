package info.openrocket.core.simulation.ensemble;

/** Standard deviations are newtons for thrust, fractions for pressure, kelvin for temperature and m/s for wind. */
public record EnsembleSettings(boolean enabled, int runs, double motorSigma, double noiseInterval, double temperatureSigma,
                               double pressureSigma, double windSigma, int seed) {
    public static final EnsembleSettings DEFAULT = new EnsembleSettings(false, 30, 5, .05, 2, .01, 1, 12345);
    public EnsembleSettings {
        if (runs < 2 || runs > 10000) throw new IllegalArgumentException("Run count must be between 2 and 10000");
        check(motorSigma, 1e6);
        if (!Double.isFinite(noiseInterval) || noiseInterval < .005 || noiseInterval > 10) throw new IllegalArgumentException("Noise interval must be 0.005–10 s"); check(temperatureSigma, 50); check(pressureSigma, .30); check(windSigma, 100);
    }
    private static void check(double value, double max) {
        if (!Double.isFinite(value) || value < 0 || value > max)
            throw new IllegalArgumentException("Variation must be finite and between 0 and " + max);
    }
    public java.util.Map<String, String> attributes() {
        return java.util.Map.of("enabled", Boolean.toString(enabled), "runs", Integer.toString(runs),
                "motorsigma", Double.toString(motorSigma), "noiseinterval", Double.toString(noiseInterval),
                "temperaturesigma", Double.toString(temperatureSigma), "pressuresigma", Double.toString(pressureSigma),
                "windsigma", Double.toString(windSigma), "seed", Integer.toString(seed));
    }
    public static EnsembleSettings fromAttributes(java.util.Map<String, String> a) {
        if (!"true".equals(a.get("enabled")) && !"false".equals(a.get("enabled"))) throw new IllegalArgumentException("Invalid ensemble enable flag");
        return new EnsembleSettings(Boolean.parseBoolean(a.get("enabled")), Integer.parseInt(a.get("runs")),
                Double.parseDouble(a.get("motorsigma")), Double.parseDouble(a.get("noiseinterval")),
                Double.parseDouble(a.get("temperaturesigma")), Double.parseDouble(a.get("pressuresigma")),
                Double.parseDouble(a.get("windsigma")), Integer.parseInt(a.get("seed")));
    }
    public EnsembleSettings disabled() {
        return new EnsembleSettings(false, runs, motorSigma, noiseInterval, temperatureSigma, pressureSigma, windSigma, seed);
    }
    public String sourceLabel() {
        boolean atmosphere = temperatureSigma > 0 || pressureSigma > 0 || windSigma > 0;
        return motorSigma > 0 ? (atmosphere ? "Motor + atmosphere" : "Motor") : (atmosphere ? "Atmosphere" : "No added variation");
    }
}
