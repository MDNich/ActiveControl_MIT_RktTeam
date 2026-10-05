package info.openrocket.core.simulation.ensemble;

import info.openrocket.core.simulation.FlightDataBranch;
import java.util.*;

/** Immutable uncertainty and per-flight summary samples accompanying the mean flight data. */
public final class EnsembleResult {
    private final EnsembleSettings settings;
    private final EnsembleRunArchive individualRuns;
    private final List<FlightDataBranch> deviations;
    private final List<Map<EnsembleMetric, double[]>> samples;
    public EnsembleResult(EnsembleSettings settings, List<FlightDataBranch> deviations,
                          List<Map<EnsembleMetric, double[]>> samples) {
        this(settings, deviations, samples, null);
    }
    public EnsembleResult(EnsembleSettings settings, List<FlightDataBranch> deviations,
                          List<Map<EnsembleMetric, double[]>> samples, EnsembleRunArchive individualRuns) {
        if (individualRuns != null && individualRuns.size() != settings.runs())
            throw new IllegalArgumentException("Incomplete individual flight archive");
        this.individualRuns = individualRuns;
        if (deviations.size() != samples.size()) throw new IllegalArgumentException("Mismatched ensemble branches");
        this.settings = settings;
        this.deviations = deviations.stream().map(b -> { var c = b.clone(); c.immute(); return c; }).toList();
        this.samples = samples.stream().map(m -> {
            Map<EnsembleMetric, double[]> copy = new EnumMap<>(EnsembleMetric.class);
            m.forEach((k, v) -> {
                if (v.length != settings.runs()) throw new IllegalArgumentException("Mismatched ensemble run count");
                copy.put(k, v.clone());
            });
            return Collections.unmodifiableMap(copy);
        }).toList();
    }
    /** Null for older files which contain only aggregate data. */
    public EnsembleRunArchive individualRuns() { return individualRuns; }
    public EnsembleSettings settings() { return settings; }
    public int branchCount() { return deviations.size(); }
    public FlightDataBranch deviation(int branch) { return deviations.get(branch); }
    public double[] samples(int branch, EnsembleMetric metric) {
        double[] a = samples.get(branch).get(metric);
        return a == null ? new double[0] : a.clone();
    }
    public double mean(int branch, EnsembleMetric metric) {
        return Arrays.stream(samples(branch, metric)).filter(Double::isFinite).average().orElse(Double.NaN);
    }
    public String description() { return settings.sourceLabel() + ": " + settings.runs() + " runs; mean ±1 sample standard deviation"; }
}
