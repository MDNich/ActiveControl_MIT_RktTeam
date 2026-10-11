package info.openrocket.core.simulation.ensemble;

import info.openrocket.core.simulation.FlightDataBranch;
import java.util.*;

/** Immutable uncertainty and per-flight summary samples accompanying the mean flight data. */
public final class EnsembleResult {
    private final EnsembleSettings settings;
    private final EnsembleRunArchive individualRuns;
    private final List<FlightDataBranch> deviations;
    private final List<Map<EnsembleMetric, double[]>> samples;

    /**
     * Creates a EnsembleResult using the supplied settings and defaults for omitted options.
     *
     * @param settings validated configuration for this operation
     * @param deviations deviations
     * @param samples samples
     */
    public EnsembleResult(
            EnsembleSettings settings,
            List<FlightDataBranch> deviations,
            List<Map<EnsembleMetric, double[]>> samples) {
        this(settings, deviations, samples, null);
    }

    /**
     * Creates a EnsembleResult after validating the supplied configuration.
     *
     * @param settings validated configuration for this operation
     * @param deviations initial deviations
     * @param samples initial samples
     * @param individualRuns initial individual runs
     * @throws IllegalArgumentException when rejected: Incomplete individual flight archive
     */
    public EnsembleResult(
            EnsembleSettings settings,
            List<FlightDataBranch> deviations,
            List<Map<EnsembleMetric, double[]>> samples,
            EnsembleRunArchive individualRuns) {
        if (individualRuns != null && individualRuns.size() != settings.runs())
            throw new IllegalArgumentException("Incomplete individual flight archive");
        this.individualRuns = individualRuns;
        if (deviations.size() != samples.size())
            throw new IllegalArgumentException("Mismatched ensemble branches");
        this.settings = settings;
        this.deviations =
                deviations.stream()
                        .map(
                                b -> {
                                    var c = b.clone();
                                    c.immute();
                                    return c;
                                })
                        .toList();
        this.samples =
                samples.stream()
                        .map(
                                m -> {
                                    Map<EnsembleMetric, double[]> copy =
                                            new EnumMap<>(EnsembleMetric.class);
                                    m.forEach(
                                            (k, v) -> {
                                                if (v.length != settings.runs())
                                                    throw new IllegalArgumentException(
                                                            "Mismatched ensemble run count");
                                                copy.put(k, v.clone());
                                            });
                                    return Collections.unmodifiableMap(copy);
                                })
                        .toList();
    }

    /** Null for older files which contain only aggregate data. */
    public EnsembleRunArchive individualRuns() {
        return individualRuns;
    }

    /**
     * Returns the variation settings used to produce this result.
     *
     * @return the variation settings used to produce this result
     */
    public EnsembleSettings settings() {
        return settings;
    }

    /**
     * Returns the number of flight branches represented by the result.
     *
     * @return the number of flight branches represented by the result
     */
    public int branchCount() {
        return deviations.size();
    }

    /**
     * Returns the standard-deviation samples for a branch and plot quantity.
     *
     * @param branch flight-data branch receiving or providing samples
     * @return the standard-deviation samples for a branch and plot quantity
     */
    public FlightDataBranch deviation(int branch) {
        return deviations.get(branch);
    }

    /**
     * Returns the individual scalar samples used by a probability-density plot.
     *
     * @param branch flight-data branch receiving or providing samples
     * @param metric metric
     * @return the individual scalar samples used by a probability-density plot
     */
    public double[] samples(int branch, EnsembleMetric metric) {
        double[] a = samples.get(branch).get(metric);
        return a == null ? new double[0] : a.clone();
    }

    /**
     * Returns the ensemble mean flight data.
     *
     * @param branch flight-data branch receiving or providing samples
     * @param metric metric
     * @return the ensemble mean flight data
     */
    public double mean(int branch, EnsembleMetric metric) {
        return Arrays.stream(samples(branch, metric))
                .filter(Double::isFinite)
                .average()
                .orElse(Double.NaN);
    }

    /**
     * Describes the variation sources and member count represented by the result.
     *
     * @return formatted or resolved text
     */
    public String description() {
        return settings.sourceLabel()
                + ": "
                + settings.runs()
                + " runs; mean ±1 sample standard deviation";
    }
}
