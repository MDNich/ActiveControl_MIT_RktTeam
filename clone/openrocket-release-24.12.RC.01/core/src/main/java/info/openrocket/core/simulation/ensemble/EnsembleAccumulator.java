package info.openrocket.core.simulation.ensemble;

import info.openrocket.core.simulation.*;
import info.openrocket.core.logging.WarningSet;
import java.util.*;

/** Streaming moments: memory grows with one flight, not the number of runs. */
public final class EnsembleAccumulator {
    private static final FlightDataType[] Q = {FlightDataType.TYPE_ORIENTATION_QW, FlightDataType.TYPE_ORIENTATION_QX,
            FlightDataType.TYPE_ORIENTATION_QY, FlightDataType.TYPE_ORIENTATION_QZ};
    private final EnsembleSettings settings;
    private final List<Branch> branches = new ArrayList<>();
    private final WarningSet warnings = new WarningSet();
    private int runs;
    public EnsembleAccumulator(EnsembleSettings settings) { this.settings = settings; }
    public void add(FlightData flight) {
        if (runs >= settings.runs()) throw new IllegalStateException("Too many ensemble runs");
        if (flight == null || flight.getBranchCount() == 0) throw new IllegalArgumentException("Run has no flight data");
        for (var branch : flight.getBranches()) {
            if (branch.getFirstEvent(FlightEvent.Type.SIM_ABORT) != null)
                throw new IllegalArgumentException("Run aborted: " + branch.getFirstEvent(FlightEvent.Type.SIM_ABORT).getData());
        }
        if (runs == 0) for (var b : flight.getBranches()) branches.add(new Branch(b));
        if (flight.getBranchCount() != branches.size()) throw new IllegalArgumentException("Stage branches differ between runs");
        for (int i = 0; i < branches.size(); i++) branches.get(i).add(flight.getBranch(i));
        warnings.addAll(flight.getWarningSet());
        runs++;
    }
    public FlightData finish() { return finish(null); }
    public FlightData finish(EnsembleRunArchive individualRuns) {
        if (runs != settings.runs()) throw new IllegalStateException("Incomplete ensemble");
        List<FlightDataBranch> means = new ArrayList<>(), deviations = new ArrayList<>();
        List<Map<EnsembleMetric, double[]>> samples = new ArrayList<>();
        for (var b : branches) {
            var mean = new FlightDataBranch(b.name, b.moments.keySet().toArray(FlightDataType[]::new));
            var sd = new FlightDataBranch(b.name, mean.getTypes());
            for (int i = 0; i < b.time.length; i++) {
                double t = b.time[i];
                if (t < b.start || t > b.end) continue;
                mean.addPoint(); sd.addPoint();
                for (var entry : b.moments.entrySet()) {
                    var type = entry.getKey(); var m = entry.getValue();
                    mean.setValue(type, type == FlightDataType.TYPE_TIME ? t : type.equals(info.openrocket.core.simulation.flightcomputer.FlightComputerData.STATE)&&m.sd(i,runs)>0 ? -1 : m.mean(i, runs));
                    sd.setValue(type, type == FlightDataType.TYPE_TIME ? t : m.sd(i, runs));
                }
                // Keep component means for valid component bands. The 3D reader normalizes
                // this hemisphere-aligned quaternion when constructing its attitude.

            }
            if (mean.getLength() < 2) throw new IllegalArgumentException("Runs have no common flight time interval");
            for (var event : b.events.values()) if (event.count == runs) {
                double time = event.sum / runs;
                if (time >= b.start && time <= b.end) mean.addEvent(new FlightEvent(event.example.getType(), time, event.example.getSource()));
            }
            mean.immute(); sd.immute(); means.add(mean); deviations.add(sd); samples.add(b.samples);
        }
        var data = new FlightData(means.toArray(FlightDataBranch[]::new));
        data.getWarningSet().addAll(warnings);
        data.setEnsembleResult(new EnsembleResult(settings, deviations, samples, individualRuns));
        data.immute(); return data;
    }
    private final class Branch {
        final String name;
        final double[] time;
        double start, end;
        final Map<FlightDataType, Moments> moments = new LinkedHashMap<>();
        final Map<EnsembleMetric, double[]> samples = new EnumMap<>(EnsembleMetric.class);
        final Map<String, EventMean> events = new LinkedHashMap<>();
        Branch(FlightDataBranch b) {
            name = b.getName();
            var ts = b.get(FlightDataType.TYPE_TIME);
            if (ts == null || ts.size() < 2) throw new IllegalArgumentException("Run has no trajectory");
            time = ts.stream().mapToDouble(Double::doubleValue).distinct().toArray();
            start = time[0]; end = time[time.length - 1];
            for (var metric : EnsembleMetric.values()) samples.put(metric, new double[settings.runs()]);
        }
        void add(FlightDataBranch b) {
            if (!name.equals(b.getName())) throw new IllegalArgumentException("Stage identities differ between runs");
            var ts = b.get(FlightDataType.TYPE_TIME);
            if (ts == null || ts.size() < 2) throw new IllegalArgumentException("Run has no trajectory");
            for (int i = 0; i < ts.size(); i++) if (!Double.isFinite(ts.get(i)) || (i > 0 && ts.get(i) < ts.get(i-1)))
                throw new IllegalArgumentException("Run contains invalid time ordering");
            start = Math.max(start, ts.get(0)); end = Math.min(end, ts.get(ts.size()-1));
            var ground = b.getFirstEvent(FlightEvent.Type.GROUND_HIT);
            if (ground != null) end = Math.min(end, ground.getTime());
            Map<FlightDataType, List<Double>> columns = new HashMap<>();
            for (var type : b.getTypes()) {
                moments.computeIfAbsent(type, k -> new Moments(time.length, circular(k)));
                columns.put(type, b.get(type));
            }
            int lo = 0;
            for (int i = 0; i < time.length; i++) {
                double t = time[i]; if (t < start || t > end) continue;
                while (lo + 1 < ts.size() && ts.get(lo + 1) <= t) lo++;
                int hi = Math.min(lo+1, ts.size()-1);
                double fraction = hi == lo ? 0 : (t - ts.get(lo)) / (ts.get(hi) - ts.get(lo));
                for (var e : moments.entrySet()) {
                    if (Arrays.asList(Q).contains(e.getKey())) continue;
                    var values = columns.get(e.getKey());
                    if (values == null) continue;
                    e.getValue().add(i, e.getKey().equals(info.openrocket.core.simulation.flightcomputer.FlightComputerData.STATE)?values.get(lo):interpolate(values.get(lo), values.get(hi), fraction, circular(e.getKey())));
                }
                double[] q = new double[4]; double dot = 0, norm = 0;
                boolean available = true;
                for (var type : Q) if (!columns.containsKey(type)) available = false;
                if (available) {
                    for (var type : Q) dot += columns.get(type).get(lo) * columns.get(type).get(hi);
                    for (int k=0;k<4;k++) {
                        q[k] = interpolate(columns.get(Q[k]).get(lo), (dot < 0 ? -1 : 1) * columns.get(Q[k]).get(hi), fraction, false);
                        norm += q[k]*q[k];
                    }
                    norm = Math.sqrt(norm); double meanDot = 0;
                    for (int k=0;k<4;k++) meanDot += q[k] * moments.get(Q[k]).mean[i];
                    for (int k=0;k<4;k++) moments.get(Q[k]).add(i, norm > 1e-12 ? q[k]/norm*(meanDot < 0 ? -1 : 1) : Double.NaN);
                }
            }
            var single = new FlightData(b);
            for (var metric : EnsembleMetric.values()) samples.get(metric)[runs] = metric.value(single);
            Map<String, Integer> occurrences = new HashMap<>();
            for (var event : b.getEvents()) {
                // These events carry structured data, not representative phase changes.
                if (event.getType() == FlightEvent.Type.ALTITUDE || event.getType() == FlightEvent.Type.SIM_WARN || event.getType() == FlightEvent.Type.SIM_ABORT) continue;
                String base = event.getType().name() + ":" + (event.getSource() == null ? "" : event.getSource().getID());
                String key = base + ":" + occurrences.merge(base, 1, Integer::sum);
                var e = events.computeIfAbsent(key, k -> new EventMean(event)); e.sum += event.getTime(); e.count++;
            }
        }
    }
    private static class EventMean {
        final FlightEvent example; double sum; int count;
        EventMean(FlightEvent example) { this.example = example; }
    }
    static boolean circular(FlightDataType t) {
        return t == FlightDataType.TYPE_POSITION_DIRECTION || t == FlightDataType.TYPE_WIND_DIRECTION || t == FlightDataType.TYPE_ORIENTATION_PHI;
    }
    static double interpolate(double a, double b, double f, boolean circular) {
        if (f == 0) return a;
        if (f == 1) return b;
        double delta = circular ? Math.atan2(Math.sin(b-a), Math.cos(b-a)) : b-a;
        return a + f*delta;
    }
    private static class Moments {
        final double[] mean, m2, sine, cosine; final int[] count; final boolean circular;
        Moments(int n, boolean circular) { mean=new double[n];m2=new double[n];count=new int[n];this.circular=circular;sine=circular?new double[n]:null;cosine=circular?new double[n]:null; }
        void add(int i, double x) {
            if (!Double.isFinite(x)) return;
            int n=++count[i]; double delta=x-mean[i]; mean[i]+=delta/n; m2[i]+=delta*(x-mean[i]);
            if (circular) { sine[i]+=Math.sin(x);cosine[i]+=Math.cos(x); }
        }
        double mean(int i, int n) { return count[i]!=n ? Double.NaN : circular ? Math.atan2(sine[i],cosine[i]) : mean[i]; }
        double sd(int i, int n) {
            if (count[i]!=n || n<2) return Double.NaN;
            if (circular) return Math.sqrt(-2*Math.log(Math.min(1,Math.hypot(sine[i],cosine[i])/n))*n/(n-1));
            return Math.sqrt(Math.max(0,m2[i]/(n-1)));
        }
    }
}
