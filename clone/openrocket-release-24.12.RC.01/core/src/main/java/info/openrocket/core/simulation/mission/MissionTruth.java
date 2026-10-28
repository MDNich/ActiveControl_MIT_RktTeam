package info.openrocket.core.simulation.mission;

import info.openrocket.core.document.Simulation;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.listeners.AbstractSimulationListener;
import info.openrocket.core.simulation.exception.SimulationException;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import jakarta.json.*;
import java.io.*;
import java.util.*;

/** Accepted solver history, fully precomputed before ordinary multi-body link playback. */
public final class MissionTruth extends AbstractSimulationListener {
    public record Sample(long timeUs, double[] ecef, double agl, double velocityUp, double[] quaternion, Set<String> stages) { }
    public static final class Branch {
        public final String id, componentId;
        public final List<Sample> samples = new ArrayList<>();
        public final List<JsonObject> events = new ArrayList<>();
        private boolean landed, aborted;
        private Branch(String component) { componentId = component; id = "stage-" + component; }
        public long start() { return samples.get(0).timeUs; }
        public long end() { return samples.get(samples.size() - 1).timeUs; }
        public boolean landed() { return landed; }
        public boolean aborted() { return aborted; }
    }
    private final IdentityHashMap<FlightDataBranch, Branch> identities = new IdentityHashMap<>();
    private final List<Branch> branches = new ArrayList<>();
    private final Runnable checkpoint;
    public MissionTruth(Runnable checkpoint) { this.checkpoint = checkpoint; }
    public static MissionTruth compute(Simulation flight, Runnable checkpoint, info.openrocket.core.simulation.listeners.SimulationListener... extra) throws SimulationException {
        for (var extension : flight.getSimulationExtensions()) if (extension instanceof ZephyrusFlightComputer fc) { var config = fc.getConfig(); config.put("enabled", false); fc.setConfig(config); }
        var truth = new MissionTruth(checkpoint);
        var listeners = new ArrayList<info.openrocket.core.simulation.listeners.SimulationListener>(Arrays.asList(extra));
        listeners.add(truth);flight.simulate(listeners.toArray(info.openrocket.core.simulation.listeners.SimulationListener[]::new));return truth;
    }
    @Override public boolean isSystemListener() { return true; }
    @Override public void startSimulation(SimulationStatus status) { capture(status); }
    @Override public boolean preStep(SimulationStatus status) { checkpoint.run(); capture(status); return true; }
    @Override public void postStep(SimulationStatus status) { capture(status); }
    @Override public boolean handleFlightEvent(SimulationStatus status, FlightEvent event) {
        Branch branch = branch(status); capture(status);
        if (event.getType() != FlightEvent.Type.ALTITUDE) {
            var json = Json.createObjectBuilder().add("timeUs", Long.toString(Math.round(event.getTime() * 1e6)))
                    .add("kind", event.getType().name()).add("componentId", event.getSource() == null ? "" : event.getSource().getID().toString());
            branch.events.add(json.build());
        }
        if (event.getType() == FlightEvent.Type.GROUND_HIT) branch.landed = true;
        if (event.getType() == FlightEvent.Type.SIM_ABORT || event.getType() == FlightEvent.Type.EXCEPTION) branch.aborted = true;
        return true;
    }
    private Branch branch(SimulationStatus status) {
        return identities.computeIfAbsent(status.getFlightDataBranch(), key -> {
            var active = new ArrayList<>(status.getConfiguration().getActiveStages());
            active.sort(Comparator.comparingInt(s -> s.getStageNumber()));
            if (active.isEmpty()) throw new IllegalArgumentException("Flight branch has no active stage identity");
            var b = new Branch(active.get(0).getID().toString()); branches.add(b); return b;
        });
    }
    private void capture(SimulationStatus status) {
        var world = status.getRocketWorldPosition(); var q = status.getRocketOrientationQuaternion();
        double lat = world.getLatitudeDeg(), lon = world.getLongitudeDeg();
        double h = world.getAltitude() + MissionGeoid.bundled().separation(lat, lon);
        var stages = new LinkedHashSet<String>(); status.getConfiguration().getActiveStages().forEach(s -> stages.add(s.getID().toString()));
        var sample = new Sample(Math.round(status.getSimulationTime() * 1e6), MissionGeodesy.ecef(lat, lon, h),
                status.getRocketPosition().z, status.getRocketVelocity().z, new double[] {q.getW(), q.getX(), q.getY(), q.getZ()}, Set.copyOf(stages));
        Branch branch = branch(status);
        // Keep both sides of discontinuities; exact duplicates add no information.
        if (!branch.samples.isEmpty()) {
            Sample previous = branch.samples.get(branch.samples.size() - 1);
            if (previous.timeUs == sample.timeUs && Arrays.equals(previous.ecef, sample.ecef)
                    && Arrays.equals(previous.quaternion, sample.quaternion) && previous.velocityUp == sample.velocityUp
                    && previous.stages.equals(sample.stages)) return;
        }
        branch.samples.add(sample);
    }
    public List<Branch> branches() { return Collections.unmodifiableList(branches); }
    public long end() { return branches.stream().mapToLong(Branch::end).max().orElseThrow(); }
    public Sample at(JsonObject binding, long timeUs) {
        String component = binding.getString("componentId");
        long t = Math.max(0, timeUs);
        Branch selected = null;
        for (Branch b : branches) if (b.start() <= t) {
            if (b.componentId.equals(component)) { selected = b; break; }
            Sample sample = interpolate(b, t);
            if (sample != null && sample.stages.contains(component)) selected = b;
        }
        Sample result = selected == null ? null : interpolate(selected, t);
        if (result == null) return null;
        if (binding.containsKey("activeStartUs") && timeUs < Long.parseLong(binding.getString("activeStartUs"))) return null;
        if (binding.containsKey("activeEndUs") && timeUs >= Long.parseLong(binding.getString("activeEndUs"))) return null;
        if (!binding.containsKey("offsetBodyM")) return result;
        var offset = binding.getJsonArray("offsetBodyM");
        double[] q = result.quaternion;
        var rotation = new info.openrocket.core.util.Quaternion(q[0], q[1], q[2], q[3]);
        var local = rotation.rotate(new info.openrocket.core.util.Coordinate(offset.getJsonNumber(0).doubleValue(), offset.getJsonNumber(1).doubleValue(), offset.getJsonNumber(2).doubleValue()));
        double[] location = MissionGeodesy.geodetic(result.ecef); double lat = Math.toRadians(location[0]), lon = Math.toRadians(location[1]);
        double[] point = result.ecef.clone();
        point[0] += -Math.sin(lon) * local.x - Math.sin(lat) * Math.cos(lon) * local.y + Math.cos(lat) * Math.cos(lon) * local.z;
        point[1] += Math.cos(lon) * local.x - Math.sin(lat) * Math.sin(lon) * local.y + Math.cos(lat) * Math.sin(lon) * local.z;
        point[2] += Math.cos(lat) * local.y + Math.sin(lat) * local.z;
        return new Sample(result.timeUs, point, result.agl + local.z, result.velocityUp, result.quaternion, result.stages);
    }
    private static Sample interpolate(Branch branch, long t) {
        if (t > branch.end()) return branch.landed && !branch.aborted ? branch.samples.get(branch.samples.size() - 1) : null;
        var list = branch.samples; int low = 0, high = list.size();
        while (low < high) { int mid = (low + high) >>> 1; if (list.get(mid).timeUs <= t) low = mid + 1; else high = mid; }
        if (low == 0) return null;
        Sample a = list.get(low - 1); if (low == list.size() || a.timeUs == t) return a;
        Sample b = list.get(low); double u = (double) (t - a.timeUs) / (b.timeUs - a.timeUs);
        double[] point = new double[3]; for (int i = 0; i < 3; i++) point[i] = a.ecef[i] + u * (b.ecef[i] - a.ecef[i]);
        return new Sample(t, point, a.agl + u * (b.agl - a.agl), a.velocityUp + u * (b.velocityUp - a.velocityUp), slerp(a.quaternion, b.quaternion, u), a.stages);
    }
    static double[] slerp(double[] a, double[] b, double t) {
        double dot = 0; for (int i = 0; i < 4; i++) dot += a[i] * b[i];
        double sign = dot < 0 ? -1 : 1; dot = Math.abs(dot);
        double x = 1 - t, y = t;
        if (dot < .9995) { double theta = Math.acos(Math.min(1, dot)); x = Math.sin((1 - t) * theta) / Math.sin(theta); y = Math.sin(t * theta) / Math.sin(theta); }
        double[] out = new double[4]; double norm = 0;
        for (int i = 0; i < 4; i++) { out[i] = x * a[i] + y * sign * b[i]; norm += out[i] * out[i]; }
        for (int i = 0; i < 4; i++) out[i] /= Math.sqrt(norm); return out;
    }
    public void record(MissionRecorder recorder, String run) throws IOException {
        recorder.json("runs/" + run + "/truth/schema.json", Json.createObjectBuilder().add("version", 1)
                .add("interpolation", "ECEF-linear-quaternion-slerp-post-event.v1")
                .add("columns", "timeUs,branchId,xM,yM,zM,aglM,velocityUpMs,qw,qx,qy,qz,activeStageIds").build());
        for (Branch b : branches) {
            double maxSpacing = 0; long previous = b.start();
            for (Sample s : b.samples) {
                maxSpacing = Math.max(maxSpacing, (s.timeUs - previous) / 1e6); previous = s.timeUs;
                String line = s.timeUs + "," + b.id;
                for (double v : s.ecef) line += "," + Double.toString(v);
                line += "," + s.agl + "," + s.velocityUp;
                for (double v : s.quaternion) line += "," + Double.toString(v);
                line += "," + String.join(";", new TreeSet<>(s.stages)) + "\n";
                recorder.line("runs/" + run + "/truth", "csv", s.timeUs, line);
            }
            for (var event : b.events) recorder.event(run, Json.createObjectBuilder(event).add("branchId", b.id).build());
            String parent = "";
            if (b.start() > 0) for (Branch candidate : branches) if (candidate != b && candidate.start() < b.start()) {
                Sample attached = interpolate(candidate, b.start() - 1);
                if (attached != null && attached.stages.contains(b.componentId)) parent = candidate.id;
            }
            recorder.event(run, Json.createObjectBuilder().add("parentBranchId", parent).add("kind", "branch-summary").add("timeUs", Long.toString(b.end()))
                    .add("branchId", b.id).add("componentId", b.componentId).add("startUs", Long.toString(b.start()))
                    .add("landed", b.landed).add("aborted", b.aborted).add("maxSampleSpacingS", maxSpacing).build());
        }
    }
}
