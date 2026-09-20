package info.openrocket.swing.gui.plot;

import info.openrocket.core.simulation.FlightDataBranch;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.FlightEvent;
import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.Quaternion;
import java.util.*;
import static info.openrocket.core.simulation.FlightDataType.*;

/** Read-only snapshot in OR's east/north/up frame; body +Z is the nose. */
public final class TrajectoryData {
    public record Sample(double time, Coordinate position, Coordinate velocity, Quaternion attitude,
                         double speed, double angularRate, boolean breakBefore) {}
    public record Marker(double time, FlightEvent.Type type, String source) {}
    public record Frame(double time, Coordinate position, Coordinate velocity, Quaternion attitude,
                        double speed, boolean recovery, boolean held, boolean undersampled) {}

    private final List<Sample> samples;
    private final List<Marker> markers;
    private final double[] times;
    private final Coordinate minimum, maximum;
    private final double start, end;
    private final String name;

    public TrajectoryData(FlightDataBranch branch) {
        name = branch.getName();
        Map<FlightDataType, List<Double>> columns = new HashMap<>();
        for (FlightDataType type : branch.getTypes()) columns.put(type, branch.get(type));
        List<Sample> points = new ArrayList<>();
        boolean gap = false;
        for (int i = 0; i < branch.getLength(); i++) {
            double t = value(columns, TYPE_TIME, i);
            if (!Double.isFinite(t)) { gap = true; continue; }
            if (!points.isEmpty() && t < points.get(points.size() - 1).time())
                throw new IllegalArgumentException("Trajectory timestamps run backwards");
            Coordinate p = vector(columns, i, TYPE_POSITION_X, TYPE_POSITION_Y, TYPE_ALTITUDE);
            Coordinate v = vector(columns, i, TYPE_VELOCITY_X, TYPE_VELOCITY_Y, TYPE_VELOCITY_Z);
            Coordinate rate = vector(columns, i, TYPE_ROLL_RATE, TYPE_PITCH_RATE, TYPE_YAW_RATE);
            Quaternion q = quaternion(value(columns, TYPE_ORIENTATION_QW, i), value(columns, TYPE_ORIENTATION_QX, i),
                    value(columns, TYPE_ORIENTATION_QY, i), value(columns, TYPE_ORIENTATION_QZ, i));
            Sample point = new Sample(t, p, v, q, v == null ? value(columns, TYPE_VELOCITY_TOTAL, i) : v.length(),
                    rate == null ? 0 : rate.length(), gap);
            if (!points.isEmpty() && t == points.get(points.size() - 1).time()) {
                // Last position-valid sample wins at a repeated timestamp.
                Sample previous = points.get(points.size() - 1);
                if (p != null) points.set(points.size() - 1, new Sample(t, p, v, q, point.speed(), point.angularRate(), previous.breakBefore() || gap));
            } else points.add(point);
            gap = p == null;
        }
        while (!points.isEmpty() && points.get(0).position() == null) points.remove(0);
        while (!points.isEmpty() && points.get(points.size() - 1).position() == null) points.remove(points.size() - 1);
        if (points.isEmpty()) throw new IllegalArgumentException("No finite trajectory positions are available");
        samples = List.copyOf(points);
        times = samples.stream().mapToDouble(Sample::time).toArray();
        List<Marker> events = new ArrayList<>();
        Set<FlightEvent.Type> visible = EnumSet.of(FlightEvent.Type.LIFTOFF, FlightEvent.Type.BURNOUT,
                FlightEvent.Type.APOGEE, FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT,
                FlightEvent.Type.GROUND_HIT, FlightEvent.Type.STAGE_SEPARATION, FlightEvent.Type.TUMBLE,
                FlightEvent.Type.SIM_ABORT);
        for (FlightEvent event : branch.getEvents()) if (visible.contains(event.getType()) && Double.isFinite(event.getTime()))
            events.add(new Marker(event.getTime(), event.getType(), event.getSource() == null ? "" : event.getSource().getName()));
        events.sort(Comparator.comparingDouble(Marker::time));
        markers = List.copyOf(events);
        start = times[0];
        end = Math.max(start, Math.min(times[times.length - 1], events.stream()
                .filter(e -> e.type() == FlightEvent.Type.GROUND_HIT && e.time() >= start)
                .mapToDouble(Marker::time).min().orElse(times[times.length - 1])));
        double x0 = 0, y0 = 0, z0 = 0, x1 = 0, y1 = 0, z1 = 0;
        for (Sample sample : samples) if (sample.position() != null) {
            Coordinate p = sample.position();
            x0 = Math.min(x0, p.x); y0 = Math.min(y0, p.y); z0 = Math.min(z0, p.z);
            x1 = Math.max(x1, p.x); y1 = Math.max(y1, p.y); z1 = Math.max(z1, p.z);
        }
        minimum = new Coordinate(x0, y0, z0); maximum = new Coordinate(x1, y1, z1);
    }

    private static double value(Map<FlightDataType, List<Double>> columns, FlightDataType type, int i) {
        List<Double> data = columns.get(type);
        return data == null || i >= data.size() || data.get(i) == null ? Double.NaN : data.get(i);
    }
    private static Coordinate vector(Map<FlightDataType, List<Double>> c, int i, FlightDataType x, FlightDataType y, FlightDataType z) {
        double a = value(c, x, i), b = value(c, y, i), d = value(c, z, i);
        return Double.isFinite(a) && Double.isFinite(b) && Double.isFinite(d) ? new Coordinate(a, b, d) : null;
    }
    private static Quaternion quaternion(double w, double x, double y, double z) {
        double norm = Math.sqrt(w*w + x*x + y*y + z*z);
        return !Double.isFinite(norm) || norm < 1e-12 ? null : new Quaternion(w/norm, x/norm, y/norm, z/norm);
    }
    public static Quaternion slerp(Quaternion a, Quaternion b, double fraction) {
        double dot = a.getW()*b.getW() + a.getX()*b.getX() + a.getY()*b.getY() + a.getZ()*b.getZ();
        double sign = dot < 0 ? -1 : 1;
        dot = Math.min(1, Math.abs(dot));
        double left = 1 - fraction, right = fraction;
        if (dot < 0.9995) {
            double theta = Math.acos(dot), sin = Math.sin(theta);
            left = Math.sin((1-fraction)*theta)/sin; right = Math.sin(fraction*theta)/sin;
        }
        return quaternion(left*a.getW()+right*sign*b.getW(), left*a.getX()+right*sign*b.getX(),
                left*a.getY()+right*sign*b.getY(), left*a.getZ()+right*sign*b.getZ());
    }
    private static Coordinate blend(Coordinate a, Coordinate b, double f) {
        return a == null || b == null ? null : a.multiply(1-f).add(b.multiply(f));
    }

    public Frame at(double requestedTime) {
        double time = Math.max(start, Math.min(end, requestedTime));
        int index = Arrays.binarySearch(times, time);
        Sample a, b;
        if (index >= 0) { a = samples.get(index); b = a; }
        else { int next = -index-1; a = samples.get(Math.max(0, next-1)); b = samples.get(Math.min(samples.size()-1, next)); }
        double f = b.time() == a.time() ? 0 : (time-a.time())/(b.time()-a.time());
        boolean missing = a.position() == null || b.position() == null || (a != b && b.breakBefore());
        boolean undersampled = a != b && Math.max(a.angularRate(), b.angularRate()) * (b.time()-a.time()) >= Math.PI;
        Quaternion attitude = a.attitude() == null || b.attitude() == null || undersampled ? null : slerp(a.attitude(), b.attitude(), f);
        boolean recovery = false, held = false;
        for (Marker event : markers) {
            if (event.time() > time) break;
            if (event.type() == FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT) recovery = true;
            if (event.type() == FlightEvent.Type.TUMBLE || event.type() == FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT) held = true;
        }
        return new Frame(time, missing ? null : blend(a.position(), b.position(), f), missing ? null : blend(a.velocity(), b.velocity(), f),
                missing ? null : attitude, a.speed()*(1-f)+b.speed()*f, recovery, held, undersampled);
    }

    public String name() { return name; }
    public List<Sample> samples() { return samples; }
    public List<Marker> markers() { return markers; }
    public Coordinate minimum() { return minimum; }
    public Coordinate maximum() { return maximum; }
    public Coordinate center() { return minimum.add(maximum).multiply(0.5); }
    public double span() { Coordinate d = maximum.sub(minimum); return Math.max(1, Math.max(d.x, Math.max(d.y, d.z))); }
    public double start() { return start; }
    public double end() { return end; }
}
