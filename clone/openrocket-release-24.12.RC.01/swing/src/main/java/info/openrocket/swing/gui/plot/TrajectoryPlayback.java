package info.openrocket.swing.gui.plot;

import java.util.function.LongSupplier;

/** Playback timing depends on elapsed monotonic time, never repaint count. */
public final class TrajectoryPlayback {
    private final LongSupplier clock;
    private double start, end, anchor, speed = 0.25;
    private long anchorNanos;
    private boolean playing;

    public TrajectoryPlayback() { this(System::nanoTime); }
    public TrajectoryPlayback(LongSupplier clock) { this.clock = clock; }
    public void range(double start, double end) { this.start = start; this.end = end; playing = false; seek(start); }
    public double time() {
        double result = anchor + (playing ? (clock.getAsLong()-anchorNanos)/1e9*speed : 0);
        if (result >= end) { result = end; anchor = end; playing = false; }
        return Math.max(start, result);
    }
    public void seek(double time) { anchor = Math.max(start, Math.min(end, time)); anchorNanos = clock.getAsLong(); }
    public void setPlaying(boolean value) { double now = time(); playing = value; seek(now); }
    public void setSpeed(double value) {
        if (!Double.isFinite(value) || value < 0.01 || value > 20) throw new IllegalArgumentException("Playback speed must be between 0.01 and 20");
        double now = time(); speed = value; seek(now);
    }
    public boolean isPlaying() { return playing; }
}
