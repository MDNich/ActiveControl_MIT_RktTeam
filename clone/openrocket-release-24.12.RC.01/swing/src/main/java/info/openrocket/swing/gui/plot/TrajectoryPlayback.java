package info.openrocket.swing.gui.plot;

import java.util.function.LongSupplier;

/** Playback timing depends on elapsed monotonic time, never repaint count. */
public final class TrajectoryPlayback {
    private final LongSupplier clock;
    private double start, end, anchor, speed = 0.25;
    private long anchorNanos;
    private boolean playing;

    /**
     * Creates a TrajectoryPlayback using the supplied settings and defaults for omitted options.
     */
    public TrajectoryPlayback() {
        this(System::nanoTime);
    }

    /**
     * Builds the trajectory playback and connects its controls to the supplied model.
     *
     * @param clock initial clock
     */
    public TrajectoryPlayback(LongSupplier clock) {
        this.clock = clock;
    }

    /**
     * Sets the playable simulation-time interval and clamps the playhead to it.
     *
     * @param start start value
     * @param end end value
     */
    public void range(double start, double end) {
        this.start = start;
        this.end = end;
        playing = false;
        seek(start);
    }

    /**
     * Returns the current simulation playhead time using elapsed real time when playing.
     *
     * @return the current simulation playhead time using elapsed real time when playing
     */
    public double time() {
        double result = anchor + (playing ? (clock.getAsLong() - anchorNanos) / 1e9 * speed : 0);
        if (result >= end) {
            result = end;
            anchor = end;
            playing = false;
        }
        return Math.max(start, result);
    }

    /**
     * Moves the playhead to a bounded simulation time while preserving playback state.
     *
     * @param time time
     */
    public void seek(double time) {
        anchor = Math.max(start, Math.min(end, time));
        anchorNanos = clock.getAsLong();
    }

    /**
     * Starts or pauses playback without jumping the current playhead.
     *
     * @param value replacement or input value
     */
    public void setPlaying(boolean value) {
        double now = time();
        playing = value;
        seek(now);
    }

    /**
     * Changes the ratio of simulation time to real playback time.
     *
     * @param value replacement or input value
     * @throws IllegalArgumentException when rejected: Playback speed must be between 0.01 and 20
     */
    public void setSpeed(double value) {
        if (!Double.isFinite(value) || value < 0.01 || value > 20)
            throw new IllegalArgumentException("Playback speed must be between 0.01 and 20");
        double now = time();
        speed = value;
        seek(now);
    }

    /**
     * Reports whether the trajectory animation is currently advancing.
     *
     * @return true when the condition described above holds
     */
    public boolean isPlaying() {
        return playing;
    }
}
