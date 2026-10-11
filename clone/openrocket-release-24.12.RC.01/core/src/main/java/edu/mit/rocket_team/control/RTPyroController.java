package edu.mit.rocket_team.control;

import java.util.Arrays;
import edu.mit.rocket_team.util.*;
import edu.mit.rocket_team.util.data.*;
import static edu.mit.rocket_team.util.RTUtilLibrary.elapsed32;

/** Firmware pulse/continuity model. Accepted firing can be connected to OR recovery. */
public class RTPyroController extends RTController {
    public static final int NUM_PYROS = 6, FIRE_DURATION_MS = 250;
    private final RTUtilLibrary.Trace trace;
    private final boolean[] armed = new boolean[6],
            fired = new boolean[6],
            connected = new boolean[6];
    private final long[] firedTimes = new long[6];
    private final RTPyroStatus[] status = new RTPyroStatus[6];

    public record FireCommand(long sequence, int channel, long bootUs, String origin) {}

    private long sequence;
    private java.util.function.Consumer<FireCommand> fireObserver = c -> {};
    private Runnable outputObserver = () -> {};

    /**
     * Installs the callback notified when a channel accepts a firing command, including retriggers
     * during a pulse.
     *
     * @param observer callback notified of accepted commands or output changes
     */
    public void setFireObserver(java.util.function.Consumer<FireCommand> observer) {
        fireObserver = observer == null ? c -> {} : observer;
    }

    /**
     * Installs the callback for logical channel output transitions.
     *
     * @param observer callback notified of accepted commands or output changes
     */
    public void setOutputObserver(Runnable observer) {
        outputObserver = observer == null ? () -> {} : observer;
    }

    /** Creates a RTPyroController using the supplied settings and defaults for omitted options. */
    public RTPyroController() {
        this(new RTUtilLibrary.Trace());
    }

    /**
     * Creates a RTPyroController with the supplied dependencies and per-instance state.
     *
     * @param trace run-local virtual clock and action-log sink
     */
    public RTPyroController(RTUtilLibrary.Trace trace) {
        this.trace = trace;
        Arrays.fill(connected, true);
        Arrays.fill(status, RTPyroStatus.PYRO_FAILURE);
    }

    /**
     * Validates that a channel index belongs to the six-channel pyro bank.
     *
     * @param c recovery channel index, 0 through 5
     * @throws IllegalArgumentException when rejected: Pyro channel must be 0..5:
     */
    private void check(int c) {
        if (c < 0 || c >= 6) throw new IllegalArgumentException("Pyro channel must be 0..5: " + c);
    }

    /**
     * Initializes the modeled recovery interface and records its logical-only hardware behavior.
     */
    @Override
    public void setup() {
        trace.log("pyro.setup", "channels=6 recovery=mapped_in_simulation continuity=connected");
    }

    /**
     * Arms a channel so a later fire command may be accepted.
     *
     * @param c recovery channel index, 0 through 5
     */
    public void armPyro(int c) {
        check(c);
        boolean changed = !armed[c];
        armed[c] = true;
        if (changed) outputObserver.run();
        trace.log("pyro.arm", "channel=" + c);
    }

    /**
     * Disarms a channel against subsequent firing commands.
     *
     * @param c recovery channel index, 0 through 5
     */
    public void disarmPyro(int c) {
        check(c);
        boolean changed = armed[c];
        armed[c] = false;
        if (changed) outputObserver.run();
        trace.log("pyro.disarm", "channel=" + c);
    }

    /**
     * Accepts a firing pulse for an armed channel, disarms it and notifies observers with exact
     * boot microseconds. Continuity affects reported status, not this command gate.
     *
     * @param c recovery channel index, 0 through 5
     */
    public void firePyro(int c) {
        firePyro(c, "controller");
    }

    /**
     * Accepts a firing pulse for an armed channel, disarms it and notifies observers with exact
     * boot microseconds. Continuity affects reported status, not this command gate.
     *
     * @param c recovery channel index, 0 through 5
     * @param origin source identifier recorded with the command
     */
    public void firePyro(int c, String origin) {
        check(c);
        if (!armed[c]) {
            trace.log("pyro.fire_ignored", "channel=" + c + " reason=unarmed");
            return;
        }
        firedTimes[c] = trace.millis();
        fired[c] = true;
        armed[c] = false;
        var command = new FireCommand(++sequence, c, trace.bootUs(), origin);
        trace.log(
                "pyro.fire",
                "channel="
                        + c
                        + " duration_ms="
                        + FIRE_DURATION_MS
                        + " sequence="
                        + sequence
                        + " origin="
                        + origin);
        outputObserver.run();
        fireObserver.accept(command);
    }

    /**
     * Clears an active channel output and notifies the output observer.
     *
     * @param c recovery channel index, 0 through 5
     */
    public void off(int c) {
        check(c);
        boolean changed = fired[c];
        fired[c] = false;
        if (changed) outputObserver.run();
        trace.log("pyro.off", "channel=" + c);
    }

    /**
     * Sets the simulated continuity of a recovery channel.
     *
     * @param c recovery channel index, 0 through 5
     * @param value replacement or input value
     */
    public void setConnected(int c, boolean value) {
        check(c);
        connected[c] = value;
        trace.log("pyro.continuity", "channel=" + c + " connected=" + value);
    }

    /**
     * Returns whether the channel is currently armed.
     *
     * @param c recovery channel index, 0 through 5
     * @return whether the channel is currently armed
     */
    public boolean isArmed(int c) {
        check(c);
        return armed[c];
    }

    /**
     * Returns whether the channel output is currently active.
     *
     * @param c recovery channel index, 0 through 5
     * @return whether the channel output is currently active
     */
    public boolean isFired(int c) {
        check(c);
        return fired[c];
    }

    /**
     * Returns the firmware status enum for one recovery channel.
     *
     * @param c recovery channel index, 0 through 5
     * @return the firmware status enum for one recovery channel
     */
    public RTPyroStatus getPyroStatus(int c) {
        check(c);
        return status[c];
    }

    /**
     * Packs all six two-bit recovery status codes into one integer.
     *
     * @return computed integer value
     */
    public int getPyrosStatus() {
        int bits = 0;
        for (int i = 0; i < 6; i++) bits |= status[i].ID << (2 * i);
        return bits;
    }

    /**
     * Expires pulses after the strict 250 ms firmware threshold and updates status from continuity
     * and flight phase.
     *
     * @param state current flight phase or state
     */
    public void pyroMonitor(RTRocketState state) {
        for (int i = 0; i < 6; i++) {
            RTPyroStatus old = status[i];
            if (fired[i] && elapsed32(trace.millis(), firedTimes[i]) > FIRE_DURATION_MS) {
                off(i);
                status[i] =
                        !connected[i] && status[i] != RTPyroStatus.PYRO_FAILURE
                                ? RTPyroStatus.PYRO_SUCCESS
                                : RTPyroStatus.PYRO_FAILURE;
            }
            if (state == RTRocketState.GROUND_TESTING)
                status[i] =
                        connected[i] ? RTPyroStatus.PYRO_CONNECTED : RTPyroStatus.PYRO_UNCONNECTED;
            else if (!connected[i] && status[i] != RTPyroStatus.PYRO_SUCCESS && !fired[i])
                status[i] = RTPyroStatus.PYRO_FAILURE;
            if (old != status[i])
                trace.log("pyro.status", "channel=" + i + " from=" + old + " to=" + status[i]);
        }
        trace.log("pyro.update", "status_bits=" + getPyrosStatus());
    }

    /**
     * Rejects the context-free controller hook; callers must use pyroMonitor with the current
     * firmware phase.
     *
     * @throws IllegalStateException when rejected: Use pyroMonitor with FC state
     */
    @Override
    public void performLoopAction() {
        throw new IllegalStateException("Use pyroMonitor with FC state");
    }

    /**
     * Rejects sensor-payload injection; simulated continuity is set through setConnected.
     *
     * @param data input data for this operation
     * @throws UnsupportedOperationException when rejected: Use setConnected
     */
    @Override
    public void backdoorFudge(RTFudgedData data) {
        throw new UnsupportedOperationException("Use setConnected");
    }
}
