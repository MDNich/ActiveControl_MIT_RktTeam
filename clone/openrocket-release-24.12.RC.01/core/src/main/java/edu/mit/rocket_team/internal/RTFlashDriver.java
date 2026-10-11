package edu.mit.rocket_team.internal;

import edu.mit.rocket_team.util.RTUtilLibrary.Trace;

/** FC allocateFlash/handleLogging side effects. Only one 512-byte page is retained. */
public class RTFlashDriver {
    private final Trace trace;
    private final byte[] page = new byte[512];
    private int buffered, writeIndex;

    /** Creates a RTFlashDriver using the supplied settings and defaults for omitted options. */
    public RTFlashDriver() {
        this(new Trace());
    }

    /**
     * Creates a RTFlashDriver with the supplied dependencies and per-instance state.
     *
     * @param trace run-local virtual clock and action-log sink
     */
    public RTFlashDriver(Trace trace) {
        this.trace = trace;
    }

    /** Records initialization of the simulated flash interface. */
    public void setup() {
        trace.log("flash.setup", "mode=memory_page");
    }

    /** Starts a new logical logging allocation without modeling flash geometry or wear. */
    public void allocate() {
        for (int address = 0; address < 5000000; address += 256000)
            trace.log("flash.erase", "address=" + address + " simulated=true");
    }

    /**
     * Records a flash append operation for the supplied telemetry bytes.
     *
     * @param packet firmware-format packet bytes
     */
    public void append(byte[] packet) {
        System.arraycopy(packet, 0, page, buffered * 128, 128);
        buffered++;
        trace.log("flash.buffer", "packets=" + buffered);
        if (buffered == 4) {
            trace.log("flash.program", "address=" + writeIndex + " bytes=512");
            writeIndex += 512;
            buffered = 0;
        }
    }
}
