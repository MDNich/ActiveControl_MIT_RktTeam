package edu.mit.rocket_team.zephyrus.internal;
import edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.Trace;
/** FC allocateFlash/handleLogging side effects. Only one 512-byte page is retained. */
public class RTFlashDriver {
    private final Trace trace;
    private final byte[] page=new byte[512];
    private int buffered,writeIndex;
    public RTFlashDriver() { this(new Trace()); }
    public RTFlashDriver(Trace trace) { this.trace=trace; }
    public void setup() { trace.log("flash.setup", "mode=memory_page"); }
    public void allocate() { for(int address=0;address<5000000;address+=256000) trace.log("flash.erase", "address="+address+" simulated=true"); }
    public void append(byte[] packet) {
        System.arraycopy(packet,0,page,buffered*128,128); buffered++;
        trace.log("flash.buffer", "packets="+buffered);
        if(buffered==4) { trace.log("flash.program", "address="+writeIndex+" bytes=512"); writeIndex+=512; buffered=0; }
    }
}
