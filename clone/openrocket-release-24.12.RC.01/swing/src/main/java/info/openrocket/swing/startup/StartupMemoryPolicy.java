package info.openrocket.swing.startup;

import java.lang.management.ManagementFactory;
import com.sun.management.HotSpotDiagnosticMXBean;
import com.sun.management.VMOption;

/** Return unused G1 heap to the OS after allocation-heavy simulations and file loading. */
public final class StartupMemoryPolicy {
    private static final String INTERVAL = "G1PeriodicGCInterval";
    private StartupMemoryPolicy() { }

    public static void configure() {
        if (ManagementFactory.getGarbageCollectorMXBeans().stream().noneMatch(gc -> gc.getName().startsWith("G1"))) {
            return;
        }
        try {
            var bean = ManagementFactory.getPlatformMXBean(HotSpotDiagnosticMXBean.class);
            var interval = bean.getVMOption(INTERVAL);
            // Respect launcher/user tuning, including an explicit zero that disables periodic GC.
            if (interval.getOrigin() != VMOption.Origin.DEFAULT || !interval.getValue().equals("0")) {
                return;
            }
            if (!Boolean.parseBoolean(bean.getVMOption("G1PeriodicGCInvokesConcurrent").getValue())) {
                return; // Do not introduce periodic stop-the-world full collections.
            }
            bean.setVMOption(INTERVAL, "30000");
            System.out.println("MEMORY G1 periodic concurrent collection enabled; interval_ms=30000; heap limits unchanged");
        } catch (IllegalArgumentException | UnsupportedOperationException | SecurityException e) {
            // Other supported runtimes may not expose the manageable HotSpot option.
            System.out.println("MEMORY automatic idle heap reclamation unavailable: " + e.getMessage());
        }
    }
}
