package info.openrocket.swing.startup;

import java.lang.management.ManagementFactory;
import com.sun.management.HotSpotDiagnosticMXBean;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

class StartupMemoryPolicyTest {
    /** Verifies that enables concurrent idle reclamation and preserves explicit tuning. */
    @Test
    void enablesConcurrentIdleReclamationAndPreservesExplicitTuning() {
        assumeTrue(
                ManagementFactory.getGarbageCollectorMXBeans().stream()
                        .anyMatch(gc -> gc.getName().startsWith("G1")));
        var bean = ManagementFactory.getPlatformMXBean(HotSpotDiagnosticMXBean.class);
        String original = bean.getVMOption("G1PeriodicGCInterval").getValue();
        try {
            StartupMemoryPolicy.configure();
            assertEquals("30000", bean.getVMOption("G1PeriodicGCInterval").getValue());
            bean.setVMOption("G1PeriodicGCInterval", "12345");
            StartupMemoryPolicy.configure();
            assertEquals("12345", bean.getVMOption("G1PeriodicGCInterval").getValue());
            bean.setVMOption("G1PeriodicGCInterval", "0");
            StartupMemoryPolicy.configure();
            assertEquals("0", bean.getVMOption("G1PeriodicGCInterval").getValue());
        } finally {
            bean.setVMOption("G1PeriodicGCInterval", original);
        }
    }
}
