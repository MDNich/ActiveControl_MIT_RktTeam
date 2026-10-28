import info.openrocket.swing.startup.jij.JarInJarStarter;
import java.util.List;

/** Exercise the distribution's isolated class loader, not just a flat Java classpath. */
public final class MissionPackageLauncher {
    public static void main(String[] args) {
        JarInJarStarter.runMain("MissionPackageProbe", args, () -> List.of(
                JarInJarStarter.class.getProtectionDomain().getCodeSource().getLocation(),
                MissionPackageLauncher.class.getProtectionDomain().getCodeSource().getLocation()));
    }
}
