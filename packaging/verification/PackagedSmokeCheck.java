import info.openrocket.core.util.BuildProperties;
import com.jogamp.common.os.Platform;
import com.jogamp.opengl.GLProfile;
import info.openrocket.swing.gui.plot.TrajectoryData;

/** Runtime, edition, graphics and latest-feature smoke test against a packaged JAR. */
public final class PackagedSmokeCheck {
    public static void main(String[] args) {
        if (!BuildProperties.isMitEdition() || !args[0].equals(BuildProperties.getMitVersion()))
            throw new AssertionError("Wrong MIT edition");
        System.out.println("OS=" + System.getProperty("os.name") + ", JVM_ARCH=" + System.getProperty("os.arch"));
        System.out.println("JAVA=" + System.getProperty("java.runtime.version"));
        System.out.println("OPENROCKET=" + BuildProperties.getVersion() + ", MIT_EDITION=" + BuildProperties.getMitVersion());
        if (java.util.Arrays.stream(TrajectoryData.Frame.class.getRecordComponents()).noneMatch(c -> c.getName().equals("powered")))
            throw new AssertionError("Latest exhaust animation is missing");
        System.out.println("EXHAUST_ANIMATION_PRESENT=true");
        Platform.initSingleton();
        System.out.println("GLUEGEN_INITIALIZED=true");
        GLProfile.initSingleton();
        System.out.println("OPENGL_DEFAULT=" + GLProfile.getDefault());
        System.out.println("PACKAGED_SMOKE_CHECK=PASS");
    }
}
