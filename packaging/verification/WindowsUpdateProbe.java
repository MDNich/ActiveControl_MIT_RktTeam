import com.sun.net.httpserver.HttpServer;
import info.openrocket.core.communication.MitReleaseInfo;
import info.openrocket.core.communication.MitUpdateInfo;
import info.openrocket.core.util.JarUtil;
import info.openrocket.swing.gui.util.MitUpdateInstaller;
import java.net.InetSocketAddress;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.util.HexFormat;

/** Runs only in a disposable copied app image; serves the candidate over loopback. */
public class WindowsUpdateProbe {
    public static void main(String[] args) throws Exception {
        Path source = Path.of(args[0]);
        Path evidence = Path.of(args[1]);
        boolean reject = args[2].equals("reject");
        System.setProperty("user.home", evidence.toString());
        Path target = JarUtil.getCurrentJarFile().toPath();
        String before = hash(target);
        String expected = reject ? "0".repeat(64) : hash(source);
        HttpServer server = HttpServer.create(new InetSocketAddress("127.0.0.1", 0), 0);
        server.createContext("/update.jar", exchange -> {
            exchange.sendResponseHeaders(200, Files.size(source));
            try (var out = exchange.getResponseBody()) { Files.copy(source, out); }
        });
        server.start();
        try {
            MitReleaseInfo.Asset asset = new MitReleaseInfo.Asset("OpenRocket-MIT-v6.3.jar",
                "http://127.0.0.1:" + server.getAddress().getPort() + "/update.jar", "", Files.size(source));
            MitUpdateInfo info = new MitUpdateInfo(null, asset, null, true, expected);
            try {
                MitUpdateInstaller.downloadVerifyAndLaunchInstaller(info);
                if (reject) throw new AssertionError("Corrupt update accepted");
                // The old app must be usable until it exits; the helper may only stage now.
                Thread.sleep(1500);
                if (!hash(target).equals(before)) throw new AssertionError("Target replaced while parent still running");
                Files.writeString(evidence.resolve("ready.txt"), "READY; original JAR unchanged while running");
            } catch (MitUpdateInstaller.UpdateInstallException failure) {
                if (!reject || !failure.getMessage().contains("checksum mismatch")) throw failure;
                if (!hash(target).equals(before)) throw new AssertionError("Rejected update altered target");
                Files.writeString(evidence.resolve("rejected.txt"), "PASS: wrong checksum rejected; target unchanged");
            }
        } finally {
            server.stop(0);
        }
        System.exit(0);
    }
    private static String hash(Path file) throws Exception {
        MessageDigest hash = MessageDigest.getInstance("SHA-256");
        try (var in = Files.newInputStream(file)) {
            byte[] buffer = new byte[8192];
            int count;
            while ((count = in.read(buffer)) >= 0) hash.update(buffer, 0, count);
        }
        return HexFormat.of().formatHex(hash.digest());
    }
}
