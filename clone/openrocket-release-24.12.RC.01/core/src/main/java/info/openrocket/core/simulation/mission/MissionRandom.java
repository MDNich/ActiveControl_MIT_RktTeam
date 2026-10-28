package info.openrocket.core.simulation.mission;

import java.io.*;
import java.math.BigInteger;
import java.nio.charset.StandardCharsets;
import java.security.*;

/** Event-addressed sampling; adding receivers or changing worker count cannot consume another stream. */
public final class MissionRandom {
    public static final String VERSION = "sha256-counter-v1";
    private MissionRandom() { }

    public static byte[] digest(String seed, long member, String participant, String phenomenon,
                                String model, long event, long draw) {
        if (!seed.matches("0|[1-9][0-9]{0,19}") || new BigInteger(seed).bitLength() > 64
                || member < 0 || event < 0 || draw < 0)
            throw new IllegalArgumentException("Invalid sampler counter or unsigned seed");
        String[] fields = {VERSION, seed, Long.toString(member), participant, phenomenon, model,
                Long.toString(event), Long.toString(draw)};
        try {
            var buffer = new ByteArrayOutputStream();
            var stream = new DataOutputStream(buffer);
            for (String field : fields) {
                if (field == null || field.isEmpty() || field.length() > 1024)
                    throw new IllegalArgumentException("Invalid sampler identity");
                byte[] bytes = field.getBytes(StandardCharsets.UTF_8);
                stream.writeInt(bytes.length);
                stream.write(bytes);
            }
            return MessageDigest.getInstance("SHA-256").digest(buffer.toByteArray());
        } catch (IOException | NoSuchAlgorithmException e) { throw new IllegalStateException(e); }
    }

    public static double uniform(String seed, long member, String participant, String phenomenon,
                                 String model, long event, long draw) {
        byte[] hash = digest(seed, member, participant, phenomenon, model, event, draw);
        long n = 0;
        for (int i = 0; i < 7; i++) n = (n << 8) | (hash[i] & 255L);
        return ((n >>> 4) + 0.5) / 0x1.0p52;
    }

    public static double normal(String seed, long member, String participant, String phenomenon,
                                String model, long event, long pair) {
        long first = Math.multiplyExact(pair, 2);
        double a = uniform(seed, member, participant, phenomenon, model, event, first);
        double b = uniform(seed, member, participant, phenomenon, model, event, Math.addExact(first, 1));
        return StrictMath.sqrt(-2 * StrictMath.log(a)) * StrictMath.cos(2 * Math.PI * b);
    }
}
