package edu.mit.rocket_team.zephyrus.telemetry;

/** Receiver effects only; never changes the firmware clock or transmitted bytes. */
public record TelemetryLinkSettings(double packetLossFraction, int delayMs, int randomSeed) {
    public static final TelemetryLinkSettings DEFAULT = new TelemetryLinkSettings(0, 0, 1);

    public TelemetryLinkSettings {
        if (!Double.isFinite(packetLossFraction) || packetLossFraction < 0 || packetLossFraction > 1)
            throw new IllegalArgumentException("Packet loss must be between 0 and 100 percent");
        if (delayMs < 0 || delayMs > 10_000)
            throw new IllegalArgumentException("Downlink delay must be between 0 and 10000 ms");
        if (randomSeed < 0) throw new IllegalArgumentException("Random seed must be nonnegative");
    }
}
