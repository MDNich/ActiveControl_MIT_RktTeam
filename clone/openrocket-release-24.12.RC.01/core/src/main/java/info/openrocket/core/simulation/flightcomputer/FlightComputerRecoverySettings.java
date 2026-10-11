package info.openrocket.core.simulation.flightcomputer;

import info.openrocket.core.util.Config;
import jakarta.json.*;
import java.io.StringReader;
import java.util.*;

/** Rocket-specific assignments. An empty map still means exclusive FC recovery authority. */
public record FlightComputerRecoverySettings(List<Binding> bindings) {
    public static final FlightComputerRecoverySettings EMPTY =
            new FlightComputerRecoverySettings(List.of());
    public static final String KEY = "recoveryMappings";
    public static final long MAX_DELAY_US = 86_400_000_000L;

    public record Binding(UUID deviceId, Set<Integer> channels, long delayUs) {
        /**
         * Creates a Binding value from the supplied device id, channels, delay us.
         *
         * @param deviceId device id
         * @param channels channels
         * @param delayUs delay in microseconds
         * @throws IllegalArgumentException when rejected: Recovery channels must be 0\u20135
         */
        public Binding {
            Objects.requireNonNull(deviceId);
            channels = Collections.unmodifiableSet(new TreeSet<>(channels));
            if (channels.stream().anyMatch(c -> c < 0 || c > 5))
                throw new IllegalArgumentException("Recovery channels must be 0–5");
            if (delayUs < 0 || delayUs > MAX_DELAY_US)
                throw new IllegalArgumentException(
                        "Recovery delay must be between 0 and 86400000 ms");
        }
    }

    /**
     * Creates a FlightComputerRecoverySettings after validating the supplied configuration.
     *
     * @param bindings initial bindings
     * @throws IllegalArgumentException when rejected: Duplicate recovery device assignment:
     */
    public FlightComputerRecoverySettings {
        bindings =
                bindings.stream()
                        .sorted(Comparator.comparing(b -> b.deviceId().toString()))
                        .toList();
        var ids = new HashSet<UUID>();
        for (var b : bindings)
            if (!ids.add(b.deviceId()))
                throw new IllegalArgumentException(
                        "Duplicate recovery device assignment: " + b.deviceId());
    }

    /**
     * Converts a user-entered deployment delay to validated integer microseconds.
     *
     * @param ms in milliseconds
     * @return computed integer value
     * @throws IllegalArgumentException when rejected: Recovery delay must be between 0 and 86400000
     *     ms
     */
    public static long delayFromMilliseconds(double ms) {
        if (!Double.isFinite(ms) || ms < 0 || ms > MAX_DELAY_US / 1000.0)
            throw new IllegalArgumentException("Recovery delay must be between 0 and 86400000 ms");
        return Math.round(ms * 1000); // nearest microsecond, independent of display precision
    }

    /**
     * Reads recovery assignments from simulation configuration, retaining stable device UUIDs.
     *
     * @param config config
     * @return the resulting FlightComputerRecoverySettings
     * @throws IllegalArgumentException when rejected: Invalid recovery mappings
     */
    public static FlightComputerRecoverySettings read(Config config) {
        if (!config.containsKey(KEY)) return EMPTY;
        if (!(config.get(KEY, null) instanceof String json))
            throw new IllegalArgumentException("Invalid recovery mappings");
        try (var reader = Json.createReader(new StringReader(json))) {
            var root = reader.readObject();
            if (root.getInt("version") != 1)
                throw new IllegalArgumentException("Unsupported recovery settings version");
            var bindings = new ArrayList<Binding>();
            for (var item : root.getJsonArray("devices").getValuesAs(JsonObject.class)) {
                var channels = new TreeSet<Integer>();
                for (var c : item.getJsonArray("channels").getValuesAs(JsonNumber.class))
                    channels.add(c.intValueExact());
                bindings.add(
                        new Binding(
                                UUID.fromString(item.getString("id")),
                                channels,
                                item.getJsonNumber("delayUs").longValueExact()));
            }
            return new FlightComputerRecoverySettings(bindings);
        } catch (RuntimeException e) {
            throw new IllegalArgumentException("Invalid recovery mappings: " + e.getMessage(), e);
        }
    }

    /**
     * Writes recovery assignments and delays into simulation configuration.
     *
     * @param config config
     */
    public void write(Config config) {
        var devices = Json.createArrayBuilder();
        for (var b : bindings) {
            var channels = Json.createArrayBuilder();
            b.channels().forEach(channels::add);
            devices.add(
                    Json.createObjectBuilder()
                            .add("id", b.deviceId().toString())
                            .add("channels", channels)
                            .add("delayUs", b.delayUs()));
        }
        config.put(
                KEY,
                Json.createObjectBuilder()
                        .add("version", 1)
                        .add("devices", devices)
                        .build()
                        .toString());
    }
}
