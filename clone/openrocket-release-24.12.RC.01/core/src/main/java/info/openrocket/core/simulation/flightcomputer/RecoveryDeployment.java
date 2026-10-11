package info.openrocket.core.simulation.flightcomputer;

import jakarta.json.*;
import java.util.UUID;

/**
 * Immutable provenance retained in saved/imported physical deployment events. Not event
 * authorization.
 */
public record RecoveryDeployment(
        UUID deviceId,
        String deviceName,
        int channel,
        String origin,
        long commandUs,
        long dueUs,
        long sequence) {
    /**
     * Creates a RecoveryDeployment after validating the supplied configuration.
     *
     * @param deviceId device id
     * @param deviceName device name
     * @param channel logical recovery channel, numbered 0 through 5
     * @param origin source identifier recorded with the command
     * @param commandUs command in microseconds
     * @param dueUs due in microseconds
     * @param sequence sequence
     * @throws IllegalArgumentException when rejected: Invalid recovery event metadata
     */
    public RecoveryDeployment {
        java.util.Objects.requireNonNull(deviceId);
        java.util.Objects.requireNonNull(deviceName);
        java.util.Objects.requireNonNull(origin);
        if (channel < 0 || channel > 5 || commandUs < 0 || dueUs < commandUs || sequence < 1)
            throw new IllegalArgumentException("Invalid recovery event metadata");
    }

    /**
     * Serializes a physical recovery command's channel, target, origin and timing.
     *
     * @return the resulting JsonObject
     */
    public JsonObject json() {
        return Json.createObjectBuilder()
                .add("deviceId", deviceId.toString())
                .add("deviceName", deviceName)
                .add("channel", channel)
                .add("origin", origin)
                .add("commandUs", commandUs)
                .add("dueUs", dueUs)
                .add("sequence", sequence)
                .build();
    }

    /**
     * Reconstructs a recovery deployment record from structured log JSON.
     *
     * @param json json
     * @return the resulting RecoveryDeployment
     */
    public static RecoveryDeployment read(JsonObject json) {
        return new RecoveryDeployment(
                UUID.fromString(json.getString("deviceId")),
                json.getString("deviceName"),
                json.getInt("channel"),
                json.getString("origin"),
                json.getJsonNumber("commandUs").longValueExact(),
                json.getJsonNumber("dueUs").longValueExact(),
                json.getJsonNumber("sequence").longValueExact());
    }
}
