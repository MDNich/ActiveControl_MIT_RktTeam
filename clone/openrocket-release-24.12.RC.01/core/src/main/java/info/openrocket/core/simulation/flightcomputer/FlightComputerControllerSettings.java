package info.openrocket.core.simulation.flightcomputer;

import info.openrocket.core.util.Config;
import info.openrocket.core.rocketcomponent.*;
import jakarta.json.*;
import java.io.StringReader;
import java.util.*;

/** Per-simulation UUID bindings, independent of the portable .fc definition. */
public record FlightComputerControllerSettings(String designId, List<Binding> bindings) {
    public static final String KEY = "controllerActuatorMappings";
    public static final FlightComputerControllerSettings EMPTY =
            new FlightComputerControllerSettings("", List.of());

    public record Binding(
            String controllerId,
            UUID deviceId,
            double direction,
            double trim,
            double minimum,
            double maximum,
            boolean cantOffset) {
        /**
         * Creates a Binding value from the supplied controller id, device id, direction, trim,
         * minimum, maximum, cant offset.
         *
         * @param controllerId controller id
         * @param deviceId device id
         * @param direction direction
         * @param trim trim
         * @param minimum minimum
         * @param maximum maximum
         * @param cantOffset cant offset
         * @throws IllegalArgumentException when rejected: Invalid actuator calibration
         */
        public Binding {
            UUID.fromString(controllerId);
            Objects.requireNonNull(deviceId);
            if (Math.abs(direction) != 1
                    || !Double.isFinite(trim)
                    || !Double.isFinite(minimum)
                    || !Double.isFinite(maximum)
                    || minimum >= maximum)
                throw new IllegalArgumentException("Invalid actuator calibration");
        }
    }

    /**
     * Creates a FlightComputerControllerSettings after validating the supplied configuration.
     *
     * @param designId design id
     * @param bindings bindings
     * @throws IllegalArgumentException when rejected: Duplicate controller target
     */
    public FlightComputerControllerSettings {
        Objects.requireNonNull(designId);
        bindings = List.copyOf(bindings);
        var seen = new HashSet<String>();
        for (var b : bindings)
            if (!seen.add(b.controllerId() + ":" + b.deviceId()))
                throw new IllegalArgumentException("Duplicate controller target");
    }

    /**
     * Returns the saved physical assignments for a controller ID.
     *
     * @param id stable identifier
     * @return the saved physical assignments for a controller ID
     */
    public List<Binding> forController(String id) {
        return bindings.stream().filter(b -> b.controllerId().equals(id)).toList();
    }

    /**
     * Lists active rocket components supporting the selected controller output mode.
     *
     * @param mode mode
     * @param c selected channel or controller
     * @return true when the condition described above holds
     */
    public static boolean eligible(FlightComputerController.Mode mode, RocketComponent c) {
        return switch (mode) {
            case TABS -> c instanceof TabControlledTrapezoidFinSet;
            case CANT -> c instanceof FinSet;
            case AIRBRAKES -> c instanceof AirbrakeSet;
        };
    }

    /**
     * Checks target identities, bounds and ownership conflicts before a simulation runs.
     *
     * @param design external flight-computer design snapshot
     * @param active active
     * @throws IllegalArgumentException when rejected: Controller assignments belong to a different
     *     flight computer; assign this design\'s outputs again
     */
    public void validate(FlightComputerDesign design, Collection<RocketComponent> active) {
        if (bindings.isEmpty()) return;
        if (!designId.equals(design.id()))
            throw new IllegalArgumentException(
                    "Controller assignments belong to a different flight computer; assign this design's outputs again");
        var blocks = new HashMap<String, FlightComputerController>();
        design.controllers().forEach(b -> blocks.put(b.id(), b));
        var targets = new HashMap<UUID, RocketComponent>();
        active.forEach(c -> targets.put(c.getID(), c));
        for (var binding : bindings) {
            var b = blocks.get(binding.controllerId());
            if (b == null)
                throw new IllegalArgumentException(
                        "Assignment references a deleted controller: " + binding.controllerId());
            var target = targets.get(binding.deviceId());
            if (target == null || !eligible(b.mode(), target))
                throw new IllegalArgumentException(
                        "Missing, inactive or incompatible target for "
                                + b.name()
                                + ": "
                                + binding.deviceId());
            double lo = b.mode() == FlightComputerController.Mode.AIRBRAKES ? 0 : -90,
                    hi = b.mode() == FlightComputerController.Mode.AIRBRAKES ? 1 : 90;
            if (binding.minimum() < lo || binding.maximum() > hi)
                throw new IllegalArgumentException("Target limits outside " + lo + ".." + hi);
        }
        for (var block : blocks.values()) {
            double low = block.n("minimum"), high = block.n("maximum");
            for (var b : forController(block.id())) {
                double offset = b.trim();
                var target = targets.get(b.deviceId());
                if (block.mode() == FlightComputerController.Mode.CANT && b.cantOffset())
                    offset += Math.toDegrees(((FinSet) target).getCantAngle());
                double a = (b.minimum() - offset) / b.direction(),
                        z = (b.maximum() - offset) / b.direction();
                low = Math.max(low, Math.min(a, z));
                high = Math.min(high, Math.max(a, z));
            }
            if (low > high)
                throw new IllegalArgumentException(
                        "Target calibrations have no common output range for " + block.name());
        }
        for (int i = 0; i < bindings.size(); i++)
            for (int j = i + 1; j < bindings.size(); j++) {
                var a = bindings.get(i);
                var b = bindings.get(j);
                var ca = blocks.get(a.controllerId());
                var cb = blocks.get(b.controllerId());
                if (a.deviceId().equals(b.deviceId())
                        && ca.mode() == cb.mode()
                        && ca.enabled()
                        && cb.enabled()
                        && !Collections.disjoint(ca.states(), cb.states()))
                    throw new IllegalArgumentException(
                            "Output ownership conflict: "
                                    + ca.name()
                                    + " and "
                                    + cb.name()
                                    + " both control "
                                    + targets.get(a.deviceId()).getName());
            }
    }

    /**
     * Reads controller-to-rocket assignments from simulation configuration.
     *
     * @param c selected channel or controller
     * @return the resulting FlightComputerControllerSettings
     * @throws IllegalArgumentException when rejected: Invalid controller assignments
     */
    public static FlightComputerControllerSettings read(Config c) {
        if (!c.containsKey(KEY)) return EMPTY;
        if (!(c.get(KEY, null) instanceof String text))
            throw new IllegalArgumentException("Invalid controller assignments");
        try (var reader = Json.createReader(new StringReader(text))) {
            var root = reader.readObject();
            if (root.getInt("version") != 1)
                throw new IllegalArgumentException("Unsupported controller assignment version");
            var result = new ArrayList<Binding>();
            for (var b : root.getJsonArray("bindings").getValuesAs(JsonObject.class))
                result.add(
                        new Binding(
                                b.getString("controllerId"),
                                UUID.fromString(b.getString("deviceId")),
                                b.getJsonNumber("direction").doubleValue(),
                                b.getJsonNumber("trim").doubleValue(),
                                b.getJsonNumber("minimum").doubleValue(),
                                b.getJsonNumber("maximum").doubleValue(),
                                b.getBoolean("cantOffset")));
            return new FlightComputerControllerSettings(root.getString("designId"), result);
        }
    }

    /**
     * Persists controller-to-rocket assignments using stable component UUIDs.
     *
     * @param c selected channel or controller
     */
    public void write(Config c) {
        var a = Json.createArrayBuilder();
        for (var b : bindings)
            a.add(
                    Json.createObjectBuilder()
                            .add("controllerId", b.controllerId())
                            .add("deviceId", b.deviceId().toString())
                            .add("direction", b.direction())
                            .add("trim", b.trim())
                            .add("minimum", b.minimum())
                            .add("maximum", b.maximum())
                            .add("cantOffset", b.cantOffset()));
        c.put(
                KEY,
                Json.createObjectBuilder()
                        .add("version", 1)
                        .add("designId", designId)
                        .add("bindings", a)
                        .build()
                        .toString());
    }
}
