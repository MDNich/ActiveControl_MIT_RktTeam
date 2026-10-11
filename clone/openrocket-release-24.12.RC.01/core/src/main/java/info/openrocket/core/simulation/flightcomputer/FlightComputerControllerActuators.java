package info.openrocket.core.simulation.flightcomputer;

import edu.mit.rocket_team.FC.RTFC;
import edu.mit.rocket_team.util.RTUtilLibrary.Trace;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.*;
import jakarta.json.*;
import java.util.*;

/**
 * Stable physical assignments for one simulation copy; controller values arrive only at the PWM
 * latch.
 */
public final class FlightComputerControllerActuators {
    private record Target(
            FlightComputerController controller,
            FlightComputerControllerSettings.Binding binding,
            RocketComponent device,
            double baseCant) {}

    private final Map<String, List<Target>> groups = new LinkedHashMap<>();
    private final Map<String, Double> commands = new HashMap<>();
    private final Map<String, Target> lastOwners = new HashMap<>();
    private final FlightComputerControllerRuntime runtime;
    private final Trace trace;

    /**
     * Creates a FlightComputerControllerActuators with the supplied dependencies and per-instance
     * state.
     *
     * @param status stat in microseconds
     * @param design external flight-computer design snapshot
     * @param settings validated configuration for this operation
     * @param runtime initial runtime
     * @param trace run-local virtual clock and action-log sink
     */
    public FlightComputerControllerActuators(
            SimulationStatus status,
            FlightComputerDesign design,
            FlightComputerControllerSettings settings,
            FlightComputerControllerRuntime runtime,
            Trace trace) {
        this.runtime = runtime;
        this.trace = trace;
        settings.validate(design, status.getConfiguration().getActiveComponents());
        var definitions = new HashMap<String, FlightComputerController>();
        runtime.definitions().forEach(c -> definitions.put(c.id(), c));
        for (var binding : settings.bindings()) {
            var c = definitions.get(binding.controllerId());
            if (c == null) continue;
            var device =
                    status.getConfiguration().getActiveComponents().stream()
                            .filter(d -> d.getID().equals(binding.deviceId()))
                            .findFirst()
                            .orElseThrow();
            var target =
                    new Target(
                            c,
                            binding,
                            device,
                            device instanceof FinSet f ? Math.toDegrees(f.getCantAngle()) : 0);
            groups.computeIfAbsent(key(c.mode(), device), k -> new ArrayList<>()).add(target);
            trace.log(
                    "controller.bind",
                    "controller="
                            + c.id()
                            + " device="
                            + device.getID()
                            + " name="
                            + device.getName()
                            + " mode="
                            + c.mode()
                            + " direction="
                            + binding.direction()
                            + " trim="
                            + binding.trim());
        }
        for (var c : runtime.definitions()) {
            var targets =
                    groups.values().stream()
                            .flatMap(List::stream)
                            .filter(t -> t.controller.id().equals(c.id()))
                            .toList();
            runtime.constrain(
                    c.id(),
                    value ->
                            targets.stream()
                                    .mapToDouble(
                                            t -> {
                                                double physical = desired(t, value);
                                                return Math.abs(
                                                        physical
                                                                - clamp(
                                                                        physical,
                                                                        t.binding.minimum(),
                                                                        t.binding.maximum()));
                                            })
                                    .max()
                                    .orElse(0));
            if (targets.isEmpty())
                trace.log("controller.recorded_only", "id=" + c.id() + " reason=no_target");
        }
    }

    /**
     * Builds the stable actuator ownership key from output mode and component identity.
     *
     * @param mode mode
     * @param c selected channel or controller
     * @return formatted or resolved text
     */
    private static String key(FlightComputerController.Mode mode, RocketComponent c) {
        return mode + ":" + c.getID();
    }

    /**
     * Tests whether an enabled mapped controller owns the physical actuator mode.
     *
     * @param mode mode
     * @param device device
     * @return true when the condition described above holds
     */
    public boolean reserved(FlightComputerController.Mode mode, RocketComponent device) {
        return groups.containsKey(key(mode, device));
    }

    /**
     * Explicit Java writes cannot silently fight a mapped block, including its inactive neutral
     * command.
     */
    public void checkJavaWrite(String port, String name) {
        var mode =
                port.equals("roll")
                        ? FlightComputerController.Mode.TABS
                        : port.equals("airbrakes") ? FlightComputerController.Mode.AIRBRAKES : null;
        if (mode == null) return;
        for (var list : groups.values())
            for (var t : list)
                if (t.controller.mode() == mode
                        && (name == null || name.equals(t.device.getName()))) {
                    if (name != null)
                        throw new IllegalArgumentException(
                                "Java output '"
                                        + name
                                        + "' is reserved by controller '"
                                        + t.controller.name()
                                        + "'. Disable or unmap that block before writing this target.");
                    trace.log(
                            "controller.broadcast_skip",
                            "device=" + t.device.getID() + " controller=" + t.controller.id());
                }
    }

    /**
     * Converts a logical controller value through the target's sign, trim and cant mode.
     *
     * @param t t
     * @param value replacement or input value
     * @return computed numeric value
     */
    private static double desired(Target t, double value) {
        return t.binding.direction() * value
                + t.binding.trim()
                + (t.controller.mode() == FlightComputerController.Mode.CANT
                                && t.binding.cantOffset()
                        ? t.baseCant
                        : 0);
    }

    /**
     * Constrains a logical output using the configured physical target limits.
     *
     * @param v v
     * @param lo lo
     * @param hi hi
     * @return computed numeric value
     */
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /**
     * Applies latched controller outputs to the run's fin tabs, fin cant or airbrake fractions.
     *
     * @param output output snapshot or destination
     * @param holdAirbrakes hold airbrakes
     */
    public void apply(RTFC.Output output, boolean holdAirbrakes) {
        for (var entry : groups.entrySet()) {
            var t =
                    entry.getValue().stream()
                            .filter(x -> output.activeControllers().contains(x.controller.id()))
                            .findFirst()
                            .orElse(
                                    lastOwners.getOrDefault(
                                            entry.getKey(), entry.getValue().get(0)));
            lastOwners.put(entry.getKey(), t);
            double requested =
                    desired(
                            t,
                            output.controllerOutputs()
                                    .getOrDefault(t.controller.id(), t.controller.n("trim")));
            double applied = clamp(requested, t.binding.minimum(), t.binding.maximum());
            if (t.controller.mode() == FlightComputerController.Mode.AIRBRAKES && holdAirbrakes)
                applied = 0;
            switch (t.controller.mode()) {
                case TABS ->
                        ((TabControlledTrapezoidFinSet) t.device)
                                .setTabAngle(Math.toRadians(applied));
                case CANT -> ((FinSet) t.device).setCantAngle(Math.toRadians(applied));
                case AIRBRAKES -> ((AirbrakeSet) t.device).setFracExposed(applied);
            }
            commands.put(entry.getKey(), applied);
            trace.log(
                    "controller.actuate",
                    "id="
                            + t.controller.id()
                            + " device="
                            + t.device.getID()
                            + " mode="
                            + t.controller.mode()
                            + " requested="
                            + requested
                            + " applied="
                            + applied
                            + " latch_us="
                            + trace.bootUs());
        }
    }

    /**
     * Maps a controller output mode to its FC output-port family.
     *
     * @param mode mode
     * @return formatted or resolved text
     */
    private static String port(FlightComputerController.Mode mode) {
        return switch (mode) {
            case TABS -> "roll";
            case CANT -> "cant";
            case AIRBRAKES -> "airbrakes";
        };
    }

    /**
     * Builds controller actuator metadata for plot/log reconstruction.
     *
     * @return the resulting JsonArray
     */
    public JsonArray metadata() {
        var a = Json.createArrayBuilder();
        for (var list : groups.values()) {
            var t = list.get(0);
            a.add(
                    Json.createObjectBuilder()
                            .add("port", port(t.controller.mode()))
                            .add("id", t.device.getID().toString())
                            .add("name", t.device.getName()));
        }
        return a.build();
    }

    /**
     * Records each binding's commanded and physical output values.
     *
     * @param branch flight-data branch receiving or providing samples
     */
    public void record(FlightDataBranch branch) {
        for (var entry : groups.entrySet()) {
            var t = entry.getValue().get(0);
            var channels =
                    FlightComputerData.actuatorChannels(
                            port(t.controller.mode()),
                            t.device.getID().toString(),
                            t.device.getName());
            double physical =
                    switch (t.controller.mode()) {
                        case TABS -> ((TabControlledTrapezoidFinSet) t.device).getTabAngle();
                        case CANT -> ((FinSet) t.device).getCantAngle();
                        case AIRBRAKES -> ((AirbrakeSet) t.device).getFracExposed();
                    };
            double command = commands.getOrDefault(entry.getKey(), Double.NaN);
            if (t.controller.mode() != FlightComputerController.Mode.AIRBRAKES)
                command = Math.toRadians(command);
            branch.setValue(channels.command(), command);
            branch.setValue(channels.physical(), physical);
        }
    }
}
