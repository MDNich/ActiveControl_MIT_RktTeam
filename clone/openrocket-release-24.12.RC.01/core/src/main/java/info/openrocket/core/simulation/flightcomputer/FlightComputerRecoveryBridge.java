package info.openrocket.core.simulation.flightcomputer;

import edu.mit.rocket_team.control.RTPyroController;
import edu.mit.rocket_team.util.RTUtilLibrary.Trace;
import info.openrocket.core.rocketcomponent.RecoveryDevice;
import info.openrocket.core.simulation.*;
import jakarta.json.*;
import java.util.*;

/** One flight's electrical commands -> ordinary OR recovery events; no separate dynamics model. */
public final class FlightComputerRecoveryBridge implements AutoCloseable {
    private record Target(
            RecoveryDevice device,
            FlightComputerRecoverySettings.Binding binding,
            FlightDataType type) {}

    private final List<Target> targets = new ArrayList<>();
    private final Queue<RTPyroController.FireCommand> commands = new ArrayDeque<>();
    private final Set<UUID> attempted = new HashSet<>();
    private final Map<FlightEvent, RecoveryDeployment> pending = new IdentityHashMap<>();
    private final RTPyroController pyro;
    private final Trace trace;
    private final long warmupUs;

    /**
     * Creates a FlightComputerRecoveryBridge after validating the supplied configuration.
     *
     * @param status stat in microseconds
     * @param settings validated configuration for this operation
     * @param pyro initial pyro
     * @param trace run-local virtual clock and action-log sink
     * @param warmupUs warmup in microseconds
     * @throws IllegalArgumentException when rejected: Recovery device
     */
    public FlightComputerRecoveryBridge(
            SimulationStatus status,
            FlightComputerRecoverySettings settings,
            RTPyroController pyro,
            Trace trace,
            long warmupUs) {
        this.pyro = pyro;
        this.trace = trace;
        this.warmupUs = warmupUs;
        var rocket = status.getConfiguration().getRocket();
        for (var b : settings.bindings()) {
            var component = rocket.findComponent(b.deviceId());
            if (!(component instanceof RecoveryDevice device))
                throw new IllegalArgumentException(
                        "Recovery device "
                                + b.deviceId()
                                + " is missing. Update its assignment in the Flight computer tab.");
            targets.add(
                    new Target(
                            device,
                            b,
                            FlightComputerData.recoveryDevice(
                                    b.deviceId().toString(), device.getName())));
        }
        trace.log(
                "recovery.authority",
                "controller=FC OR_deployment_rules=ignored channels=0..5_shared delay_rounding=nearest_microsecond");
        for (var c : rocket)
            if (c instanceof RecoveryDevice device) {
                var binding =
                        settings.bindings().stream()
                                .filter(b -> b.deviceId().equals(c.getID()))
                                .findFirst();
                trace.log(
                        "recovery.mapping",
                        Json.createObjectBuilder()
                                .add("deviceId", c.getID().toString())
                                .add("deviceName", c.getName())
                                .add(
                                        "channels",
                                        binding.map(b -> b.channels().toString()).orElse("[]"))
                                .add(
                                        "delayUs",
                                        binding.map(FlightComputerRecoverySettings.Binding::delayUs)
                                                .orElse(0L))
                                .add("active", status.getConfiguration().isComponentActive(device))
                                .add(
                                        "ignoredRule",
                                        device.getDeploymentConfigurations()
                                                .get(status.getConfiguration().getId())
                                                .getDeployEvent()
                                                .name())
                                .build()
                                .toString());
            }
        pyro.setFireObserver(commands::add);
        pyro.setOutputObserver(this::logOutputs);
        logOutputs();
    }

    /** Records logical recovery output state changes separately from physical deployment events. */
    private void logOutputs() {
        long us = trace.bootUs() - warmupUs;
        if (us < 0) return;
        var values = Json.createObjectBuilder();
        for (int c = 0; c < 6; c++) {
            values.add(FlightComputerData.RECOVERY_ARMED[c].getSymbol(), pyro.isArmed(c) ? 1 : 0);
            values.add(FlightComputerData.RECOVERY_PULSE[c].getSymbol(), pyro.isFired(c) ? 1 : 0);
        }
        trace.log(
                "fc.recovery_state",
                Json.createObjectBuilder()
                        .add("time", us / 1e6)
                        .add("values", values)
                        .build()
                        .toString());
    }

    /**
     * Transfers due accepted FC deployment commands to the OpenRocket event queue.
     *
     * @param status stat in microseconds
     * @throws info.openrocket.core.simulation.exception.SimulationException if the requested
     *     operation cannot be completed
     */
    public void drain(SimulationStatus status)
            throws info.openrocket.core.simulation.exception.SimulationException {
        for (var command = commands.poll(); command != null; command = commands.poll()) {
            long time = command.bootUs() - warmupUs;
            if (time < 0 || status.isLanded()) {
                trace.log(
                        "recovery.discard",
                        "channel="
                                + command.channel()
                                + " simulation_us="
                                + time
                                + " reason="
                                + (time < 0 ? "pad_warmup" : "after_landing"));
                continue;
            }
            boolean mapped = false;
            for (var target : targets)
                if (target.binding.channels().contains(command.channel())) {
                    mapped = true;
                    if (!status.getConfiguration().isComponentActive(target.device)) {
                        trace.log(
                                "recovery.discard",
                                "channel="
                                        + command.channel()
                                        + " device="
                                        + target.device.getID()
                                        + " reason=inactive_device");
                        continue;
                    }
                    if (!attempted.add(target.device.getID())) {
                        trace.log(
                                "recovery.discard",
                                "channel="
                                        + command.channel()
                                        + " device="
                                        + target.device.getID()
                                        + " reason=already_requested");
                        continue;
                    }
                    var provenance =
                            new RecoveryDeployment(
                                    target.device.getID(),
                                    target.device.getName(),
                                    command.channel(),
                                    command.origin(),
                                    time,
                                    Math.addExact(time, target.binding.delayUs()),
                                    command.sequence());
                    var event =
                            new FlightEvent(
                                    FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT,
                                    provenance.dueUs() / 1e6,
                                    target.device,
                                    provenance);
                    pending.put(event, provenance);
                    status.addEvent(event);
                    if (!status.getEventQueue().stream().anyMatch(queued -> queued == event)) {
                        rejected(event, "add_listener_veto");
                        continue;
                    }
                    trace.log("recovery.schedule", provenance.json().toString());
                }
            if (!mapped)
                trace.log(
                        "recovery.discard",
                        "channel="
                                + command.channel()
                                + " simulation_us="
                                + time
                                + " reason=unmapped_channel");
        }
    }

    /**
     * Checks ownership and physical eligibility of an FC recovery event before deployment.
     *
     * @param status stat in microseconds
     * @param event event to process
     * @return true when the condition described above holds
     */
    public boolean allow(SimulationStatus status, FlightEvent event) {
        if (event.getType() != FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT) return true;
        String reason = null;
        if (!pending.containsKey(event)) reason = "not_FC_authorized";
        else if (status.isLanded()
                || status.getEventQueue().stream()
                        .anyMatch(
                                e ->
                                        e.getType() == FlightEvent.Type.GROUND_HIT
                                                && e.getTime() <= event.getTime() + 1e-9))
            reason = "after_landing";
        else if (!status.getConfiguration().isComponentActive(event.getSource()))
            reason = "inactive_device";
        else if (status.getDeployedRecoveryDevices().contains(event.getSource()))
            reason = "already_deployed";
        if (reason == null) return true;
        rejected(event, reason);
        return false;
    }

    /**
     * Records why a physical deployment command was not accepted.
     *
     * @param event event to process
     * @param reason reason
     */
    public void rejected(FlightEvent event, String reason) {
        if (event.getType() != FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT) return;
        pending.remove(event);
        trace.log(
                "recovery.discard",
                "simulation_s="
                        + event.getTime()
                        + " device="
                        + (event.getSource() == null ? "unknown" : event.getSource().getID())
                        + " reason="
                        + reason);
    }

    /**
     * Acknowledges actual deployment and records its physical timing and device identity.
     *
     * @param status stat in microseconds
     * @param event event to process
     */
    public void deployed(SimulationStatus status, FlightEvent event) {
        var provenance = pending.remove(event);
        if (provenance == null) return;
        trace.log(
                "recovery.deploy",
                Json.createObjectBuilder(provenance.json())
                        .add("actualTime", status.getSimulationTime())
                        .add("latenessUs", status.getSimulationTime() * 1e6 - provenance.dueUs())
                        .build()
                        .toString());
        trace.log("fc.plot_event", FlightComputerData.eventSnapshot(event).toString());
        trace.log(
                "fc.recovery_state",
                Json.createObjectBuilder()
                        .add("time", event.getTime())
                        .add(
                                "values",
                                Json.createObjectBuilder()
                                        .add(
                                                FlightComputerData.recoveryDevice(
                                                                provenance.deviceId().toString(),
                                                                provenance.deviceName())
                                                        .getSymbol(),
                                                1))
                        .build()
                        .toString());
    }

    /**
     * Stores logical channels and actual recovery-device states in the current flight branch.
     *
     * @param branch flight-data branch receiving or providing samples
     * @param status stat in microseconds
     */
    public void record(FlightDataBranch branch, SimulationStatus status) {
        for (var target : targets)
            branch.setValue(
                    target.type,
                    status.getDeployedRecoveryDevices().contains(target.device) ? 1 : 0);
    }

    /**
     * Builds recovery mapping metadata for structured log import.
     *
     * @return the resulting JsonArray
     */
    public JsonArray metadata() {
        var array = Json.createArrayBuilder();
        for (var target : targets)
            array.add(
                    Json.createObjectBuilder()
                            .add("id", target.device.getID().toString())
                            .add("name", target.device.getName()));
        return array.build();
    }

    /** Stops accepting commands and rejects pending deployments at run shutdown. */
    @Override
    public void close() {
        pyro.setFireObserver(null);
        pyro.setOutputObserver(null);
        for (var event : pending.keySet()) rejectedOnClose(event);
        commands.clear();
        pending.clear();
        attempted.clear();
        targets.clear();
    }

    /**
     * Records a pending recovery command discarded when the simulation ends.
     *
     * @param event event to process
     */
    private void rejectedOnClose(FlightEvent event) {
        trace.log(
                "recovery.discard",
                "device="
                        + event.getSource().getID()
                        + " due_s="
                        + event.getTime()
                        + " reason=simulation_ended");
    }
}
