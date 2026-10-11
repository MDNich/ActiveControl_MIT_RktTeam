package edu.mit.rocket_team.util;

import edu.mit.rocket_team.FC.RTFC;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.FlightDataBranch;
import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.simulation.exception.SimulationException;
import info.openrocket.core.simulation.flightcomputer.FlightComputerData;
import jakarta.json.*;
import java.util.List;

/** Per-flight physical outputs, with ideal motion after the independent PWM latch. */
public class RTSimulationCommunicator {
    private List<AirbrakeSet> airbrakes = List.of();
    private List<TabControlledTrapezoidFinSet> rollFins = List.of();
    private boolean airbrakesConnected, rollConnected, bound, heldClosed;
    private RTFC.Output applied;
    private info.openrocket.core.simulation.flightcomputer.FlightComputerControllerActuators
            controllers;

    /**
     * Installs the controller-block actuator bindings used for ownership and PWM output
     * application.
     *
     * @param status stat in microseconds
     * @param design external flight-computer design snapshot
     * @param settings validated configuration for this operation
     * @param fc fc
     * @throws IllegalArgumentException when rejected: Controller assignments exist but this flight
     *     computer has no controller blocks
     */
    public void bindControllers(
            SimulationStatus status,
            info.openrocket.core.simulation.flightcomputer.FlightComputerDesign design,
            info.openrocket.core.simulation.flightcomputer.FlightComputerControllerSettings
                    settings,
            RTFC fc) {
        if (fc.getControllers() == null) {
            if (!settings.bindings().isEmpty())
                throw new IllegalArgumentException(
                        "Controller assignments exist but this flight computer has no controller blocks");
            return;
        }
        controllers =
                new info.openrocket.core.simulation.flightcomputer
                        .FlightComputerControllerActuators(
                        status, design, settings, fc.getControllers(), trace);
        fc.setControllerWriteCheck(controllers::checkJavaWrite);
    }

    /**
     * Tests whether an enabled controller block reserves the physical component and output mode.
     *
     * @param mode mode
     * @param c selected channel or controller
     * @return true when the condition described above holds
     */
    private boolean reserved(
            info.openrocket.core.simulation.flightcomputer.FlightComputerController.Mode mode,
            RocketComponent c) {
        return controllers != null && controllers.reserved(mode, c);
    }

    private final RTUtilLibrary.Trace trace;

    /**
     * Creates a RTSimulationCommunicator with the supplied dependencies and per-instance state.
     *
     * @param trace run-local virtual clock and action-log sink
     */
    public RTSimulationCommunicator(RTUtilLibrary.Trace trace) {
        this.trace = trace;
    }

    /**
     * Finds active airbrakes and tab-controlled fins in the per-run rocket and validates required
     * physical hardware.
     *
     * @param status stat in microseconds
     * @throws SimulationException if the requested operation cannot be completed
     */
    public void bind(SimulationStatus status) throws SimulationException {
        bind(status, true, true);
    }

    /**
     * Finds active airbrakes and tab-controlled fins in the per-run rocket and validates required
     * physical hardware.
     *
     * @param status stat in microseconds
     * @param connected connected
     * @throws SimulationException if the requested operation cannot be completed
     */
    public void bind(SimulationStatus status, boolean connected) throws SimulationException {
        bind(status, connected, true);
    }

    /**
     * Finds active airbrakes and tab-controlled fins in the per-run rocket and validates required
     * physical hardware.
     *
     * @param status stat in microseconds
     * @param airbrakesConnected airbrakes connected value
     * @param rollConnected roll connected value
     * @throws SimulationException if the requested operation cannot be completed
     */
    public void bind(SimulationStatus status, boolean airbrakesConnected, boolean rollConnected)
            throws SimulationException {
        var brakes = new java.util.ArrayList<AirbrakeSet>();
        var fins = new java.util.ArrayList<TabControlledTrapezoidFinSet>();
        for (RocketComponent component : status.getConfiguration().getActiveComponents()) {
            if (component instanceof AirbrakeSet set) brakes.add(set);
            if (component instanceof TabControlledTrapezoidFinSet set) fins.add(set);
        }
        if (airbrakesConnected && brakes.isEmpty())
            throw new SimulationException("FC requires an active AirbrakeSet");
        if (bound
                && airbrakes.equals(brakes)
                && rollFins.equals(fins)
                && this.airbrakesConnected == airbrakesConnected
                && this.rollConnected == rollConnected) return;
        bound = true;
        this.airbrakesConnected = airbrakesConnected;
        this.rollConnected = rollConnected;
        airbrakes = List.copyOf(brakes);
        rollFins = List.copyOf(fins);
        applied = null;
        trace.log(
                "component.bind",
                "airbrakes_connected="
                        + airbrakesConnected
                        + " airbrake_sets="
                        + brakes.size()
                        + " roll_connected="
                        + rollConnected
                        + " tab_fin_sets="
                        + fins.size()
                        + " linkage=ideal_1_to_1");
        if (rollConnected && fins.isEmpty())
            trace.log(
                    "roll.warning",
                    "reason=no_active_TabControlledTrapezoidFinSet commands_recorded_only=true");
        for (var fin : fins) {
            trace.log(
                    "component.roll_tabs",
                    "id="
                            + fin.getID()
                            + " name="
                            + fin.getName()
                            + " cna_per_rad="
                            + fin.getCNALPHA()
                            + " tab_area_m2="
                            + (fin.getTabChord() * fin.getTabSpan()));
            if (rollConnected
                    && (fin.getCNALPHA() == 0 || fin.getTabChord() == 0 || fin.getTabSpan() == 0))
                trace.log(
                        "roll.warning",
                        "id="
                                + fin.getID()
                                + " reason=zero_tab_aerodynamic_authority check=CNAlpha_span_chord");
        }
        for (var brake : brakes)
            trace.log("component.airbrakes", "id=" + brake.getID() + " name=" + brake.getName());
    }

    /** A name targets exactly one active rocket component of the appropriate type. */
    private static void validateTargets(
            java.util.Set<String> targets,
            List<? extends RocketComponent> components,
            String type) {
        for (String name : targets) {
            long matches = components.stream().filter(c -> c.getName().equals(name)).count();
            if (matches != 1)
                throw new IllegalArgumentException(
                        "FC "
                                + type
                                + " target '"
                                + name
                                + "' matches "
                                + matches
                                + " active rocket components; use a unique, exact rocket component name. Available: "
                                + components.stream().map(RocketComponent::getName).toList());
        }
    }

    /**
     * Applies PWM-latched commands to the per-run rocket, respecting controller reservations,
     * missing outputs and disabled-PWM hold behavior.
     *
     * @param output output snapshot or destination
     * @param holdClosed whether airbrakes must remain closed
     */
    public void apply(RTFC.Output output, boolean holdClosed) {
        if (applied == output && heldClosed == holdClosed) return;
        // Validate the entire latched update before actuating any named output.
        validateTargets(output.rollAngles().keySet(), rollFins, "roll");
        validateTargets(output.airbrakeFractions().keySet(), airbrakes, "airbrakes");
        applied = output;
        heldClosed = holdClosed;
        for (var brake : airbrakes) {
            if (reserved(
                    info.openrocket.core.simulation.flightcomputer.FlightComputerController.Mode
                            .AIRBRAKES,
                    brake)) continue;
            double fraction =
                    holdClosed || !airbrakesConnected ? 0 : airbrakeCommand(output, brake);
            brake.setFracExposed(fraction);
            trace.log(
                    "airbrakes.actuate",
                    "id="
                            + brake.getID()
                            + " name="
                            + brake.getName()
                            + " exposed="
                            + fraction
                            + " area_m2="
                            + brake.getExposedArea()
                            + " ideal_linkage=true hold_closed="
                            + holdClosed);
        }
        if (!airbrakesConnected) trace.log("airbrakes.discard", "reason=no_fc_output");
        if (!rollConnected) {
            for (var fin : rollFins)
                if (!reserved(
                        info.openrocket.core.simulation.flightcomputer.FlightComputerController.Mode
                                .TABS,
                        fin)) fin.setTabAngle(0);
            trace.log("roll.discard", "reason=no_fc_output tabs_neutral=true");
        } else if (rollFins.isEmpty()) {
            trace.log("roll.discard", "reason=no_physical_tabs angle_deg=" + output.rollAngle());
        } else if (output.servo2Us() == 0 || output.servo3Us() == 0) {
            // No signal is not a new zero-angle command. Ideal servos hold position.
            trace.log("roll.hold", "reason=pwm_disabled");
        } else {
            for (var fin : rollFins) {
                if (reserved(
                        info.openrocket.core.simulation.flightcomputer.FlightComputerController.Mode
                                .TABS,
                        fin)) continue;
                double angle = rollCommand(output, fin);
                fin.setTabAngle(angle);
                trace.log(
                        "roll.actuate",
                        "id="
                                + fin.getID()
                                + " name="
                                + fin.getName()
                                + " angle_deg="
                                + Math.toDegrees(angle)
                                + " ideal_linkage=true");
            }
        }
        if (controllers != null) controllers.apply(output, holdClosed);
    }

    /**
     * Resolves a named or broadcast roll command and converts degrees to the fin API's radians.
     *
     * @param out output destination or latched values
     * @param fin fin
     * @return computed numeric value
     */
    private double rollCommand(RTFC.Output out, TabControlledTrapezoidFinSet fin) {
        return Math.toRadians(
                rollConnected ? out.rollAngles().getOrDefault(fin.getName(), out.rollAngle()) : 0);
    }

    /**
     * Resolves a named or broadcast normalized airbrake deployment command.
     *
     * @param out output destination or latched values
     * @param brake brake
     * @return computed numeric value
     */
    private double airbrakeCommand(RTFC.Output out, AirbrakeSet brake) {
        return airbrakesConnected
                ? out.airbrakeFractions().getOrDefault(brake.getName(), out.exposedFraction())
                : 0;
    }

    /**
     * Builds actuator identity metadata for log import and plot channel reconstruction.
     *
     * @return the resulting JsonArray
     */
    public JsonArray plotMetadata() {
        var array = Json.createArrayBuilder();
        for (var fin : rollFins) array.add(metadata("roll", fin));
        for (var brake : airbrakes) array.add(metadata("airbrakes", brake));
        if (controllers != null) controllers.metadata().forEach(array::add);
        return array.build();
    }

    /**
     * Builds one actuator metadata entry using its stable component UUID and display name.
     *
     * @param port logical controller port name
     * @param component component
     * @return the resulting JsonObject
     */
    private JsonObject metadata(String port, RocketComponent component) {
        return Json.createObjectBuilder()
                .add("port", port)
                .add("id", component.getID().toString())
                .add("name", component.getName())
                .build();
    }

    /**
     * Records commanded and physical actuator values, including independently named targets and
     * controller bindings.
     *
     * @param branch flight-data branch receiving or providing samples
     * @param out output destination or latched values
     */
    public void recordData(FlightDataBranch branch, RTFC.Output out) {
        branch.setValue(
                FlightComputerData.ROLL_TAB_ANGLE,
                common(rollFins.stream().map(TabControlledTrapezoidFinSet::getTabAngle).toList()));
        branch.setValue(
                FlightComputerData.AIRBRAKE_PHYSICAL,
                common(airbrakes.stream().map(AirbrakeSet::getFracExposed).toList()));
        for (var fin : rollFins) {
            var channels =
                    FlightComputerData.actuatorChannels(
                            "roll", fin.getID().toString(), fin.getName());
            branch.setValue(channels.command(), rollCommand(out, fin));
            branch.setValue(channels.physical(), fin.getTabAngle());
        }
        for (var brake : airbrakes) {
            var channels =
                    FlightComputerData.actuatorChannels(
                            "airbrakes", brake.getID().toString(), brake.getName());
            branch.setValue(channels.command(), airbrakeCommand(out, brake));
            branch.setValue(channels.physical(), brake.getFracExposed());
        }
        if (controllers != null) controllers.record(branch);
    }

    /**
     * Returns the shared value when all physical actuators agree, or NaN for empty or differing
     * values.
     *
     * @param values values to process
     * @return the shared value when all physical actuators agree, or NaN for empty or differing
     *     values
     */
    private static double common(List<Double> values) {
        if (values.isEmpty()) return Double.NaN;
        double first = values.get(0);
        return values.stream().allMatch(v -> Math.abs(v - first) < 1e-10) ? first : Double.NaN;
    }
}
