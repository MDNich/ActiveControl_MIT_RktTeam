package info.openrocket.core.simulation.mission;

import edu.mit.rocket_team.RTFCVerificationRocket;
import edu.mit.rocket_team.util.RTRocketState;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.simulation.listeners.AbstractSimulationListener;
import info.openrocket.core.util.BaseTestCase;
import jakarta.json.*;
import java.nio.file.Path;
import java.util.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import static org.junit.jupiter.api.Assertions.*;

class MissionTelemetryStateTest extends BaseTestCase {
    @TempDir Path temp;

    private JsonObject binding(MissionTruth truth) {
        return Json.createObjectBuilder().add("id", "rocket")
                .add("componentId", truth.branches().get(0).componentId).build();
    }

    private MissionTelemetrySource source(JsonObject binding) {
        return new MissionTelemetrySource(JsonValue.EMPTY_JSON_OBJECT, binding,
                Json.createObjectBuilder().add("mslHeightM", 271).build(), "42", 0);
    }

    private long event(MissionTruth truth, String kind) {
        return truth.branches().get(0).events.stream().filter(e -> e.getString("kind").equals(kind))
                .mapToLong(e -> Long.parseLong(e.getString("timeUs"))).findFirst().orElseThrow();
    }

    @ParameterizedTest @EnumSource(value = DeploymentConfiguration.DeployEvent.class, names = {"APOGEE", "ALTITUDE", "EJECTION"})
    void standardWireStatesFollowLiftoffApogeeRecoveryAndLanding(DeploymentConfiguration.DeployEvent trigger) throws Exception {
        var rocket = RTFCVerificationRocket.rocket();
        for (var component : rocket) {
            if (component instanceof Parachute chute) {
                var deploy = chute.getDeploymentConfigurations().getDefault();
                deploy.setDeployEvent(trigger);
                deploy.setDeployAltitude(300);
                deploy.setDeployDelay(1);
            }
            if (component instanceof MotorMount mount && mount.isMotorMount())
                mount.getMotorConfig(rocket.getSelectedConfiguration().getFlightConfigurationID()).setEjectionDelay(15);
        }
        var flight = RTFCVerificationRocket.simulation(rocket);
        flight.getOptions().setTimeStep(.05); flight.getOptions().setMaxSimulationTime(600);
        var truth = MissionTruth.compute(flight, () -> {});
        var binding = binding(truth);
        long launch = event(truth, "LIFTOFF"), apogee = event(truth, "APOGEE"), landed = event(truth, "GROUND_HIT");
        assertEquals(RTRocketState.PRE_FLIGHT, truth.phaseAt(binding, -1_000_000));
        assertEquals(RTRocketState.PRE_FLIGHT, truth.phaseAt(binding, launch - 1));
        assertEquals(RTRocketState.FLIGHT, truth.phaseAt(binding, launch));
        assertEquals(RTRocketState.FLIGHT, truth.phaseAt(binding, apogee - 1));
        assertEquals(RTRocketState.APOGEE, truth.phaseAt(binding, apogee));
        long deployed = event(truth, "RECOVERY_DEVICE_DEPLOYMENT");
        assertEquals(RTRocketState.APOGEE, truth.phaseAt(binding, deployed - 1));
        assertEquals(RTRocketState.MAIN, truth.phaseAt(binding, deployed));
        assertFalse(truth.displayAt(binding, deployed - 1, "before-deployment").getBoolean("parachute_deployed"));
        assertTrue(truth.displayAt(binding, deployed, "deployment").getBoolean("parachute_deployed"));
        assertEquals(RTRocketState.END, truth.phaseAt(binding, landed));
        var preflightDisplay = truth.displayAt(binding, -1_000_000, "preflight");
        assertFalse(preflightDisplay.getBoolean("motor_burning"));
        assertFalse(preflightDisplay.getBoolean("parachute_deployed"));
        var burningDisplay = truth.displayAt(binding, launch, "burning");
        assertTrue(burningDisplay.getBoolean("motor_burning"));
        assertEquals("body_to_ENU", burningDisplay.getString("frame"));
        assertEquals("+Z", burningDisplay.getString("body_axis"));
        var landedDisplay = truth.displayAt(binding, landed, "landed");
        assertFalse(landedDisplay.getBoolean("motor_burning"));
        assertTrue(landedDisplay.getBoolean("motor_ignited"));
        assertTrue(landedDisplay.getBoolean("parachute_deployed"));
        double[] expectedQuaternion = truth.at(binding, landed).quaternion();
        for (int i = 0; i < 4; i++) assertEquals(expectedQuaternion[i],
                landedDisplay.getJsonArray("quaternion_wxyz").getJsonNumber(i).doubleValue());

        var source = source(binding);
        var codes = new ArrayList<Integer>();
        while (source.next() <= truth.end()) {
            var packet = source.advance(source.next(), truth);
            if (packet == null) continue;
            codes.add(packet.payload()[63] & 255);
            int checksum = 0;
            for (int i = 0; i < 127; i++) checksum += packet.payload()[i] & 255;
            assertEquals(checksum & 255, packet.payload()[127] & 255);
            assertEquals("zephyrus-v1", packet.metadata().getString("stateSchema"));
            assertFalse(packet.metadata().getJsonArray("unavailable").contains(Json.createValue("state")));
        }
        var finalPacket = source.finish(truth.end(), truth);
        if (finalPacket != null) codes.add(finalPacket.payload()[63] & 255);
        assertEquals(1, codes.get(0));
        assertTrue(codes.containsAll(List.of(1, 2, 3, 4, 5)));
        assertEquals(5, codes.get(codes.size() - 1));
        assertNull(source.finish(truth.end(), truth));

        Path archive = temp.resolve("state.rktmission");
        try (var recorder = new MissionRecorder(temp.resolve("staging"), MissionDefinition.createUrrg("State test").json())) {
            truth.record(recorder, "member-0"); recorder.finish(archive, "complete");
        }
        try (var result = new MissionArchive(archive)) {
            var replay = MissionTruth.read(result, "runs/member-0/", () -> {});
            for (long time : new long[]{-1_000_000, launch, apogee, deployed - 1, deployed, landed - 1, landed}) {
                assertEquals(truth.phaseAt(binding, time), replay.phaseAt(binding, time));
                assertEquals(truth.displayAt(binding, time, "replay").getBoolean("parachute_deployed"),
                        replay.displayAt(binding, time, "replay").getBoolean("parachute_deployed"));
            }
        }
    }

    @Test void vetoedDeploymentNeverPublishesADeployedParachute() throws Exception {
        var flight = RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());
        flight.getOptions().setMaxSimulationTime(600);
        var truth = MissionTruth.compute(flight, () -> {}, new AbstractSimulationListener() {
            @Override public boolean recoveryDeviceDeployment(SimulationStatus status, RecoveryDevice device) { return false; }
        });
        var binding = binding(truth);
        assertTrue(truth.branches().stream().flatMap(b -> b.events.stream())
                .noneMatch(e -> e.getString("kind").equals("RECOVERY_DEVICE_DEPLOYMENT")));
        long apogee = event(truth, "APOGEE");
        assertEquals(RTRocketState.APOGEE, truth.phaseAt(binding, apogee));
        assertFalse(truth.displayAt(binding, truth.end(), "no-deployment").getBoolean("parachute_deployed"));
        var source = source(binding);
        while (source.next() <= truth.end()) {
            var packet = source.advance(source.next(), truth);
            if (packet != null) assertNotEquals(4, packet.payload()[63] & 255);
        }
    }

    @Test void stoppingAFlightEarlyIsNotAnEndState() throws Exception {
        var flight = RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());
        flight.getOptions().setMaxSimulationTime(2);
        var truth = MissionTruth.compute(flight, () -> {});
        var binding = binding(truth);
        assertEquals(RTRocketState.FLIGHT, truth.phaseAt(binding, truth.end()));
        assertNull(source(binding).finish(truth.end(), truth));
    }
}
