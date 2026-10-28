package info.openrocket.core.simulation.mission;

import info.openrocket.core.document.Simulation;
import info.openrocket.core.motor.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.util.Config;
import jakarta.json.*;
import java.util.*;
import java.util.function.Function;

/** Complete editable flight view over an isolated native simulation, without the old simulation dialog. */
public final class MissionFlightEditor {
    private MissionFlightEditor() { }

    public static JsonObject capture(Simulation simulation) {
        var stages = Json.createArrayBuilder(); var motors = Json.createArrayBuilder(); var recovery = Json.createArrayBuilder();
        var extensions = Json.createArrayBuilder(); var id = simulation.getId();
        for (var component : simulation.getRocket()) {
            var b = Json.createObjectBuilder().add("componentId", component.getID().toString()).add("name", component.getName());
            if (component instanceof AxialStage stage) {
                var separation = stage.getSeparationConfigurations().get(id);
                stages.add(b.add("active", simulation.getActiveConfiguration().isStageActive(stage.getStageNumber()))
                        .add("event", separation.getSeparationEvent().name()).add("altitudeM", separation.getSeparationAltitude())
                        .add("delayS", separation.getSeparationDelay()));
            }
            if (component instanceof MotorMount mount && mount.isMotorMount()) {
                var motor = mount.getMotorConfig(id);
                if (motor == null) motor = new MotorConfiguration(mount, id);
                b.add("motorDigest", motor.getMotor() instanceof ThrustCurveMotor curve ? curve.getDigest() : "")
                        .add("motorName", motor.getMotor() == null ? "No motor" : motor.getMotor().getDesignation())
                        .add("ignitionOverride", motor.hasIgnitionOverride()).add("event", motor.getIgnitionEvent().name())
                        .add("delayS", motor.getIgnitionDelay());
                if (Double.isFinite(motor.getEjectionDelay())) b.add("ejectionDelayS", motor.getEjectionDelay()); else b.addNull("ejectionDelayS");
                motors.add(b);
            }
            if (component instanceof RecoveryDevice device) {
                var deployment = device.getDeploymentConfigurations().get(id);
                recovery.add(b.add("event", deployment.getDeployEvent().name()).add("altitudeM", deployment.getDeployAltitude())
                        .add("delayS", deployment.getDeployDelay()));
            }
        }
        for (var extension : simulation.getSimulationExtensions()) {
            var config = Json.createObjectBuilder();
            for (String key : extension.getConfig().keySet()) config.add(key, json(extension.getConfig().get(key, null)));
            extensions.add(Json.createObjectBuilder().add("id", extension.getId()).add("name", extension.getName()).add("config", config));
        }
        return Json.createObjectBuilder().add("options", MissionFlightOptions.capture(simulation.getOptions()))
                .add("stages", stages).add("motors", motors).add("recovery", recovery).add("extensions", extensions).build();
    }

    /** Caller supplies an isolated snapshot. Never invoke against a live design during a run. */
    public static void apply(Simulation simulation, JsonObject data, Function<String, Motor> motorLookup) {
        var options = MissionFlightOptions.restore(data.getJsonObject("options"));
        var components = new HashMap<String, RocketComponent>();
        for (var c : simulation.getRocket()) components.put(c.getID().toString(), c);
        var id = simulation.getId();
        requireInventory(capture(simulation), data);
        for (var entry : data.getJsonArray("stages")) {
            var value = entry.asJsonObject(); var stage = (AxialStage) components.get(value.getString("componentId"));
            var separation = stage.getSeparationConfigurations().get(id).clone();
            separation.setSeparationEvent(StageSeparationConfiguration.SeparationEvent.valueOf(value.getString("event")));
            separation.setSeparationAltitude(number(value, "altitudeM", -1000, 1e7));
            separation.setSeparationDelay(number(value, "delayS", 0, 1e6));
            stage.getSeparationConfigurations().set(id, separation);
            simulation.getActiveConfiguration()._setStageActive(stage.getStageNumber(), value.getBoolean("active"), false);
        }
        for (var entry : data.getJsonArray("motors")) {
            var value = entry.asJsonObject(); var mount = (MotorMount) components.get(value.getString("componentId"));
            String digest = value.getString("motorDigest"); var current = mount.getMotorConfig(id);
            Motor motor = current == null ? null : current.getMotor();
            if (digest.isEmpty()) motor = null;
            else if (!(motor instanceof ThrustCurveMotor curve) || !curve.getDigest().equals(digest)) motor = motorLookup.apply(digest);
            if (!digest.isEmpty() && motor == null) throw new IllegalArgumentException("Motor curve unavailable: " + digest);
            var config = new MotorConfiguration(mount, id); config.setMotor(motor);
            config.setEjectionDelay(value.isNull("ejectionDelayS") ? Motor.PLUGGED_DELAY : number(value, "ejectionDelayS", 0, 1e6));
            if (value.getBoolean("ignitionOverride")) {
                config.setIgnitionEvent(IgnitionEvent.valueOf(value.getString("event")));
                config.setIgnitionDelay(number(value, "delayS", 0, 1e6));
            } else config.useDefaultIgnition();
            mount.setMotorConfig(config, id);
        }
        for (var entry : data.getJsonArray("recovery")) {
            var value = entry.asJsonObject(); var device = (RecoveryDevice) components.get(value.getString("componentId"));
            var deployment = device.getDeploymentConfigurations().get(id).clone();
            deployment.setDeployEvent(DeploymentConfiguration.DeployEvent.valueOf(value.getString("event")));
            deployment.setDeployAltitude(number(value, "altitudeM", 0, 1e7));
            deployment.setDeployDelay(number(value, "delayS", 0, 1e6));
            device.getDeploymentConfigurations().set(id, deployment);
        }
        var extensions = data.getJsonArray("extensions");
        for (int i = 0; i < extensions.size(); i++) {
            var value = extensions.getJsonObject(i); var extension = simulation.getSimulationExtensions().get(i);
            if (!extension.getId().equals(value.getString("id"))) throw new IllegalArgumentException("Extension identity changed");
            if (extension instanceof info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer) {
                for (String key : List.of("designFile", "designId", "computer")) {
                    Object old = extension.getConfig().get(key, null);
                    Object next = value.getJsonObject("config").containsKey(key) ? object(value.getJsonObject("config").get(key)) : null;
                    if (!Objects.equals(old, next)) throw new IllegalArgumentException("Use Choose flight computer to change the frozen FC identity");
                }
            }
            var config = new Config(); value.getJsonObject("config").forEach((key, v) -> config.put(key, object(v)));
            extension.setConfig(config);
        }
        simulation.copySimulationOptionsFrom(options); simulation.getOptions().setRandomSeed(options.getRandomSeed());
    }

    private static void requireInventory(JsonObject original, JsonObject edited) {
        for (String group : List.of("stages", "motors", "recovery")) {
            var a = new HashSet<String>(); var b = new HashSet<String>();
            original.getJsonArray(group).forEach(v -> a.add(v.asJsonObject().getString("componentId")));
            edited.getJsonArray(group).forEach(v -> b.add(v.asJsonObject().getString("componentId")));
            if (!a.equals(b) || b.size() != edited.getJsonArray(group).size()) throw new IllegalArgumentException("Incomplete " + group + " inventory");
        }
        if (edited.getJsonArray("extensions").size() != original.getJsonArray("extensions").size())
            throw new IllegalArgumentException("Extension inventory changed; use the extension selection action");
    }
    private static double number(JsonObject data, String key, double min, double max) { return MissionFlightOptions.number(data, key, min, max); }
    private static JsonValue json(Object value) {
        if (value instanceof Boolean b) return b ? JsonValue.TRUE : JsonValue.FALSE;
        if (value instanceof Number n) return Json.createValue(new java.math.BigDecimal(n.toString()));
        if (value instanceof String s) return Json.createValue(s);
        if (value instanceof List<?> l) { var a = Json.createArrayBuilder(); l.forEach(v -> a.add(json(v))); return a.build(); }
        throw new IllegalArgumentException("Unsupported extension configuration value");
    }
    private static Object object(JsonValue value) {
        if (value instanceof JsonNumber n) return n.bigDecimalValue();
        if (value instanceof JsonString s) return s.getString();
        if (value == JsonValue.TRUE) return true;
        if (value == JsonValue.FALSE) return false;
        if (value instanceof JsonArray a) return a.stream().map(MissionFlightEditor::object).toList();
        throw new IllegalArgumentException("Extension settings require scalar values or lists");
    }
}
