package info.openrocket.core.simulation.mission;

import info.openrocket.core.file.MotorFinder;
import info.openrocket.core.motor.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.util.Coordinate;
import jakarta.json.*;
import java.util.*;

/** Frozen thrust/mass/CG data prevents a different local motor database changing a mission. */
public final class MissionMotorSnapshot {
    private MissionMotorSnapshot() { }

    public static JsonObject capture(Rocket rocket) {
        var motors = new LinkedHashMap<String, JsonObject>();
        for (var component : rocket) if (component instanceof MotorMount mount) {
            var iterator = mount.getMotorIterator();
            while (iterator.hasNext()) {
                Motor motor = iterator.next().getMotor();
                if (motor == null) continue;
                if (!(motor instanceof ThrustCurveMotor m)) throw new IllegalArgumentException("Mission snapshots need thrust-curve motors");
                var cg = Json.createArrayBuilder();
                for (Coordinate c : m.getCGPoints()) cg.add(Json.createArrayBuilder().add(c.x).add(c.y).add(c.z).add(c.weight));
                JsonObject definition = Json.createObjectBuilder().add("digest", m.getDigest())
                        .add("manufacturer", m.getManufacturer().getDisplayName()).add("designation", m.getDesignation())
                        .add("code", m.getCode()).add("commonName", m.getCommonName()).add("description", m.getDescription())
                        .add("type", m.getMotorType().name()).add("diameter", m.getDiameter()).add("length", m.getLength())
                        .add("initialMass", m.getInitialMass()).add("time", array(m.getTimePoints()))
                        .add("thrust", array(m.getThrustPoints())).add("cg", cg).add("delays", array(m.getStandardDelays())).build();
                String key = m.getDigest() + "/" + m.getManufacturer().getDisplayName() + "/" + m.getDesignation();
                JsonObject prior = motors.putIfAbsent(key, definition);
                if (prior != null && !prior.equals(definition)) throw new IllegalArgumentException("Conflicting motor identities");
            }
        }
        var values = Json.createArrayBuilder(); motors.values().forEach(values::add);
        return Json.createObjectBuilder().add("format", "openrocket.mission-motors").add("version", 1).add("motors", values).build();
    }

    private static JsonArray array(double[] values) {
        var a = Json.createArrayBuilder();
        for (double value : values) { if (value == Double.POSITIVE_INFINITY) a.addNull(); else a.add(value); }
        return a.build();
    }

    private static double[] doubles(JsonArray a) {
        double[] values = new double[a.size()];
        for (int i = 0; i < a.size(); i++) values[i] = a.isNull(i) ? Double.POSITIVE_INFINITY : a.getJsonNumber(i).doubleValue();
        return values;
    }

    public static MotorFinder finder(JsonObject snapshot) {
        if (!"openrocket.mission-motors".equals(snapshot.getString("format", "")) || snapshot.getInt("version", -1) != 1)
            throw new IllegalArgumentException("Unsupported motor snapshot");
        var motors = new ArrayList<ThrustCurveMotor>();
        for (var value : snapshot.getJsonArray("motors")) {
            var m = value.asJsonObject(); var points = m.getJsonArray("cg");
            Coordinate[] cg = new Coordinate[points.size()];
            for (int i = 0; i < points.size(); i++) {
                var c = points.getJsonArray(i);
                cg[i] = new Coordinate(c.getJsonNumber(0).doubleValue(), c.getJsonNumber(1).doubleValue(), c.getJsonNumber(2).doubleValue(), c.getJsonNumber(3).doubleValue());
            }
            motors.add(new ThrustCurveMotor.Builder().setDigest(m.getString("digest"))
                    .setManufacturer(Manufacturer.getManufacturer(m.getString("manufacturer"))).setDesignation(m.getString("designation"))
                    .setCode(m.getString("code")).setCommonName(m.getString("commonName")).setDescription(m.getString("description"))
                    .setMotorType(Motor.Type.valueOf(m.getString("type"))).setDiameter(m.getJsonNumber("diameter").doubleValue())
                    .setLength(m.getJsonNumber("length").doubleValue()).setInitialMass(m.getJsonNumber("initialMass").doubleValue())
                    .setTimePoints(doubles(m.getJsonArray("time"))).setThrustPoints(doubles(m.getJsonArray("thrust")))
                    .setCGPoints(cg).setStandardDelays(doubles(m.getJsonArray("delays"))).build());
        }
        return (type, manufacturer, designation, diameter, length, digest, warnings) -> {
            var matches = motors.stream().filter(m -> digest != null && !digest.isBlank() ? digest.equals(m.getDigest())
                    : (manufacturer == null || m.getManufacturer().matches(manufacturer)) && Objects.equals(designation, m.getDesignation())).toList();
            if (matches.size() != 1) throw new IllegalArgumentException("Frozen motor is missing or ambiguous: " + designation);
            return matches.get(0);
        };
    }
}
