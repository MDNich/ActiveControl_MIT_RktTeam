package info.openrocket.core.simulation.mission;

import info.openrocket.core.models.wind.*;
import info.openrocket.core.simulation.SimulationOptions;
import info.openrocket.core.simulation.ensemble.EnsembleSettings;
import info.openrocket.core.util.GeodeticComputationStrategy;
import jakarta.json.*;
import java.util.*;

/** Lossless explicit settings companion to the native rocket/extension snapshot. SI units throughout. */
public final class MissionFlightOptions {
    private MissionFlightOptions() { }

    public static JsonObject capture(SimulationOptions o) {
        var levels = Json.createArrayBuilder();
        for (var level : o.getMultiLevelWindModel().getLevels()) levels.add(Json.createObjectBuilder()
                .add("altitudeM", level.getAltitude()).add("speedMS", level.getSpeed())
                .add("directionRad", level.getDirection()).add("sigmaMS", level.getStandardDeviation()));
        var ensemble = Json.createObjectBuilder();
        new TreeMap<>(o.getEnsembleSettings().attributes()).forEach(ensemble::add);
        return Json.createObjectBuilder().add("format", "openrocket.mission-flight-options").add("version", 1)
                .add("guideLengthM", o.getLaunchRodLength()).add("guideAngleRad", o.getLaunchRodAngle())
                .add("manualBearingRad", o.getLaunchRodDirectionSetting()).add("intoWind", o.getLaunchIntoWind())
                .add("initialRollRateRadS", o.getInitialRollRate())
                .add("launchLatitudeDeg", o.getLaunchLatitude()).add("launchLongitudeDeg", o.getLaunchLongitude())
                .add("launchMslM", o.getLaunchAltitude()).add("geodeticMethod", o.getGeodeticComputation().name())
                .add("isa", o.isISAAtmosphere()).add("temperatureK", o.getLaunchTemperature()).add("pressurePa", o.getLaunchPressure())
                .add("timeStepS", o.getTimeStep()).add("maximumTimeS", o.getMaxSimulationTime())
                .add("maximumStepAngleRad", o.getMaximumStepAngle()).add("randomSeed", o.getRandomSeed())
                .add("windModel", o.getWindModelType().name())
                .add("averageWind", Json.createObjectBuilder().add("speedMS", o.getAverageWindModel().getAverage())
                        .add("directionRad", o.getAverageWindModel().getDirection()).add("sigmaMS", o.getAverageWindModel().getStandardDeviation()))
                .add("windAltitudeReference", o.getMultiLevelWindModel().getAltitudeReference().name())
                .add("windLevels", levels).add("ensemble", ensemble).build();
    }

    /** Reconstruct independently; never partially mutates an existing simulation on validation failure. */
    public static SimulationOptions restore(JsonObject data) {
        if (!"openrocket.mission-flight-options".equals(data.getString("format", "")) || data.getInt("version", -1) != 1)
            throw new IllegalArgumentException("Unsupported flight options snapshot");
        var o = new SimulationOptions();
        o.setLaunchIntoWind(false);
        o.setLaunchRodLength(number(data, "guideLengthM", 0, 10000));
        o.setLaunchRodAngle(number(data, "guideAngleRad", -Math.PI / 2, Math.PI / 2));
        o.setLaunchRodDirection(number(data, "manualBearingRad", 0, 2 * Math.PI));
        o.setInitialRollRate(number(data, "initialRollRateRadS", -1e6, 1e6));
        o.setLaunchLatitude(number(data, "launchLatitudeDeg", -90, 90));
        o.setLaunchLongitude(number(data, "launchLongitudeDeg", -180, 180));
        o.setLaunchAltitude(number(data, "launchMslM", -1000, 100000));
        o.setGeodeticComputation(GeodeticComputationStrategy.valueOf(data.getString("geodeticMethod")));
        o.setISAAtmosphere(data.getBoolean("isa"));
        o.setLaunchTemperature(number(data, "temperatureK", 0.001, 2000));
        o.setLaunchPressure(number(data, "pressurePa", 0.001, 1e8));
        o.setTimeStep(number(data, "timeStepS", 1e-6, 10));
        o.setMaxSimulationTime(number(data, "maximumTimeS", 0.001, 1e7));
        o.setMaximumStepAngle(number(data, "maximumStepAngleRad", 1e-9, Math.PI));
        o.setRandomSeed(data.getJsonNumber("randomSeed").intValueExact());
        var wind = data.getJsonObject("averageWind");
        o.getAverageWindModel().setAverage(number(wind, "speedMS", 0, 10000));
        o.getAverageWindModel().setDirection(number(wind, "directionRad", 0, 2 * Math.PI));
        o.getAverageWindModel().setStandardDeviation(number(wind, "sigmaMS", 0, 10000));
        var multi = o.getMultiLevelWindModel();
        multi.clearLevels();
        var altitudes = new HashSet<Double>();
        if (data.getJsonArray("windLevels").size() > 10000) throw new IllegalArgumentException("Too many wind levels");
        for (var value : data.getJsonArray("windLevels")) {
            var level = value.asJsonObject();
            double altitude = number(level, "altitudeM", -1000, 1e7);
            if (!altitudes.add(altitude)) throw new IllegalArgumentException("Duplicate wind level altitude");
            multi.addWindLevel(altitude, number(level, "speedMS", 0, 10000), number(level, "directionRad", 0, 2 * Math.PI), number(level, "sigmaMS", 0, 10000));
        }
        multi.setAltitudeReference(WindModel.AltitudeReference.valueOf(data.getString("windAltitudeReference")));
        o.setWindModelType(WindModelType.valueOf(data.getString("windModel")));
        if (o.getWindModelType() == WindModelType.MULTI_LEVEL && multi.getLevels().isEmpty())
            throw new IllegalArgumentException("Multi-level wind needs at least one level");
        var attributes = new LinkedHashMap<String, String>();
        data.getJsonObject("ensemble").forEach((k, v) -> attributes.put(k, ((JsonString) v).getString()));
        o.setEnsembleSettings(EnsembleSettings.fromAttributes(attributes));
        o.setLaunchIntoWind(data.getBoolean("intoWind"));
        // Detect clamping instead of accepting settings the editor could not faithfully apply.
        JsonObject restored = capture(o);
        for (String key : restored.keySet()) if (!equivalent(restored.get(key), data.get(key)))
            throw new IllegalArgumentException("Flight setting was normalized or lost: " + key);
        return o;
    }

    private static boolean equivalent(JsonValue a, JsonValue b) {
        if (a instanceof JsonNumber x && b instanceof JsonNumber y)
            return Math.abs(x.doubleValue() - y.doubleValue()) <= 1e-12 * Math.max(1, Math.abs(y.doubleValue()));
        if (a instanceof JsonObject x && b instanceof JsonObject y)
            return x.keySet().stream().allMatch(k -> equivalent(x.get(k), y.get(k)));
        if (a instanceof JsonArray x && b instanceof JsonArray y) {
            if (x.size() != y.size()) return false;
            for (int i = 0; i < x.size(); i++) if (!equivalent(x.get(i), y.get(i))) return false;
            return true;
        }
        return Objects.equals(a, b);
    }

    static double number(JsonObject data, String key, double min, double max) {
        JsonNumber n = data.getJsonNumber(key);
        if (n == null || !Double.isFinite(n.doubleValue()) || n.doubleValue() < min || n.doubleValue() > max)
            throw new IllegalArgumentException("Invalid flight setting " + key + " (expected " + min + ".." + max + ")");
        return n.doubleValue();
    }
}
