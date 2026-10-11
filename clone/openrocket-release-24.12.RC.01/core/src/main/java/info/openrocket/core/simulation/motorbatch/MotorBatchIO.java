package info.openrocket.core.simulation.motorbatch;

import jakarta.json.*;
import java.io.*;
import java.util.*;
import info.openrocket.core.motor.*;
import info.openrocket.core.util.Coordinate;
import static info.openrocket.core.simulation.motorbatch.MotorBatchReport.*;

/**
 * Explicit versioned data encoding. No Java object deserialization or executable FC definitions.
 */
public final class MotorBatchIO {
    /** Prevents instantiation of this utility class. */
    private MotorBatchIO() {}

    /**
     * Converts a numeric report field between JSON and Java while retaining missing-value
     * semantics.
     *
     * @param value replacement or input value
     * @return the resulting JsonValue
     * @throws IllegalArgumentException when rejected: Invalid number
     */
    static JsonValue number(double value) {
        if (Double.isNaN(value)) return JsonValue.NULL;
        if (value == Double.POSITIVE_INFINITY) return Json.createValue("plugged");
        if (!Double.isFinite(value)) throw new IllegalArgumentException("Invalid number");
        return Json.createValue(value);
    }

    /**
     * Converts a numeric report field between JSON and Java while retaining missing-value
     * semantics.
     *
     * @param value replacement or input value
     * @return computed numeric value
     */
    static double number(JsonValue value) {
        if (value == null || value == JsonValue.NULL) return Double.NaN;
        if (value instanceof JsonString s && s.getString().equals("plugged"))
            return Double.POSITIVE_INFINITY;
        return ((JsonNumber) value).doubleValue();
    }

    /**
     * Converts a collection of report strings between JSON and Java.
     *
     * @param strings strings
     * @return the resulting JsonArray
     */
    static JsonArray strings(Collection<String> strings) {
        var a = Json.createArrayBuilder();
        strings.forEach(a::add);
        return a.build();
    }

    /**
     * Converts a collection of report strings between JSON and Java.
     *
     * @param a a
     * @return the resulting List&lt;String&gt;
     */
    static List<String> strings(JsonArray a) {
        return a.getValuesAs(JsonString.class).stream().map(JsonString::getString).toList();
    }

    /**
     * Converts batch search settings between JSON and Java.
     *
     * @param s s
     * @return the resulting JsonObject
     */
    public static JsonObject settings(MotorBatchSettings s) {
        var bounds = Json.createObjectBuilder();
        s.criteria()
                .forEach(
                        (metric, b) ->
                                bounds.add(
                                        metric.name(),
                                        Json.createArrayBuilder()
                                                .add(
                                                        b.minimum() == null
                                                                ? JsonValue.NULL
                                                                : number(b.minimum()))
                                                .add(
                                                        b.maximum() == null
                                                                ? JsonValue.NULL
                                                                : number(b.maximum()))));
        return Json.createObjectBuilder()
                .add("mount", s.mountId().toString())
                .add("threads", s.threads())
                .add("diameter", s.nominalDiameter())
                .add("length", s.maximumLength())
                .add("manufacturers", strings(new TreeSet<>(s.manufacturers())))
                .add("search", s.search())
                .add("motorType", s.motorType())
                .add("certificationLevel", s.certificationLevel())
                .add("hideUnavailable", s.hideUnavailable())
                .add("ignoreComponentIntersections", s.ignoreComponentIntersections())
                .add("minImpulse", s.minImpulse())
                .add("maxImpulse", s.maxImpulse())
                .add("minThrust", s.minThrust())
                .add("maxThrust", s.maxThrust())
                .add("minBurn", s.minBurn())
                .add("maxBurn", s.maxBurn())
                .add("delayPolicy", s.delayPolicy().name())
                .add("criteria", bounds)
                .build();
    }

    /**
     * Converts batch search settings between JSON and Java.
     *
     * @param s s
     * @return the resulting MotorBatchSettings
     */
    public static MotorBatchSettings settings(JsonObject s) {
        var bounds =
                new EnumMap<MotorBatchMetrics.Metric, MotorBatchSettings.Bounds>(
                        MotorBatchMetrics.Metric.class);
        s.getJsonObject("criteria")
                .forEach(
                        (key, value) -> {
                            var a = (JsonArray) value;
                            bounds.put(
                                    MotorBatchMetrics.Metric.valueOf(key),
                                    new MotorBatchSettings.Bounds(
                                            a.isNull(0) ? null : number(a.get(0)),
                                            a.isNull(1) ? null : number(a.get(1))));
                        });
        return new MotorBatchSettings(
                UUID.fromString(s.getString("mount")),
                s.getInt("threads"),
                number(s.get("diameter")),
                number(s.get("length")),
                new HashSet<>(strings(s.getJsonArray("manufacturers"))),
                s.getString("search"),
                s.getString("motorType"),
                s.getString("certificationLevel", "ALL"),
                s.getBoolean("hideUnavailable"),
                number(s.get("minImpulse")),
                number(s.get("maxImpulse")),
                number(s.get("minThrust")),
                number(s.get("maxThrust")),
                number(s.get("minBurn")),
                number(s.get("maxBurn")),
                MotorBatchSettings.DelayPolicy.valueOf(s.getString("delayPolicy")),
                bounds,
                s.getBoolean("ignoreComponentIntersections", false));
    }

    /**
     * Converts a selected motor curve and its identifying metadata between JSON and Java.
     *
     * @param m m
     * @return the resulting JsonObject
     */
    public static JsonObject motor(ThrustCurveMotor m) {
        var time = Json.createArrayBuilder();
        for (double x : m.getTimePoints()) time.add(x);
        var thrust = Json.createArrayBuilder();
        for (double x : m.getThrustPoints()) thrust.add(x);
        var delays = Json.createArrayBuilder();
        for (double x : m.getStandardDelays()) delays.add(number(x));
        var cg = Json.createArrayBuilder();
        for (Coordinate c : m.getCGPoints())
            cg.add(Json.createArrayBuilder().add(c.x).add(c.y).add(c.z).add(c.weight));
        return Json.createObjectBuilder()
                .add("manufacturer", m.getManufacturer().getDisplayName())
                .add("designation", m.getDesignation())
                .add("commonName", Objects.toString(m.getCommonName(), ""))
                .add("code", Objects.toString(m.getCode(), ""))
                .add("digest", Objects.toString(m.getDigest(), ""))
                .add("description", Objects.toString(m.getDescription(), ""))
                .add("diameter", m.getDiameter())
                .add("length", m.getLength())
                .add("initialMass", number(m.getInitialMass()))
                .add("type", m.getMotorType().name())
                .add("available", m.isAvailable())
                .add("case", Objects.toString(m.getCaseInfo(), ""))
                .add("propellant", Objects.toString(m.getPropellantInfo(), ""))
                .add("time", time)
                .add("thrust", thrust)
                .add("cg", cg)
                .add("delays", delays)
                .build();
    }

    /**
     * Converts a selected motor curve and its identifying metadata between JSON and Java.
     *
     * @param j time value
     * @return the resulting ThrustCurveMotor
     * @throws IllegalArgumentException when rejected: Invalid saved motor curve size
     */
    public static ThrustCurveMotor motor(JsonObject j) {
        var time = j.getJsonArray("time");
        var thrust = j.getJsonArray("thrust");
        var cg = j.getJsonArray("cg");
        if (time.size() < 2
                || time.size() > 100000
                || thrust.size() != time.size()
                || cg.size() != time.size())
            throw new IllegalArgumentException("Invalid saved motor curve size");
        double[] times = time.stream().mapToDouble(MotorBatchIO::number).toArray(),
                forces = thrust.stream().mapToDouble(MotorBatchIO::number).toArray();
        Coordinate[] coords = new Coordinate[cg.size()];
        for (int i = 0; i < coords.length; i++) {
            var c = cg.getJsonArray(i);
            coords[i] =
                    new Coordinate(
                            number(c.get(0)), number(c.get(1)), number(c.get(2)), number(c.get(3)));
        }
        return new ThrustCurveMotor.Builder()
                .setManufacturer(Manufacturer.getManufacturer(j.getString("manufacturer")))
                .setDesignation(j.getString("designation"))
                .setCommonName(j.getString("commonName"))
                .setCode(j.getString("code"))
                .setDigest(j.getString("digest"))
                .setDescription(j.getString("description"))
                .setDiameter(number(j.get("diameter")))
                .setLength(number(j.get("length")))
                .setInitialMass(
                        j.containsKey("initialMass")
                                ? number(j.get("initialMass"))
                                : coords[0].weight)
                .setMotorType(Motor.Type.valueOf(j.getString("type")))
                .setAvailability(j.getBoolean("available"))
                .setCaseInfo(j.getString("case"))
                .setPropellantInfo(j.getString("propellant"))
                .setTimePoints(times)
                .setThrustPoints(forces)
                .setCGPoints(coords)
                .setStandardDelays(
                        j.getJsonArray("delays").stream()
                                .mapToDouble(MotorBatchIO::number)
                                .toArray())
                .build();
    }

    /**
     * Converts a complete trial result, including failures and criteria, between JSON and Java.
     *
     * @param t t
     * @return the resulting JsonObject
     */
    private static JsonObject trial(Trial t) {
        var metrics = Json.createObjectBuilder();
        t.metrics().forEach((k, v) -> metrics.add(k.name(), number(v)));
        return Json.createObjectBuilder()
                .add("number", t.number())
                .add("delay", number(t.delay()))
                .add("status", t.status().name())
                .add("assessment", t.assessment().name())
                .add("reasons", strings(t.reasons()))
                .add("metrics", metrics)
                .add("warnings", t.warnings())
                .add("error", t.error())
                .add("csv", t.telemetryPath())
                .add("log", t.logPath())
                .add("deploymentRank", number(t.deploymentRank()))
                .add("apogeeRank", number(t.apogeeRank()))
                .build();
    }

    /**
     * Converts a complete trial result, including failures and criteria, between JSON and Java.
     *
     * @param j j
     * @return the resulting Trial
     */
    private static Trial trial(JsonObject j) {
        var metrics = new EnumMap<MotorBatchMetrics.Metric, Double>(MotorBatchMetrics.Metric.class);
        j.getJsonObject("metrics")
                .forEach((k, v) -> metrics.put(MotorBatchMetrics.Metric.valueOf(k), number(v)));
        return new Trial(
                j.getInt("number"),
                number(j.get("delay")),
                Status.valueOf(j.getString("status")),
                Assessment.valueOf(j.getString("assessment")),
                strings(j.getJsonArray("reasons")),
                metrics,
                j.getString("warnings"),
                j.getString("error"),
                j.getString("csv"),
                j.getString("log"),
                number(j.get("deploymentRank")),
                number(j.get("apogeeRank")));
    }

    /**
     * Serializes a motor-batch report for persistence.
     *
     * @param r r
     * @return formatted or resolved text
     */
    public static String write(MotorBatchReport r) {
        var rows = Json.createArrayBuilder();
        for (var row : r.rows()) {
            var trials = Json.createArrayBuilder();
            row.trials().forEach(t -> trials.add(trial(t)));
            rows.add(
                    Json.createObjectBuilder()
                            .add("id", row.candidate().id())
                            .add("motor", motor(row.candidate().motor()))
                            .add(
                                    "ignoredIntersections",
                                    strings(row.candidate().ignoredIntersections()))
                            .add("fitWarnings", strings(row.candidate().fitWarnings()))
                            .add("exclusion", row.candidate().exclusion())
                            .add("planned", row.planned())
                            .add("trials", trials)
                            .add("selected", trial(row.selected())));
        }
        return Json.createObjectBuilder()
                .add("version", 1)
                .add("id", r.id())
                .add("created", r.created())
                .add("settings", settings(r.settings()))
                .add("input", r.input())
                .add("rows", rows)
                .add("peakThreads", r.peakThreads())
                .build()
                .toString();
    }

    /**
     * Reconstructs a persisted motor-batch report.
     *
     * @param text text
     * @return the resulting MotorBatchReport
     * @throws IllegalArgumentException when rejected: Motor batch record exceeds 64 MiB
     */
    public static MotorBatchReport read(String text) {
        if (text.length() > 64 * 1024 * 1024)
            throw new IllegalArgumentException("Motor batch record exceeds 64 MiB");
        JsonObject j;
        try (var reader = Json.createReader(new StringReader(text))) {
            j = reader.readObject();
        }
        if (j.getInt("version") != 1)
            throw new IllegalArgumentException("Unsupported motor batch version");
        if (j.getJsonArray("rows").size() > 100000)
            throw new IllegalArgumentException("Invalid batch row count");
        UUID.fromString(j.getString("id"));
        java.time.Instant.parse(j.getString("created"));
        var settings = settings(j.getJsonObject("settings"));
        if (j.getInt("peakThreads") < 0 || j.getInt("peakThreads") > settings.threads())
            throw new IllegalArgumentException("Invalid batch worker count");
        var input = j.getJsonObject("input");
        for (String key :
                List.of(
                        "baseline",
                        "configuration",
                        "version",
                        "fcReference",
                        "fcFingerprint",
                        "targetName",
                        "xml",
                        "hash")) input.getString(key);
        input.getInt("seed");
        input.getInt("fcSeed");
        input.getInt("fcTimingSeed");
        input.getBoolean("fcEnabled");
        if (input.getInt("motorCount") < 1 || input.getJsonArray("motors").size() > 100000)
            throw new IllegalArgumentException("Invalid baseline motor count");
        String xml = input.getString("xml");
        if (xml.length() > 32 * 1024 * 1024 || xml.contains("<motorbatch"))
            throw new IllegalArgumentException("Invalid nested batch input");
        String hash =
                info.openrocket.core.simulation.flightcomputer.FlightComputerDesign.hash(
                        (xml + input.getString("fcFingerprint"))
                                .getBytes(java.nio.charset.StandardCharsets.UTF_8));
        if (!hash.equals(input.getString("hash")))
            throw new IllegalArgumentException("Batch input hash does not match");
        var rows = new ArrayList<Row>();
        var ids = new HashSet<String>();
        for (var v : j.getJsonArray("rows").getValuesAs(JsonObject.class)) {
            if (!ids.add(v.getString("id")))
                throw new IllegalArgumentException("Duplicate batch candidate ID");
            if (v.getInt("planned") < 0
                    || v.getInt("planned") > 1000
                    || v.getJsonArray("trials").size() > 1000)
                throw new IllegalArgumentException("Invalid delay trial count");
            var candidate =
                    new MotorCandidateSelector.Candidate(
                            v.getString("id"),
                            motor(v.getJsonObject("motor")),
                            v.getString("exclusion"),
                            v.containsKey("ignoredIntersections")
                                    ? strings(v.getJsonArray("ignoredIntersections"))
                                    : List.of(),
                            v.containsKey("fitWarnings")
                                    ? strings(v.getJsonArray("fitWarnings"))
                                    : List.of());
            rows.add(
                    new Row(
                            candidate,
                            v.getJsonArray("trials").getValuesAs(JsonObject.class).stream()
                                    .map(MotorBatchIO::trial)
                                    .toList(),
                            v.getInt("planned"),
                            trial(v.getJsonObject("selected"))));
        }
        return new MotorBatchReport(
                j.getString("id"),
                j.getString("created"),
                settings,
                input,
                rows,
                j.getInt("peakThreads"));
    }
}
