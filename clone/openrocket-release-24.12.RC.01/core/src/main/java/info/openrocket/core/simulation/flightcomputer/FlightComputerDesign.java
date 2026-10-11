package info.openrocket.core.simulation.flightcomputer;

import edu.mit.rocket_team.FC.FlightComputerTimingSettings;
import jakarta.json.*;
import jakarta.json.stream.JsonGenerator;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.security.*;
import java.util.*;
import java.util.function.ToDoubleFunction;

/**
 * Immutable, external FC definition. Java board source is stored as text and requires local
 * approval before execution.
 */
public final class FlightComputerDesign {
    public static final String MODEL = "zephyrus-java-1";
    private final JsonObject json;

    /**
     * Creates a FlightComputerDesign after validating the supplied configuration.
     *
     * @param json json
     * @throws IllegalArgumentException when rejected: Unsupported .fc format/version
     */
    public FlightComputerDesign(JsonObject json) {
        this.json = Objects.requireNonNull(json);
        if (!"openrocket.flight-computer".equals(json.getString("format", ""))
                || !Set.of(1, 2).contains(json.getInt("version", -1)))
            throw new IllegalArgumentException("Unsupported .fc format/version");
        if (json.getString("id", "").isBlank() || json.getString("name", "").isBlank())
            throw new IllegalArgumentException("A flight computer needs an ID and name");
        nodes();
        connections();
        rules();
        parameters(); // Require structural sections even for drafts.
    }

    /**
     * Returns the controller-block definitions stored in this design.
     *
     * @return the controller-block definitions stored in this design
     */
    public List<FlightComputerController> controllers() {
        return FlightComputerController.list(this);
    }

    /**
     * Returns the immutable JSON representation of this external FC design.
     *
     * @return the immutable JSON representation of this external FC design
     */
    public JsonObject json() {
        return json;
    }

    /**
     * Returns the stable design identifier.
     *
     * @return the stable design identifier
     */
    public String id() {
        return json.getString("id");
    }

    /**
     * Returns the user-visible design name.
     *
     * @return the user-visible design name
     */
    public String name() {
        return json.getString("name");
    }

    /**
     * Returns the selected flight-computer family identifier.
     *
     * @return the selected flight-computer family identifier
     */
    public String computer() {
        return json.getString("computer");
    }

    /**
     * Returns the runtime model identifier used to interpret this design.
     *
     * @return the runtime model identifier used to interpret this design
     */
    public String model() {
        return json.getString("model");
    }

    /**
     * Returns the hardware graph nodes in saved order.
     *
     * @return the hardware graph nodes in saved order
     */
    public JsonArray nodes() {
        return Objects.requireNonNull(json.getJsonArray("nodes"), "Missing nodes");
    }

    /**
     * Returns the hardware graph connections.
     *
     * @return the hardware graph connections
     */
    public JsonArray connections() {
        return Objects.requireNonNull(json.getJsonArray("connections"), "Missing connections");
    }

    /**
     * Returns the configured legacy transition rules.
     *
     * @return the configured legacy transition rules
     */
    public JsonArray rules() {
        return Objects.requireNonNull(json.getJsonArray("rules"), "Missing rules");
    }

    /**
     * Reports whether the file contains an explicit editable state graph.
     *
     * @return true when the condition described above holds
     */
    public boolean hasStateMachine() {
        return json.containsKey("stateMachine");
    }

    /**
     * Returns the saved state graph or the materialized firmware-compatible default graph.
     *
     * @return the saved state graph or the materialized firmware-compatible default graph
     */
    public JsonObject stateMachine() {
        return hasStateMachine()
                ? json.getJsonObject("stateMachine")
                : FlightComputerStateMachine.defaults(this);
    }

    /**
     * Returns the design's runtime parameter object.
     *
     * @return the design's runtime parameter object
     */
    public JsonObject parameters() {
        return Objects.requireNonNull(json.getJsonObject("parameters"), "Missing parameters");
    }

    /**
     * Reads a named numeric runtime parameter with its model default.
     *
     * @param key configuration or lookup key
     * @return computed numeric value
     */
    public double parameter(String key) {
        return parameters().containsKey(key)
                ? parameters().getJsonNumber(key).doubleValue()
                : FlightComputerModels.PARAMETERS.get(key).value();
    }

    /**
     * Finds a hardware graph node by stable ID.
     *
     * @param id stable identifier
     * @return the resulting JsonObject
     */
    public JsonObject node(String id) {
        return nodes().getValuesAs(JsonObject.class).stream()
                .filter(n -> id.equals(n.getString("id")))
                .findFirst()
                .orElse(null);
    }

    /**
     * Returns a design copy with the supplied JSON field replaced.
     *
     * @param key configuration or lookup key
     * @param value replacement or input value
     * @return a design copy with the supplied JSON field replaced
     */
    public FlightComputerDesign with(String key, JsonValue value) {
        return new FlightComputerDesign(Json.createObjectBuilder(json).add(key, value).build());
    }

    /**
     * Builds a JSON object copy for immutable design edits.
     *
     * @param name display name or lookup key
     * @return the resulting FlightComputerDesign
     */
    public FlightComputerDesign copy(String name) {
        return new FlightComputerDesign(
                Json.createObjectBuilder(json)
                        .add("id", UUID.randomUUID().toString())
                        .add("name", name)
                        .add("origin", id())
                        .build());
    }

    /**
     * Returns the design's validated virtual execution timing settings.
     *
     * @return the design's validated virtual execution timing settings
     */
    public FlightComputerTimingSettings timing() {
        return new FlightComputerTimingSettings(
                (int) parameter("sensorReadUs"),
                (int) parameter("extraWorkUs"),
                (int) parameter("workJitterUs"),
                (int) parameter("pwmPhaseUs"),
                (int) parameter("timingSeed"));
    }

    /**
     * Returns a design copy with replacement virtual timing settings.
     *
     * @param t t
     * @return a design copy with replacement virtual timing settings
     */
    public FlightComputerDesign withTiming(FlightComputerTimingSettings t) {
        return with(
                "parameters",
                Json.createObjectBuilder(parameters())
                        .add("sensorReadUs", t.sensorReadUs())
                        .add("extraWorkUs", t.extraWorkUs())
                        .add("workJitterUs", t.workJitterUs())
                        .add("pwmPhaseUs", t.pwmPhaseUs())
                        .add("timingSeed", t.randomSeed())
                        .build());
    }

    /**
     * Reads and validates an external .fc JSON design subject to the file-size limit.
     *
     * @param path source or destination filesystem path
     * @return the resulting FlightComputerDesign
     * @throws IOException if the requested operation cannot be completed
     */
    public static FlightComputerDesign read(Path path) throws IOException {
        if (Files.size(path) > 4_000_000) throw new IOException("FC file exceeds 4 MB");
        try (var input = Files.newInputStream(path)) {
            return read(input);
        }
    }

    /**
     * Reads and validates an external .fc JSON design subject to the file-size limit.
     *
     * @param input input snapshot for this operation
     * @return the resulting FlightComputerDesign
     * @throws IOException if the requested operation cannot be completed
     */
    public static FlightComputerDesign read(InputStream input) throws IOException {
        try (var reader = Json.createReader(input)) {
            return new FlightComputerDesign(reader.readObject());
        } catch (RuntimeException e) {
            throw new IOException("Invalid flight computer: " + e.getMessage(), e);
        }
    }

    /**
     * Loads the bundled Zephyrus baseline design as an ordinary editable .fc model.
     *
     * @return the resulting FlightComputerDesign
     */
    public static FlightComputerDesign zephyrus() {
        try (var in =
                FlightComputerDesign.class.getResourceAsStream("/flightcomputers/zephyrus.fc")) {
            if (in == null) throw new IOException("Missing Zephyrus template");
            return read(in);
        } catch (IOException e) {
            throw new IllegalStateException(e);
        }
    }

    /**
     * Serializes the design as human-readable JSON.
     *
     * @return formatted or resolved text
     */
    public String text() {
        var out = new StringWriter();
        try (var writer =
                Json.createWriterFactory(Map.of(JsonGenerator.PRETTY_PRINTING, true))
                        .createWriter(out)) {
            writer.writeObject(json);
        }
        return out + "\n";
    }

    /**
     * Writes the external design file in UTF-8.
     *
     * @param path source or destination filesystem path
     * @throws IOException if the requested operation cannot be completed
     */
    public void write(Path path) throws IOException {
        Path absolute = path.toAbsolutePath();
        Files.createDirectories(absolute.getParent());
        Path temp = Files.createTempFile(absolute.getParent(), ".fc-save-", ".tmp");
        try {
            Files.writeString(temp, text(), StandardCharsets.UTF_8);
            try {
                Files.move(
                        temp,
                        absolute,
                        StandardCopyOption.ATOMIC_MOVE,
                        StandardCopyOption.REPLACE_EXISTING);
            } catch (AtomicMoveNotSupportedException e) {
                Files.move(temp, absolute, StandardCopyOption.REPLACE_EXISTING);
            }
        } finally {
            Files.deleteIfExists(temp);
        }
    }

    /**
     * Computes the design fingerprint from canonical behavior-affecting content.
     *
     * @param bytes bytes
     * @return formatted or resolved text
     */
    public static String hash(byte[] bytes) {
        try {
            return HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(bytes));
        } catch (NoSuchAlgorithmException e) {
            throw new AssertionError(e);
        }
    }

    /**
     * Order by stable object IDs; condition order is retained. Layout/names are not runtime inputs.
     */
    public String fingerprint() {
        return hash(canonical(json).toString().getBytes(StandardCharsets.UTF_8));
    }

    /**
     * Normalizes JSON for reproducible hashing while excluding presentation-only fields.
     *
     * @param value replacement or input value
     * @return the resulting JsonValue
     */
    private static JsonValue canonical(JsonValue value) {
        if (value instanceof JsonObject object) {
            var b = Json.createObjectBuilder();
            object.keySet().stream()
                    .sorted()
                    .filter(
                            k ->
                                    !Set.of("layout", "name", "description", "origin", "notes")
                                            .contains(k))
                    .forEach(k -> b.add(k, canonical(object.get(k))));
            return b.build();
        }
        if (value instanceof JsonArray array) {
            var list = new ArrayList<JsonValue>(array);
            if (list.stream().allMatch(v -> v instanceof JsonObject o && o.containsKey("id")))
                list.sort(Comparator.comparing(v -> ((JsonObject) v).getString("id")));
            var b = Json.createArrayBuilder();
            list.forEach(v -> b.add(canonical(v)));
            return b.build();
        }
        return value;
    }

    public record Diagnostic(String item, String message, boolean error) {
        /**
         * Returns the display representation used by this value in logs or UI controls.
         *
         * @return the display representation used by this value in logs or UI controls
         */
        @Override
        public String toString() {
            return (error ? "Error: " : "Note: ") + message;
        }
    }

    /**
     * Collects actionable design validation errors without running a simulation.
     *
     * @return the resulting List&lt;Diagnostic&gt;
     * @throws IllegalArgumentException when rejected: Controller blocks require .fc version 2
     */
    public List<Diagnostic> diagnostics() {
        var issues = new ArrayList<Diagnostic>();
        try {
            if (!Set.of(MODEL, "flight-computer-java-1").contains(model()))
                issues.add(new Diagnostic("", computer() + " runtime is unavailable", true));
            for (String key : json.keySet())
                if (!Set.of(
                                "format",
                                "version",
                                "id",
                                "name",
                                "computer",
                                "model",
                                "description",
                                "origin",
                                "notes",
                                "parameters",
                                "nodes",
                                "connections",
                                "rules",
                                "layout",
                                "stateMachine",
                                "controllers",
                                "legacyInputs")
                        .contains(key))
                    issues.add(new Diagnostic("", "Unsupported design field: " + key, true));
            timing();
            for (var spec : FlightComputerModels.PARAMETERS.values()) {
                double v = parameter(spec.key());
                if (!Double.isFinite(v)
                        || v < spec.min()
                        || v > spec.max()
                        || (spec.integer() && v != Math.rint(v)))
                    issues.add(
                            new Diagnostic(
                                    "", spec.label() + " is outside its supported range", true));
            }
            for (String key : parameters().keySet())
                if (!FlightComputerModels.PARAMETERS.containsKey(key))
                    issues.add(new Diagnostic("", "Unsupported parameter: " + key, true));
            var ids = new HashSet<String>();
            for (var node : nodes().getValuesAs(JsonObject.class)) {
                String id = node.getString("id");
                String type = node.getString("type");
                if (id.isBlank() || node.getString("name").isBlank())
                    issues.add(new Diagnostic(id, "Component ID and name are required", true));
                var layout = node.getJsonObject("layout");
                for (String axis : List.of("x", "y"))
                    if (!Double.isFinite(layout.getJsonNumber(axis).doubleValue())
                            || Math.abs(layout.getJsonNumber(axis).doubleValue()) > 1e6)
                        issues.add(new Diagnostic(id, "Invalid canvas position", true));
                for (String key : node.keySet())
                    if (!Set.of(
                                    "id",
                                    "type",
                                    "name",
                                    "board",
                                    "properties",
                                    "layout",
                                    "notes",
                                    "description",
                                    "program")
                            .contains(key))
                        issues.add(new Diagnostic(id, "Unsupported component field: " + key, true));
                if (!ids.add(id))
                    issues.add(new Diagnostic(id, "Duplicate component ID " + id, true));
                if (!FlightComputerModels.TYPES.contains(type))
                    issues.add(new Diagnostic(id, "Unsupported component model " + type, true));
                String board = node.getString("board", "");
                if (!board.isEmpty()
                        && (node(board) == null
                                || !FlightComputerModels.isBoard(node(board).getString("type"))))
                    issues.add(
                            new Diagnostic(
                                    id,
                                    "Missing parent board for " + node.getString("name"),
                                    true));
                var ancestors = new HashSet<String>();
                var p = node;
                while (p != null && !p.getString("board", "").isEmpty()) {
                    String parent = p.getString("board");
                    if (!ancestors.add(parent) || parent.equals(id)) {
                        issues.add(new Diagnostic(id, "Board nesting cycle", true));
                        break;
                    }
                    p = node(parent);
                }
                for (var spec : FlightComputerModels.properties(type)) {
                    double v =
                            node.getJsonObject("properties").containsKey(spec.key())
                                    ? node.getJsonObject("properties")
                                            .getJsonNumber(spec.key())
                                            .doubleValue()
                                    : spec.value();
                    if (!Double.isFinite(v)
                            || v < spec.min()
                            || v > spec.max()
                            || (spec.integer() && v != Math.rint(v)))
                        issues.add(
                                new Diagnostic(
                                        id,
                                        node.getString("name") + ": invalid " + spec.label(),
                                        true));
                }
                for (String key : node.getJsonObject("properties").keySet())
                    if (FlightComputerModels.properties(type).stream()
                            .noneMatch(s -> s.key().equals(key)))
                        issues.add(new Diagnostic(id, "Unsupported property: " + key, true));
            }
            for (var n : nodes().getValuesAs(JsonObject.class)) {
                if (n.getString("type").equals("java_board")) {
                    if (n.getString("program", "").isBlank()
                            || n.getString("program").length() > 100000)
                        issues.add(
                                new Diagnostic(
                                        n.getString("id"),
                                        "Java board needs source code (up to 100 KB)",
                                        true));
                } else if (n.containsKey("program"))
                    issues.add(
                            new Diagnostic(
                                    n.getString("id"),
                                    "Only a Java board may contain a program",
                                    true));
            }
            if (hasStateMachine()) FlightComputerStateMachine.validate(stateMachine());
            if (json.containsKey("controllers") && json.getInt("version") != 2)
                throw new IllegalArgumentException("Controller blocks require .fc version 2");
            var controllerIds = new HashSet<String>();
            if (controllers().size() > 128)
                throw new IllegalArgumentException("At most 128 controller blocks are supported");
            for (var controller : controllers()) {
                controller.validate(this);
                if (!controllerIds.add(controller.id()) || ids.contains(controller.id()))
                    throw new IllegalArgumentException("Duplicate controller ID");
            }
            if (json.containsKey("legacyInputs"))
                for (var entry : json.getJsonObject("legacyInputs").entrySet()) {
                    if (!(entry.getValue() instanceof JsonString id)
                            || node(id.getString()) == null
                            || !node(id.getString()).getString("type").equals(entry.getKey())
                            || FlightComputerSignals.channels(entry.getKey()).isEmpty())
                        throw new IllegalArgumentException(
                                "Invalid legacy sensor selection: " + entry.getKey());
                }
            var sinks = new HashSet<String>();
            var resources = new HashSet<String>();
            var linkIds = new HashSet<String>();
            var roleSources = new HashMap<String, String>();
            for (var c : connections().getValuesAs(JsonObject.class)) {
                String id = c.getString("id"),
                        from = c.getString("from"),
                        to = c.getString("to"),
                        port = c.getString("port");
                if (!linkIds.add(id))
                    issues.add(new Diagnostic(id, "Duplicate connection ID", true));
                if (node(from) == null || node(to) == null) {
                    issues.add(new Diagnostic(id, "Connection has missing endpoint", true));
                    continue;
                }
                if (!FlightComputerModels.isControllerHost(node(to).getString("type"))
                        || !FlightComputerModels.portType(port)
                                .equals(node(from).getString("type")))
                    issues.add(
                            new Diagnostic(
                                    id, "Incompatible component / controller port " + port, true));
                boolean actuator = Set.of("roll", "airbrakes", "recovery").contains(port);
                if (!sinks.add(to + ":" + port + (actuator ? ":" + from : "")))
                    issues.add(new Diagnostic(id, "Multiple components drive " + port, true));
                String other = roleSources.putIfAbsent(port, from);
                if (!actuator
                        && other != null
                        && !other.equals(from)
                        && !(json.containsKey("legacyInputs")
                                && json.getJsonObject("legacyInputs").containsKey(port)))
                    issues.add(
                            new Diagnostic(
                                    id,
                                    "The shared controller input '"
                                            + port
                                            + "' has more than one source. Connect the same component to each host or use one source for this role.",
                                    true));
                String bus = c.getString("bus", ""), address = c.getString("address", "");
                if (!Set.of("SPI", "UART", "I2C", "GPIO", "PWM", "Power").contains(bus))
                    issues.add(new Diagnostic(id, "Unknown bus " + bus, true));
                if ((bus.equals("SPI") || bus.equals("I2C")) && address.isBlank())
                    issues.add(new Diagnostic(id, "Chip select / address is required", true));
                String resource = c.getString("resource", bus);
                if (!resources.add(to + ":" + resource + ":" + address))
                    issues.add(
                            new Diagnostic(
                                    id,
                                    "Conflicting bus address, select, or pin on " + resource,
                                    true));
            }
            for (String port : FlightComputerModels.REQUIRED_PORTS)
                if (active(port) == null && !Set.of("airbrakes", "roll", "recovery").contains(port))
                    issues.add(
                            new Diagnostic(
                                    "",
                                    "Connect a component to a processor or Java-board port: "
                                            + port,
                                    true));
            if (nodes().getValuesAs(JsonObject.class).stream()
                            .filter(n -> FlightComputerModels.isControllerHost(n.getString("type")))
                            .count()
                    > 1)
                issues.add(
                        new Diagnostic(
                                "",
                                "Processor nodes describe hardware. The built-in controller uses shared inputs/state; each custom Java board runs its own program.",
                                false));
            var ruleIds = new HashSet<String>();
            for (var rule : rules().getValuesAs(JsonObject.class)) {
                String id = rule.getString("id");
                for (String key : rule.keySet())
                    if (!Set.of("id", "condition", "notes", "description", "layout", "name")
                            .contains(key))
                        issues.add(new Diagnostic(id, "Unsupported rule field: " + key, true));
                if (!FlightComputerModels.RULES.containsKey(id) || !ruleIds.add(id))
                    issues.add(new Diagnostic(id, "Unsupported or duplicate rule " + id, true));
                validateCondition(rule.getJsonObject("condition"), 0);
            }
            for (String id : FlightComputerModels.RULES.keySet())
                if (!ruleIds.contains(id))
                    issues.add(new Diagnostic(id, "Missing rule " + id, true));
        } catch (RuntimeException e) {
            issues.add(new Diagnostic("", "Invalid design structure: " + e.getMessage(), true));
        }
        issues.add(
                new Diagnostic(
                        "",
                        "Roll drives active tab-controlled fins through an ideal 1:1 linkage; tab area and CNAlpha must be nonzero. Airbrakes drive active AirbrakeSets. Recovery deploys devices assigned in the simulation; power wiring and board geometry are descriptive.",
                        false));
        return List.copyOf(issues);
    }

    /** Rejects a design with validation errors before execution starts. */
    public void requireRunnable() {
        var errors =
                diagnostics().stream().filter(Diagnostic::error).map(Diagnostic::message).toList();
        if (!errors.isEmpty()) throw new IllegalArgumentException(String.join("; ", errors));
    }

    /**
     * Resolves the hardware node connected to a logical controller port.
     *
     * @param port logical controller port name
     * @return the resulting JsonObject
     */
    public JsonObject active(String port) {
        if (json.containsKey("legacyInputs")
                && json.getJsonObject("legacyInputs").containsKey(port))
            return node(json.getJsonObject("legacyInputs").getString(port));
        for (var c : connections().getValuesAs(JsonObject.class))
            if (c.getString("port").equals(port)) return node(c.getString("from"));
        return null;
    }

    public static final Set<String> SIGNALS =
            Set.of(
                    "command",
                    "flight_ms",
                    "apogee_ms",
                    "accel_vertical",
                    "baro_drop",
                    "gps_drop",
                    "gps_fix",
                    "baro_altitude",
                    "gps_altitude",
                    "state_ms",
                    "time_ms",
                    "velocity",
                    "roll_deg",
                    "pitch_deg",
                    "yaw_deg");

    /**
     * Checks a rule condition's operator, signal and comparison parameters.
     *
     * @param c selected channel or controller
     * @param depth depth
     * @throws IllegalArgumentException when rejected: Condition nesting exceeds 12
     */
    public static void validateCondition(JsonObject c, int depth) {
        if (depth > 12) throw new IllegalArgumentException("Condition nesting exceeds 12");
        String op = c.getString("op");
        Set<String> allowed =
                (op.equals("all") || op.equals("any"))
                        ? Set.of("op", "terms")
                        : Set.of("op", "signal", "value");
        if (!allowed.containsAll(c.keySet()))
            throw new IllegalArgumentException("Unsupported condition fields");
        if (op.equals("all") || op.equals("any")) {
            var terms = c.getJsonArray("terms");
            if (terms.isEmpty()) throw new IllegalArgumentException("Condition group is empty");
            for (var term : terms.getValuesAs(JsonObject.class)) validateCondition(term, depth + 1);
        } else {
            if (!Set.of(">", ">=", "<", "<=", "==", "!=").contains(op)
                    || !SIGNALS.contains(c.getString("signal")))
                throw new IllegalArgumentException("Unknown condition signal/operator");
            if (!Double.isFinite(c.getJsonNumber("value").doubleValue()))
                throw new IllegalArgumentException("Nonfinite condition value");
        }
    }

    /**
     * Evaluates a named transition rule against the supplied measured-signal lookup.
     *
     * @param id stable identifier
     * @param signals signals
     * @return true when the condition described above holds
     */
    public boolean test(String id, ToDoubleFunction<String> signals) {
        var rule =
                rules().getValuesAs(JsonObject.class).stream()
                        .filter(r -> r.getString("id").equals(id))
                        .findFirst()
                        .orElseThrow();
        return evaluate(rule.getJsonObject("condition"), signals);
    }

    /**
     * Evaluates a condition expression against the supplied measured-signal lookup.
     *
     * @param c selected channel or controller
     * @param signals a value
     * @return true when the condition described above holds
     */
    public static boolean evaluate(JsonObject c, ToDoubleFunction<String> signals) {
        String op = c.getString("op");
        if (op.equals("all"))
            return c.getJsonArray("terms").getValuesAs(JsonObject.class).stream()
                    .allMatch(t -> evaluate(t, signals));
        if (op.equals("any"))
            return c.getJsonArray("terms").getValuesAs(JsonObject.class).stream()
                    .anyMatch(t -> evaluate(t, signals));
        double a = signals.applyAsDouble(c.getString("signal")),
                b = c.getJsonNumber("value").doubleValue();
        return switch (op) {
            case ">" -> a > b;
            case ">=" -> a >= b;
            case "<" -> a < b;
            case "<=" -> a <= b;
            case "==" -> a == b;
            case "!=" -> a != b;
            default -> false;
        };
    }
}
