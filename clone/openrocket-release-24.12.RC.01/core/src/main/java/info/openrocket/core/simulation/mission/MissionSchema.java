package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.util.*;
import java.util.regex.Pattern;

/** Validates the deliberately bounded subset used by the bundled contracts, with no remote refs. */
public final class MissionSchema {
    public record Issue(String path, String code, String message) { }

    private static final Set<String> KEYWORDS = Set.of("$schema", "$id", "title", "description",
            "type", "properties", "required", "items", "const", "enum", "minimum", "maximum",
            "minItems", "maxItems", "minLength", "maxLength", "pattern");

    private MissionSchema() { }

    public static List<Issue> validate(JsonObject schema, JsonValue value) {
        checkSchema(schema);
        var issues = new ArrayList<Issue>();
        validate(schema, value, "", issues);
        return List.copyOf(issues);
    }

    private static void checkSchema(JsonObject schema) {
        for (String key : schema.keySet()) {
            if (!KEYWORDS.contains(key)) throw new IllegalArgumentException("Unsupported schema keyword: " + key);
        }
        if (schema.containsKey("properties"))
            schema.getJsonObject("properties").values().forEach(v -> checkSchema(v.asJsonObject()));
        if (schema.containsKey("items")) checkSchema(schema.getJsonObject("items"));
    }

    private static String escape(String key) { return key.replace("~", "~0").replace("/", "~1"); }

    private static void fail(List<Issue> out, String path, String code, String message) {
        out.add(new Issue(path, code, message));
    }

    private static boolean type(JsonValue value, String type) {
        return switch (type) {
            case "object" -> value instanceof JsonObject;
            case "array" -> value instanceof JsonArray;
            case "string" -> value instanceof JsonString;
            case "number" -> value instanceof JsonNumber;
            case "integer" -> value instanceof JsonNumber n && n.isIntegral();
            case "boolean" -> value == JsonValue.TRUE || value == JsonValue.FALSE;
            case "null" -> value == JsonValue.NULL;
            default -> throw new IllegalArgumentException("Unsupported schema type: " + type);
        };
    }

    private static boolean equal(JsonValue a, JsonValue b) {
        if (a instanceof JsonNumber x && b instanceof JsonNumber y)
            return x.bigDecimalValue().compareTo(y.bigDecimalValue()) == 0;
        return a.equals(b);
    }

    private static void validate(JsonObject s, JsonValue v, String path, List<Issue> out) {
        JsonValue types = s.get("type");
        if (types != null) {
            boolean ok = types instanceof JsonString t ? type(v, t.getString())
                    : types.asJsonArray().stream().anyMatch(t -> type(v, ((JsonString) t).getString()));
            if (!ok) { fail(out, path, "type", "Expected " + types); return; }
        }
        if (s.containsKey("const") && !equal(v, s.get("const")))
            fail(out, path, "const", "Unsupported contract value");
        if (s.containsKey("enum") && s.getJsonArray("enum").stream().noneMatch(e -> equal(v, e)))
            fail(out, path, "enum", "Unsupported choice");
        if (v instanceof JsonObject object) {
            for (var required : s.getJsonArray("required") == null ? JsonValue.EMPTY_JSON_ARRAY : s.getJsonArray("required")) {
                String key = ((JsonString) required).getString();
                if (!object.containsKey(key)) fail(out, path + "/" + escape(key), "required", "Missing field");
            }
            JsonObject properties = s.getJsonObject("properties");
            if (properties != null) for (String key : properties.keySet()) {
                if (object.containsKey(key)) validate(properties.getJsonObject(key), object.get(key), path + "/" + escape(key), out);
            }
        } else if (v instanceof JsonArray array) {
            if (array.size() < s.getInt("minItems", 0) || array.size() > s.getInt("maxItems", Integer.MAX_VALUE))
                fail(out, path, "items", "Array length is out of bounds");
            if (s.containsKey("items")) for (int i = 0; i < array.size(); i++)
                validate(s.getJsonObject("items"), array.get(i), path + "/" + i, out);
        } else if (v instanceof JsonNumber number) {
            if (!Double.isFinite(number.doubleValue())) fail(out, path, "finite", "A finite number is required");
            if (s.containsKey("minimum") && number.bigDecimalValue().compareTo(s.getJsonNumber("minimum").bigDecimalValue()) < 0)
                fail(out, path, "minimum", "Below minimum");
            if (s.containsKey("maximum") && number.bigDecimalValue().compareTo(s.getJsonNumber("maximum").bigDecimalValue()) > 0)
                fail(out, path, "maximum", "Above maximum");
        } else if (v instanceof JsonString string) {
            String text = string.getString();
            int size = text.codePointCount(0, text.length());
            if (size < s.getInt("minLength", 0) || size > s.getInt("maxLength", Integer.MAX_VALUE))
                fail(out, path, "length", "String length is out of bounds");
            if (s.containsKey("pattern") && !Pattern.compile(s.getString("pattern")).matcher(text).find())
                fail(out, path, "pattern", "Invalid field syntax");
        }
    }
}
