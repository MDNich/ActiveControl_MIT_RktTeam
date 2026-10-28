package info.openrocket.core.simulation.mission;

import jakarta.json.*;

/** Deliberate import of the non-executable design preview. Keeps every original field for review. */
public final class MissionPreviewImport {
    private MissionPreviewImport() { }

    public static MissionDefinition migrate(JsonObject source) {
        if (!source.getString("format", "").equals("openrocket.mission-configuration-preview") || source.getInt("version", -1) != 1)
            throw new IllegalArgumentException("Expected a version 1 configuration preview");
        var draft = MissionDefinition.createUrrg(source.getString("title", "Imported preview")).json();
        var issues = Json.createArrayBuilder();
        var sites = Json.createArrayBuilder();
        for (var entry : draft.getJsonArray("sites")) {
            JsonObject site = entry.asJsonObject();
            JsonObject old = source.getJsonArray("sites").getValuesAs(JsonObject.class).stream()
                    .filter(s -> s.getString("id").equals(site.getString("id"))).findFirst().orElse(null);
            if (old == null) { sites.add(site); issue(issues, "/sites/" + site.getString("id"), "Missing preview location; URRG draft value retained for review"); continue; }
            var b = Json.createObjectBuilder(site).add("name", old.getString("name", site.getString("name")))
                    .add("latitudeDeg", old.get("lat")).add("longitudeDeg", old.get("lon"))
                    .add("heightM", old.getOrDefault("alt", JsonValue.NULL)).add("heightDatum", old.getString("datum", "MSL"));
            if (old.containsKey("coordinateInput")) b.add("coordinateInput", old.get("coordinateInput"));
            else b.add("coordinateInput", Json.createObjectBuilder().add("format", "latlon").add("reference", "point").add("source", "preview-import"));
            sites.add(b);
            if (old.isNull("alt")) issue(issues, "/sites/" + site.getString("id") + "/heightM", "Enter station/launch height and confirm its datum");
        }
        var pointers = Json.createArrayBuilder();
        for (int i = 0; i < draft.getJsonArray("pointers").size(); i++)
            pointers.add(Json.createObjectBuilder(draft.getJsonArray("pointers").getJsonObject(i))
                    .add("enabled", i < source.getInt("pointerCount", 4)));
        issue(issues, "/flight/rocket", "Select the actual rocket; the preview saved only a filename, not its contents");
        if (source.containsKey("flight")) for (String field : source.getJsonObject("flight").keySet())
            issue(issues, "/flight/" + field, "Review against the selected flight; preview values and text placeholders are retained in migration.source");
        for (String group : new String[] {"telemetry", "video", "connections", "run"})
            if (source.containsKey(group)) for (String field : source.getJsonObject(group).keySet())
                issue(issues, "/" + group + "/" + field, "Review and apply explicitly; preview defaults do not configure production execution");
        return new MissionDefinition(Json.createObjectBuilder(draft).add("sites", sites).add("pointers", pointers)
                .add("migration", Json.createObjectBuilder().add("reviewed", false).add("source", source).add("issues", issues)).build());
    }

    private static void issue(JsonArrayBuilder issues, String path, String message) {
        issues.add(Json.createObjectBuilder().add("path", path).add("code", "preview-review-required")
                .add("severity", "error").add("message", message));
    }
}
