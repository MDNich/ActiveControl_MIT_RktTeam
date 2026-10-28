package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import mil.nga.mgrs.MGRS;
import mil.nga.mgrs.utm.UTM;
import java.text.ParseException;
import java.util.Locale;

/** Explicit MGRS grid-point/cell-center conversion. Saved coordinates are never re-quantized. */
public final class MissionCoordinates {
    private MissionCoordinates() { }

    public static JsonObject mgrs(String input, String reference) {
        if (!reference.equals("grid-point") && !reference.equals("cell-center"))
            throw new IllegalArgumentException("Choose grid-point or cell-center");
        String value = input.replaceAll("\\s+", "").toUpperCase(Locale.ROOT);
        if (!value.matches("(?:[1-9]|[1-5][0-9]|60)[C-HJ-NP-X][A-HJ-NP-Z]{2}(?:[0-9]{2}){0,5}"))
            throw new IllegalArgumentException("Enter a UTM MGRS square with 0-5 digits per axis; use latitude/longitude for polar locations");
        try {
            MGRS parsed = MGRS.parse(value);
            double precision = Math.pow(10, 5 - MGRS.accuracy(value));
            UTM utm = parsed.toUTM();
            if (reference.equals("cell-center"))
                utm = new UTM(utm.getZone(), utm.getHemisphere(), utm.getEasting() + precision / 2,
                        utm.getNorthing() + precision / 2);
            var point = utm.toPoint().toDegrees();
            return Json.createObjectBuilder().add("latitudeDeg", point.getLatitude())
                    .add("longitudeDeg", point.getLongitude())
                    .add("coordinateInput", Json.createObjectBuilder().add("format", "mgrs").add("value", value)
                            .add("reference", reference).add("precisionM", precision).add("source", "nga-mgrs-2.1.3"))
                    .build();
        } catch (ParseException | RuntimeException e) { throw new IllegalArgumentException("Invalid MGRS coordinate: " + value, e); }
    }

    /** Reapplying the same saved notation preserves its exact point and its original convention. */
    public static JsonObject reapply(JsonObject savedSite, String value) {
        JsonObject original = savedSite.getJsonObject("coordinateInput");
        if (original != null && original.getString("format", "").equals("mgrs")
                && original.getString("value", "").replaceAll("\\s+", "").equalsIgnoreCase(value.replaceAll("\\s+", "")))
            return savedSite;
        var converted = mgrs(value, "grid-point");
        return Json.createObjectBuilder(savedSite).add("latitudeDeg", converted.get("latitudeDeg"))
                .add("longitudeDeg", converted.get("longitudeDeg")).add("coordinateInput", converted.get("coordinateInput")).build();
    }
}
