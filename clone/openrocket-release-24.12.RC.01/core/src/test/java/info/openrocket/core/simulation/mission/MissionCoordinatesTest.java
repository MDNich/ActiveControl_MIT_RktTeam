package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class MissionCoordinatesTest {
    @Test void allUrrgMgrsPointsAgreeWithinThreeCentimetres() {
        for (var value : MissionJson.resource("urrg-sites.json").getJsonArray("sites")) {
            var site = value.asJsonObject();
            var point = MissionCoordinates.mgrs(site.getString("mgrs"), "grid-point");
            assertEquals(site.getJsonNumber("latitudeDeg").doubleValue(), point.getJsonNumber("latitudeDeg").doubleValue(), 3e-7);
            assertEquals(site.getJsonNumber("longitudeDeg").doubleValue(), point.getJsonNumber("longitudeDeg").doubleValue(), 3e-7);
        }
    }

    @Test void reapplyingDoesNotQuantizeSavedCellCenterOrSurvey() {
        var center = MissionCoordinates.mgrs("18TUN2061530290", "cell-center");
        var saved = Json.createObjectBuilder(center).add("latitudeDeg", 42.704171).build();
        assertSame(saved, MissionCoordinates.reapply(saved, "18t un 20615 30290"));
        var point = MissionCoordinates.mgrs("18TUN2061530290", "grid-point");
        assertNotEquals(center.get("latitudeDeg"), point.get("latitudeDeg"));
        assertThrows(IllegalArgumentException.class, () -> MissionCoordinates.mgrs("18TUN1", "grid-point"));
        assertThrows(IllegalArgumentException.class, () -> MissionCoordinates.mgrs("91TUN12341234", "grid-point"));
    }

    @Test void geoidMatchesIndependentKarneyBilinearVectorsIncludingSeamAndPoles() {
        // PyGeodesy GeoidKarney(kind=2), independently evaluated against the published PGM.
        double[][] cases = {{42.70416776971639, -77.19029501648315, -34.56785885502213},
                {0, 0, 17.163}, {90, 0, 13.605}, {-90, 0, -29.535},
                {10, -179.9, 12.5988}, {-30, 179.9, 47.3766}, {27.9881, 86.925, -28.86631248}};
        var geoid = MissionGeoid.bundled();
        for (var c : cases) assertEquals(c[2], geoid.separation(c[0], c[1]), 1e-9);
        assertEquals(geoid.separation(42, -180), geoid.separation(42, 180), 1e-12);
        assertThrows(IllegalArgumentException.class, () -> geoid.separation(Double.NaN, 0));
        assertEquals("egm96-15.bilinear.v1", geoid.provenance().getString("id"));
    }

    @Test void previewImportPreservesOriginalAndReportsPlaceholdersWithoutEnablingThem() {
        var sites = Json.createArrayBuilder();
        for (var value : MissionDefinition.createUrrg("x").json().getJsonArray("sites")) {
            var site = value.asJsonObject();
            sites.add(Json.createObjectBuilder().add("id", site.getString("id")).add("lat", site.get("latitudeDeg"))
                    .add("lon", site.get("longitudeDeg")).addNull("alt").add("datum", "MSL")
                    .add("coordinateInput", site.get("coordinateInput")));
        }
        var source = Json.createObjectBuilder().add("format", "openrocket.mission-configuration-preview").add("version", 1)
                .add("title", "Old draft").add("sites", sites).add("pointerCount", 2)
                .add("flight", Json.createObjectBuilder().add("motors", "Use selected flight configuration"))
                .add("connections", Json.createObjectBuilder().add("path", "/old/machine/path")).build();
        var result = MissionPreviewImport.migrate(source).json();
        assertEquals(source, result.getJsonObject("migration").getJsonObject("source"));
        assertFalse(result.getJsonObject("migration").getBoolean("reviewed"));
        assertFalse(result.getJsonArray("pointers").getJsonObject(2).getBoolean("enabled"));
        assertTrue(result.getJsonObject("flight").isEmpty());
        assertEquals(9, result.getJsonObject("migration").getJsonArray("issues").size());
    }
}
