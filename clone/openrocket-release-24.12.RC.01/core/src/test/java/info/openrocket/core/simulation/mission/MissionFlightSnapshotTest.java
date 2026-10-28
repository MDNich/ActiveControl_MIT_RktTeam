package info.openrocket.core.simulation.mission;

import edu.mit.rocket_team.RTFCVerificationRocket;
import info.openrocket.core.models.wind.*;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.ensemble.EnsembleSettings;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.simulation.flightcomputer.FlightComputerDesign;
import info.openrocket.core.simulation.flightcomputer.FlightComputerLibrary;
import info.openrocket.core.util.BaseTestCase;
import jakarta.json.*;
import java.lang.reflect.Modifier;
import java.nio.file.*;
import java.util.*;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import static org.junit.jupiter.api.Assertions.*;

class MissionFlightSnapshotTest extends BaseTestCase {
    @TempDir Path temp;
    String previousLibrary;
    @BeforeEach void useTemporaryLibrary() {
        previousLibrary = System.getProperty("openrocket.fc.library");
        System.setProperty("openrocket.fc.library", temp.resolve("library").toString());
    }
    @AfterEach void restoreLibrary() {
        if (previousLibrary == null) System.clearProperty("openrocket.fc.library");
        else System.setProperty("openrocket.fc.library", previousLibrary);
    }

    private SimulationOptions unusualOptions() {
        var o = new SimulationOptions();
        o.setLaunchIntoWind(false); o.setLaunchRodDirection(.123); o.setLaunchRodLength(7.5);
        o.setLaunchRodAngle(.02); o.setInitialRollRate(1.7);
        o.setLaunchLatitude(42.70416776971639); o.setLaunchLongitude(-77.19029501648315); o.setLaunchAltitude(123);
        o.setISAAtmosphere(false); o.setLaunchTemperature(300.2); o.setLaunchPressure(98765.4);
        o.setTimeStep(.031); o.setMaxSimulationTime(42); o.setMaximumStepAngle(.0321); o.setRandomSeed(-123456);
        o.getAverageWindModel().setAverage(3.21); o.getAverageWindModel().setDirection(.456); o.getAverageWindModel().setStandardDeviation(.789);
        o.getMultiLevelWindModel().clearLevels();
        o.getMultiLevelWindModel().addWindLevel(0, 4, .6, .3);
        o.getMultiLevelWindModel().addWindLevel(1234, 9, .9, .8);
        o.getMultiLevelWindModel().setAltitudeReference(WindModel.AltitudeReference.AGL);
        o.setWindModelType(WindModelType.MULTI_LEVEL); o.setLaunchIntoWind(true);
        o.setEnsembleSettings(new EnsembleSettings(true, 7, 1.5, .031, 2.5, .02, 3.5, 9876, .04, .05, .06, .07, .08, 3));
        return o;
    }

    @Test
    void allPrimitiveFlightFieldsHaveExplicitInventoryCoverage() {
        var fields = new TreeSet<String>();
        for (var f : SimulationOptions.class.getDeclaredFields())
            if (!Modifier.isStatic(f.getModifiers()) && f.getType().isPrimitive()) fields.add(f.getName());
        assertEquals(Set.of("launchRodLength", "launchIntoWind", "launchRodAngle", "launchRodDirection", "initialRollRate",
                "launchAltitude", "launchLatitude", "launchLongitude", "useISA", "launchTemperature", "launchPressure",
                "timeStep", "maxSimulationTime", "maximumAngle", "randomSeed"), fields,
                "Update the mission option adapter and round-trip fixture when adding flight fields");
    }

    @Test
    void everySettingRoundTripsIncludingInactiveWindAndManualBearing() {
        var original = unusualOptions();
        var snapshot = MissionFlightOptions.capture(original);
        assertEquals(.123, original.getLaunchRodDirectionSetting());
        var restored = MissionFlightOptions.restore(snapshot);
        assertEquals(snapshot, MissionFlightOptions.capture(restored));
        assertEquals(original.getEnsembleSettings(), restored.getEnsembleSettings());
        restored.setLaunchIntoWind(false);
        assertEquals(.123, restored.getLaunchRodDirection());
        restored.getMultiLevelWindModel().clearLevels();
        assertEquals(2, original.getMultiLevelWindModel().getLevels().size());
    }

    @Test
    void invalidSettingsCannotPartiallyModifySource() {
        var o = unusualOptions(); var before = MissionFlightOptions.capture(o);
        assertThrows(IllegalArgumentException.class, () -> MissionFlightOptions.restore(Json.createObjectBuilder(before).add("timeStepS", 0).build()));
        assertEquals(before, MissionFlightOptions.capture(o));
    }

    @Test
    void nativeSnapshotPreservesMotorsComponentsRecoveryExtensionsAndOptions() throws Exception {
        var source = RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());
        source.copySimulationOptionsFrom(unusualOptions()); source.getOptions().setRandomSeed(-123456);
        var fcPath = temp.resolve("library/input.fc"); FlightComputerDesign.zephyrus().write(fcPath);
        var fc = new ZephyrusFlightComputer(); fc.setDesignReference(FlightComputerLibrary.reference(fcPath));
        source.getSimulationExtensions().add(fc);
        var before = MissionFlightOptions.capture(source.getOptions());
        var motors = MissionMotorSnapshot.capture(source.getRocket());
        var folder = temp.resolve("frozen");
        MissionFlightSnapshot.capture(source, folder);
        assertEquals(before, MissionFlightOptions.capture(source.getOptions()));
        Files.delete(fcPath); // Restoration must use frozen bytes, not the original path/library.
        var restored = MissionFlightSnapshot.restore(folder);
        assertEquals(before, MissionFlightOptions.capture(restored.getOptions()));
        assertEquals(source.getId(), restored.getId());
        assertEquals(motors, MissionMotorSnapshot.capture(restored.getRocket()));
        var originalIds = new ArrayList<String>(); var restoredIds = new ArrayList<String>();
        for (var component : source.getRocket()) originalIds.add(component.getID().toString());
        for (var component : restored.getRocket()) restoredIds.add(component.getID().toString());
        assertEquals(originalIds, restoredIds);
        var restoredFc = (ZephyrusFlightComputer) restored.getSimulationExtensions().get(0);
        assertTrue(Files.isRegularFile(restoredFc.getMissionDesign().path()));
        assertTrue(restoredFc.getMissionDesign().provenance().get("reference").startsWith("mission:sha256/"));
        assertEquals(fc.getConfig().keySet(), restoredFc.getConfig().keySet());
        for (String key : fc.getConfig().keySet()) if (!key.equals("designFile"))
            assertEquals(fc.getConfig().get(key, null), restoredFc.getConfig().get(key, null), key);
    }

    @Test
    void restoredSnapshotProducesTheSameOrdinaryFlight() throws Exception {
        var source = RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());
        source.getOptions().setTimeStep(.05);
        var folder = temp.resolve("flight-parity");
        MissionFlightSnapshot.capture(source, folder);
        var restored = MissionFlightSnapshot.restore(folder);
        source.simulate(); restored.simulate();
        var expected = source.getSimulatedData(); var actual = restored.getSimulatedData();
        assertEquals(expected.getBranchCount(), actual.getBranchCount());
        assertEquals(expected.getMaxAltitude(), actual.getMaxAltitude(), 1e-6);
        for (int branch = 0; branch < expected.getBranchCount(); branch++) {
            for (var type : List.of(FlightDataType.TYPE_TIME, FlightDataType.TYPE_ALTITUDE, FlightDataType.TYPE_VELOCITY_Z)) {
                var a = expected.getBranch(branch).get(type); var b = actual.getBranch(branch).get(type);
                assertEquals(a.size(), b.size(), type.toString());
                for (int i = 0; i < a.size(); i++) assertEquals(a.get(i), b.get(i), 1e-5, type + " sample " + i);
            }
        }
    }

    @Test
    void changedFrozenAssetIsRejected() throws Exception {
        var source = RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());
        var folder = temp.resolve("frozen"); MissionFlightSnapshot.capture(source, folder);
        Files.writeString(folder.resolve("rocket.ork"), "altered");
        assertThrows(java.io.IOException.class, () -> MissionFlightSnapshot.restore(folder));
    }
}
