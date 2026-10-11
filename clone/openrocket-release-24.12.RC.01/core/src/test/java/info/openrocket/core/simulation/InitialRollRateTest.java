package info.openrocket.core.simulation;

import info.openrocket.core.document.*;
import info.openrocket.core.file.*;
import info.openrocket.core.file.openrocket.OpenRocketSaver;
import info.openrocket.core.file.openrocket.importt.OpenRocketLoader;
import info.openrocket.core.logging.*;
import info.openrocket.core.util.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import static org.junit.jupiter.api.Assertions.*;

class InitialRollRateTest extends BaseTestCase {
    @TempDir Path directory;

    /** Verifies that initial spin follows body axis on vertical and tilted rods. */
    @Test
    void initialSpinFollowsBodyAxisOnVerticalAndTiltedRods() {
        var rocket = TestRockets.makeEstesAlphaIII();
        for (double tilt : new double[] {0, Math.toRadians(25)}) {
            for (double rate : new double[] {0, -2 * Math.PI, 2 * Math.PI}) {
                var options = new SimulationOptions();
                options.setLaunchIntoWind(false);
                options.setLaunchRodAngle(tilt);
                options.setLaunchRodDirection(.7);
                options.setInitialRollRate(rate);
                var conditions = options.toSimulationConditions();
                var status = new SimulationStatus(rocket.getSelectedConfiguration(), conditions);
                var bodyRate =
                        status.getRocketOrientationQuaternion()
                                .invRotate(status.getRocketRotationVelocity());
                assertEquals(0, bodyRate.x, 1e-12);
                assertEquals(0, bodyRate.y, 1e-12);
                assertEquals(rate, bodyRate.z, 1e-12);
                assertEquals(
                        Math.abs(rate) * Math.cos(tilt),
                        Math.abs(status.getRocketRotationVelocity().z),
                        1e-12);
            }
        }
    }

    /**
     * Verifies that spin persists along rod then responds to aerodynamic damping.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void spinPersistsAlongRodThenRespondsToAerodynamicDamping() throws Exception {
        for (double rate : new double[] {0, -2 * Math.PI, 2 * Math.PI}) {
            var simulation = new Simulation(TestRockets.makeEstesAlphaIII());
            simulation.setFlightConfigurationId(TestRockets.TEST_FCID_0);
            var options = simulation.getOptions();
            options.setLaunchIntoWind(false);
            options.setLaunchRodAngle(Math.toRadians(15));
            options.setLaunchRodLength(1);
            options.getAverageWindModel().setAverage(0);
            options.getAverageWindModel().setStandardDeviation(0);
            options.setGeodeticComputation(GeodeticComputationStrategy.FLAT);
            options.setTimeStep(.005);
            options.setMaxSimulationTime(1.3);
            options.setInitialRollRate(rate);
            simulation.simulate();
            var branch = simulation.getSimulatedData().getBranch(0);
            double clearance =
                    branch.getEvents().stream()
                            .filter(e -> e.getType() == FlightEvent.Type.LAUNCHROD)
                            .mapToDouble(FlightEvent::getTime)
                            .findFirst()
                            .orElseThrow();
            assertTrue(clearance > 0);
            var times = branch.get(FlightDataType.TYPE_TIME);
            var rates = branch.get(FlightDataType.TYPE_ROLL_RATE);
            int rodSamples = 0;
            for (int i = 0; i < times.size(); i++)
                if (times.get(i) < clearance && Double.isFinite(rates.get(i))) {
                    assertEquals(rate, rates.get(i), 1e-8);
                    rodSamples++;
                }
            assertTrue(rodSamples > 3);
            double finalRate = branch.getLast(FlightDataType.TYPE_ROLL_RATE);
            if (rate == 0) assertEquals(0, finalRate, 1e-6);
            else {
                assertTrue(
                        rate * finalRate > 0,
                        "Damping should preserve the direction of this initial spin");
                assertTrue(
                        Math.abs(finalRate) < Math.abs(rate) - .01,
                        "Spin should damp after clearance: " + finalRate);
            }
            System.out.println(
                    "INITIAL_SPIN rate="
                            + rate
                            + " clearance_s="
                            + clearance
                            + " final_rate="
                            + finalRate);
            options.setInitialRollRate(rate + 1);
            assertEquals(Simulation.Status.OUTDATED, simulation.getStatus());
        }
    }

    /** Verifies that spin is copied and non finite settings are rejected. */
    @Test
    void spinIsCopiedAndNonFiniteSettingsAreRejected() {
        var options = new SimulationOptions();
        var copy = options.clone();
        options.setInitialRollRate(-4.5);
        assertNotEquals(options, copy);
        copy.copyConditionsFrom(options);
        assertEquals(options, copy);
        assertEquals(-4.5, options.clone().getInitialRollRate());
        for (double bad :
                new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
            assertThrows(IllegalArgumentException.class, () -> options.setInitialRollRate(bad));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new SimulationConditions().setInitialRollRate(bad));
        }
    }

    /**
     * Verifies that ork round trip keeps spin and older files default to zero.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void orkRoundTripKeepsSpinAndOlderFilesDefaultToZero() throws Exception {
        var document = OpenRocketDocumentFactory.createNewRocket();
        var simulation = new Simulation(document, document.getRocket());
        simulation.getOptions().setInitialRollRate(Math.toRadians(-123.456));
        document.addSimulation(simulation);
        var bytes = new ByteArrayOutputStream();
        new OpenRocketSaver()
                .save(bytes, document, new StorageOptions(), new WarningSet(), new ErrorSet());
        String xml = bytes.toString(StandardCharsets.UTF_8);
        assertTrue(xml.contains("<initialrollrate>"));
        assertEquals(
                simulation.getOptions().getInitialRollRate(), load(xml).getInitialRollRate(), 1e-8);
        assertEquals(
                0,
                load(xml.replaceAll("<initialrollrate>[^<]*</initialrollrate>", ""))
                        .getInitialRollRate());
        assertEquals(
                0,
                load(xml.replaceAll(
                                "<initialrollrate>[^<]*</initialrollrate>",
                                "<initialrollrate>Infinity</initialrollrate>"))
                        .getInitialRollRate());
    }

    /**
     * Builds or reads the load fixture for the regression assertions.
     *
     * @param xml xml
     * @return the resulting SimulationOptions
     * @throws Exception if the requested operation cannot be completed
     */
    private SimulationOptions load(String xml) throws Exception {
        var context = new DocumentLoadingContext();
        context.setOpenRocketDocument(OpenRocketDocumentFactory.createEmptyRocket());
        context.setAttachmentFactory(new FileSystemAttachmentFactory(directory.toFile()));
        new OpenRocketLoader()
                .loadFromStream(
                        context,
                        new ByteArrayInputStream(xml.getBytes(StandardCharsets.UTF_8)),
                        "spin.ork");
        return context.getOpenRocketDocument().getSimulation(0).getOptions();
    }
}
