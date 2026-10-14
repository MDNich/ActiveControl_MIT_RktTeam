package info.openrocket.core.simulation.ensemble;

import static org.junit.jupiter.api.Assertions.*;
import static info.openrocket.core.simulation.FlightDataType.*;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.exception.*;
import info.openrocket.core.simulation.listeners.AbstractSimulationListener;
import info.openrocket.core.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;
import org.junit.jupiter.api.Test;

class EnsembleParallelTest extends BaseTestCase {
    @org.junit.jupiter.api.io.TempDir java.nio.file.Path output;

    @Test
    void parallelFlightComputersWriteIndependentCompleteTelemetryAndLogs() throws Exception {
        var sim = edu.mit.rocket_team.RTFCVerificationRocket.simulation(
                edu.mit.rocket_team.RTFCVerificationRocket.rocket());
        sim.getOptions().setMaxSimulationTime(2);
        sim.getOptions().setEnsembleSettings(new EnsembleSettings(true, 2, 0, .05, 0, 0, 0, 42,
                0, 0, 0, 0, 0, 2));
        info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer.apply(sim, true,
                edu.mit.rocket_team.telemetry.TelemetryLinkSettings.DEFAULT,
                new edu.mit.rocket_team.telemetry.FlightComputerOutputSettings(
                        output.resolve("telemetry.csv").toString(), output.resolve("OR.log").toString()));
        sim.simulate();
        java.util.List<java.nio.file.Path> files;
        try (var stream = java.nio.file.Files.list(output)) { files = stream.toList(); }
        var csv = files.stream().filter(p -> p.getFileName().toString().endsWith(".csv")
                && p.getFileName().toString().startsWith("telemetry-ensemble-")
                && !p.getFileName().toString().endsWith("-transmitted.csv")).sorted().toList();
        var logs = files.stream().filter(p -> p.getFileName().toString().endsWith(".log")).sorted().toList();
        assertEquals(2, csv.size(), files.toString());
        assertEquals(2, logs.size(), files.toString());
        assertTrue(csv.get(0).getFileName().toString().endsWith("-run-1.csv"));
        assertTrue(csv.get(1).getFileName().toString().endsWith("-run-2.csv"));
        assertTrue(java.nio.file.Files.readAllLines(csv.get(0)).size() > 2);
        assertEquals(java.nio.file.Files.readString(csv.get(0)), java.nio.file.Files.readString(csv.get(1)));
        for (var log : logs) assertTrue(java.nio.file.Files.readString(log).contains("simulation.end"));
        assertFalse(java.nio.file.Files.exists(output.resolve("telemetry.csv")));
        assertEquals(2, sim.getSimulatedData().getEnsembleResult().settings().runs());
    }

    private EnsembleSettings settings(int threads) {
        return new EnsembleSettings(true, 4, .1, .05, .1, .001, .1, 42,
                .01, .02, .03, .04, Math.toRadians(15), threads);
    }

    @Test
    void bearingRotationUsesNorthClockwiseConventionAndKeepsWindSpeed() {
        var listener = new EnsembleRunListener(settings(1),
                new EnsembleRunParameters(1, 12, 288, 101325, 0, 0, 1, 1, 1, 1, Math.PI / 2));
        var wind = listener.postWindModel(null, new Coordinate(0, 10, 2));
        assertEquals(10, wind.x, 1e-12);
        assertEquals(0, wind.y, 1e-12);
        assertEquals(2, wind.z);
        assertEquals(Math.sqrt(104), wind.length(), 1e-12);
    }

    @Test
    void threadCountDoesNotChangeDrawsStatisticsOrRunOrderAndWorkersOverlap() throws Exception {
        var sim = EnsembleTest.simulation();
        sim.getOptions().setMaxSimulationTime(2);
        sim.getOptions().setEnsembleSettings(settings(1));
        sim.simulate();
        var serial = sim.getSimulatedData().getEnsembleResult();
        sim.getOptions().setEnsembleSettings(settings(4));
        var active = new AtomicInteger();
        var peak = new AtomicInteger();
        var together = new CountDownLatch(4);
        sim.simulate(new AbstractSimulationListener() {
            @Override public void startSimulation(SimulationStatus status) throws SimulationException {
                peak.accumulateAndGet(active.incrementAndGet(), Math::max);
                together.countDown();
                try { if (!together.await(10, TimeUnit.SECONDS)) throw new SimulationException("Workers did not overlap"); }
                catch (InterruptedException e) { Thread.currentThread().interrupt(); throw new SimulationCancelledException(e); }
            }
            @Override public void endSimulation(SimulationStatus status, SimulationException exception) { active.decrementAndGet(); }
        });
        assertEquals(4, peak.get());
        assertEquals(0, active.get());
        var parallel = sim.getSimulatedData().getEnsembleResult();
        for (var metric : EnsembleMetric.values()) assertArrayEquals(serial.samples(0, metric), parallel.samples(0, metric), 0);
        for (int i = 0; i < 4; i++) assertEquals(serial.individualRuns().parameters(i), parallel.individualRuns().parameters(i));
        assertEquals(serial.deviation(0).getMaximum(TYPE_ALTITUDE), parallel.deviation(0).getMaximum(TYPE_ALTITUDE));
        assertEquals(0, sim.getEnsembleRunNumber());
    }

    @Test
    void directionOnlyProducesLateralSpreadAndIsRecorded() throws Exception {
        var sim = EnsembleTest.simulation();
        sim.getOptions().setEnsembleSettings(new EnsembleSettings(true, 3, 0, .05, 0, 0, 0, 42, 0, 0, 0, 0, .3, 3));
        sim.simulate();
        var result = sim.getSimulatedData().getEnsembleResult();
        assertTrue(result.deviation(0).getMaximum(TYPE_POSITION_X) > .01);
        assertNotEquals(0, result.individualRuns().parameters(0).windDirectionOffset());
        assertEquals("Atmosphere", result.settings().sourceLabel());
    }

    @Test
    void cancellingOneMemberWaitsForAllWorkersToCloseAndPreservesPreviousResult() throws Exception {
        var sim = EnsembleTest.simulation();
        sim.getOptions().setMaxSimulationTime(1);
        sim.simulate();
        var previous = sim.getSimulatedData();
        sim.getOptions().setEnsembleSettings(settings(4));
        var active = new AtomicInteger();
        assertThrows(SimulationCancelledException.class, () -> sim.simulate(new AbstractSimulationListener() {
            @Override public void startSimulation(SimulationStatus status) { active.incrementAndGet(); }
            @Override public void postStep(SimulationStatus status) throws SimulationException { throw new SimulationCancelledException(); }
            @Override public void endSimulation(SimulationStatus status, SimulationException exception) { active.decrementAndGet(); }
        }));
        assertEquals(0, active.get());
        assertSame(previous, sim.getSimulatedData());
    }
}
