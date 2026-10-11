package edu.mit.rocket_team;

import java.nio.file.*;
import java.util.*;
import java.util.function.Consumer;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import info.openrocket.core.util.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.rocketcomponent.position.AxialMethod;
import info.openrocket.core.motor.*;
import info.openrocket.core.document.*;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.listeners.*;
import edu.mit.rocket_team.FC.RTFC;
import static org.junit.jupiter.api.Assertions.*;

class RTFCSimulationTest extends BaseTestCase {
    @TempDir Path dir;
    private String previous;

    /** Creates an isolated telemetry output directory for each regression case. */
    @BeforeEach
    void outputDirectory() {
        previous = System.getProperty("openrocket.fc.telemetryDir");
        System.setProperty("openrocket.fc.telemetryDir", dir.toString());
    }

    /** Restores preferences, properties and test resources after each regression case. */
    @AfterEach
    void restore() {
        if (previous == null) System.clearProperty("openrocket.fc.telemetryDir");
        else System.setProperty("openrocket.fc.telemetryDir", previous);
    }

    /**
     * Builds the rocket fixture used by this regression case.
     *
     * @return the resulting Rocket
     */
    static Rocket rocket() {
        return RTFCVerificationRocket.rocket();
    }

    /**
     * Builds a simulation configured for this regression case.
     *
     * @param rocket rocket model for this operation
     * @return the resulting Simulation
     */
    static Simulation simulation(Rocket rocket) {
        return RTFCVerificationRocket.simulation(rocket);
    }

    private record Result(
            Simulation simulation, FlightControllerSimulatorListener listener, List<Long> ticks) {}

    /**
     * Runs the configured test simulation and returns its flight results.
     *
     * @param step step
     * @param closed closed
     * @return the resulting Result
     * @throws Exception if the requested operation cannot be completed
     */
    private Result run(double step, boolean closed) throws Exception {
        List<Long> ticks = new java.util.ArrayList<>();
        Consumer<String> log =
                s -> {
                    if (s.contains("action=fc.loop_begin"))
                        ticks.add(Long.parseLong(s.split("boot_us=")[1].split(" ")[0]));
                };
        FlightControllerSimulatorListener fc =
                new FlightControllerSimulatorListener(log, step, closed);
        Simulation sim = simulation(rocket());
        sim.simulate(fc);
        return new Result(sim, fc, ticks);
    }

    /**
     * Verifies that closed loop changes trajectory and survives landing stepper copies.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void closedLoopChangesTrajectoryAndSurvivesLandingStepperCopies() throws Exception {
        Result active = run(0.0025, false), closed = run(0.0025, true);
        assertEquals(
                edu.mit.rocket_team.util.RTRocketState.APOGEE,
                active.listener.getFlightComputer().getState());
        assertTrue(active.listener.getFlightComputer().getAirbrakeUpdateCount() > 1000);
        assertTrue(
                active.simulation.getSimulatedData().getMaxAltitude()
                        < closed.simulation.getSimulatedData().getMaxAltitude() - 0.01,
                "Active="
                        + active.simulation.getSimulatedData().getMaxAltitude()
                        + " closed="
                        + closed.simulation.getSimulatedData().getMaxAltitude());
        assertEquals(0, active.listener.getFlightComputer().getOutput().exposedFraction());
        assertTrue(Files.size(active.listener.getFlightComputer().telemetry.getCsvPath()) > 1000);
    }

    /**
     * Verifies that tick schedule independent of step and repeated runs.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void tickScheduleIndependentOfStepAndRepeatedRuns() throws Exception {
        Result a = run(0.0025, false), b = run(0.001, false), repeat = run(0.0025, false);
        assertEquals(a.ticks, b.ticks);
        assertEquals(a.ticks, repeat.ticks);
        assertEquals(
                a.simulation.getSimulatedData().getMaxAltitude(),
                repeat.simulation.getSimulatedData().getMaxAltitude(),
                1e-8);
        assertEquals(
                a.simulation.getSimulatedData().getMaxAltitude(),
                b.simulation.getSimulatedData().getMaxAltitude(),
                2.0);
    }

    /**
     * Verifies that rk4 also respects firmware deadlines.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void rk4AlsoRespectsFirmwareDeadlines() throws Exception {
        boolean original = NewControlStepListener.useRK6;
        try {
            NewControlStepListener.useRK6 = true;
            Result rk6 = run(0.0025, false);
            NewControlStepListener.useRK6 = false;
            Result rk4 = run(0.0025, false);
            assertEquals(rk6.ticks, rk4.ticks);
            assertEquals(
                    rk6.simulation.getSimulatedData().getMaxAltitude(),
                    rk4.simulation.getSimulatedData().getMaxAltitude(),
                    2.0);
        } finally {
            NewControlStepListener.useRK6 = original;
        }
    }

    /**
     * Verifies that zero launch rod and off grid ignition make positive progress.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void zeroLaunchRodAndOffGridIgnitionMakePositiveProgress() throws Exception {
        Simulation sim = simulation(rocket());
        sim.getOptions().setLaunchRodLength(0);
        sim.getOptions().setMaxSimulationTime(1);
        for (RocketComponent c : sim.getRocket())
            if (c instanceof BodyTube body && body.isMotorMount())
                body.getMotorConfig(sim.getFlightConfigurationId()).setIgnitionDelay(0.1237);
        var fc = new FlightControllerSimulatorListener(s -> {}, 0.0025, false);
        sim.simulate(fc);
        assertTrue(fc.getFlightComputer().getLoopCount() >= 200);
        assertEquals(
                edu.mit.rocket_team.util.RTRocketState.FLIGHT, fc.getFlightComputer().getState());
    }

    /**
     * Verifies that component missing is actionable and airbrake drag is monotonic.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void componentMissingIsActionableAndAirbrakeDragIsMonotonic() throws Exception {
        Rocket r = rocket();
        AirbrakeSet b = null;
        for (RocketComponent c : r) if (c instanceof AirbrakeSet a) b = a;
        assertNotNull(b);
        info.openrocket.core.aerodynamics.FlightConditions cond =
                new info.openrocket.core.aerodynamics.FlightConditions(
                        r.getSelectedConfiguration());
        cond.setAOA(0);
        cond.setMach(0.5);
        var calc = new info.openrocket.core.aerodynamics.barrowman.AirbrakeSetCalc(b);
        b.setFracExposed(0);
        double zero =
                calc.calculatePressureCD(cond, 0, 0, new info.openrocket.core.logging.WarningSet());
        b.setFracExposed(0.5);
        double half =
                calc.calculatePressureCD(cond, 0, 0, new info.openrocket.core.logging.WarningSet());
        b.setFracExposed(1);
        double full =
                calc.calculatePressureCD(cond, 0, 0, new info.openrocket.core.logging.WarningSet());
        assertTrue(full > half && half > zero);
        b.getParent().removeChild(b);
        Exception e =
                assertThrows(
                        Exception.class,
                        () ->
                                simulation(r)
                                        .simulate(
                                                new FlightControllerSimulatorListener(
                                                        s -> {}, 0.0025, false)));
        assertTrue(e.getMessage().contains("AirbrakeSet"));
    }
}
