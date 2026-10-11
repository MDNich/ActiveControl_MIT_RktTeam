package edu.mit.rocket_team;

import edu.mit.rocket_team.FC.*;
import edu.mit.rocket_team.telemetry.*;
import info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener;
import info.openrocket.core.util.BaseTestCase;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import java.nio.file.Path;
import java.util.*;
import static org.junit.jupiter.api.Assertions.*;

class RTFCTimingTest extends BaseTestCase {
    @TempDir Path directory;
    private String previous;

    /** Creates or selects an isolated FC output directory for this regression case. */
    @BeforeEach
    void output() {
        previous = System.getProperty("openrocket.fc.telemetryDir");
        System.setProperty("openrocket.fc.telemetryDir", directory.toString());
    }

    /** Restores preferences, properties and test resources after each regression case. */
    @AfterEach
    void restore() {
        if (previous == null) System.clearProperty("openrocket.fc.telemetryDir");
        else System.setProperty("openrocket.fc.telemetryDir", previous);
    }

    record Result(FlightControllerSimulatorListener listener, List<String> lines) {
        /**
         * Builds or reads the actions fixture for the regression assertions.
         *
         * @param action action identifier or state-entry action
         * @return the resulting List&lt;String&gt;
         */
        List<String> actions(String action) {
            return lines.stream().filter(s -> s.contains("action=" + action + " ")).toList();
        }

        /**
         * Builds or reads the times fixture for the regression assertions.
         *
         * @param action action identifier or state-entry action
         * @return the resulting List&lt;Long&gt;
         */
        List<Long> times(String action) {
            return actions(action).stream().map(s -> value(s, "boot_us")).toList();
        }
    }

    /**
     * Builds or reads the value fixture for the regression assertions.
     *
     * @param line text line to record
     * @param key configuration or lookup key
     * @return computed integer value
     */
    static long value(String line, String key) {
        return Long.parseLong(line.split(key + "=")[1].split(" ")[0]);
    }

    /**
     * Runs the configured test simulation and returns its flight results.
     *
     * @param timing timing
     * @param step step
     * @param manualCommand manual command
     * @return the resulting Result
     * @throws Exception if the requested operation cannot be completed
     */
    private Result run(FlightComputerTimingSettings timing, double step, boolean manualCommand)
            throws Exception {
        List<String> lines = new ArrayList<>();
        var ref =
                new java.util.concurrent.atomic.AtomicReference<
                        FlightControllerSimulatorListener>();
        var listener =
                new FlightControllerSimulatorListener(
                        line -> {
                            lines.add(line);
                            if (manualCommand && line.contains("action=fc.loop_begin count=1 "))
                                ref.get()
                                        .getFlightComputer()
                                        .enqueueCommand(RTFC.angleCommand(3, -117));
                        },
                        step,
                        false,
                        TelemetryLinkSettings.DEFAULT,
                        FlightComputerOutputSettings.DEFAULT,
                        timing);
        ref.set(listener);
        var simulation = RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());
        simulation.getOptions().setMaxSimulationTime(.3);
        simulation.simulate(listener);
        return new Result(listener, lines);
    }

    /**
     * Verifies that default readout advances virtual clock without changing nominal period.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void defaultReadoutAdvancesVirtualClockWithoutChangingNominalPeriod() throws Exception {
        var r = run(FlightComputerTimingSettings.DEFAULT, .0025, false);
        var starts = r.times("fc.loop_begin");
        var sensor = r.times("sensors.deliver");
        for (int i = 1; i < starts.size(); i++)
            assertEquals(10000, starts.get(i) - starts.get(i - 1));
        for (int i = 0; i < sensor.size(); i++) assertEquals(3000, sensor.get(i) - starts.get(i));
        for (String s : r.actions("sensors.deliver"))
            assertTrue(value(s, "age_us") >= 3000 && value(s, "age_us") <= 5500, s);
        var pwm = r.times("pwm.latch");
        for (int i = 1; i < pwm.size(); i++) assertEquals(20000, pwm.get(i) - pwm.get(i - 1));
        assertEquals(0, r.listener.getTimingSummary().overruns());
        assertEquals(3000, r.listener.getTimingSummary().maxExecutionUs());
        assertTrue(r.listener.getTimingSummary().maxPwmSampleAgeUs() > 10000);
    }

    /**
     * Verifies that overruns stretch loops while pwm and gps continue independently.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void overrunsStretchLoopsWhilePwmAndGpsContinueIndependently() throws Exception {
        var r = run(new FlightComputerTimingSettings(3000, 10000, 0, 7000, 1), .0025, false);
        var starts = r.times("fc.loop_begin");
        for (int i = 1; i < starts.size(); i++)
            assertEquals(13000, starts.get(i) - starts.get(i - 1));
        var pwm = r.times("pwm.latch");
        assertEquals(7000, pwm.get(1));
        for (int i = 2; i < pwm.size(); i++) assertEquals(20000, pwm.get(i) - pwm.get(i - 1));
        var gps = r.times("gps.fix_available");
        for (int i = 1; i < gps.size(); i++) assertEquals(100000, gps.get(i) - gps.get(i - 1));
        assertEquals(
                r.listener.getTimingSummary().completedLoops(),
                r.listener.getTimingSummary().overruns());
        assertEquals(13000, r.listener.getTimingSummary().maxExecutionUs());
        var power = r.times("power.command");
        for (int i = 1; i < power.size(); i++)
            assertEquals(104000, power.get(i) - power.get(i - 1));
    }

    /**
     * Verifies that pwm at completion tie uses old command and then updates at next interrupt.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void pwmAtCompletionTieUsesOldCommandAndThenUpdatesAtNextInterrupt() throws Exception {
        var r = run(new FlightComputerTimingSettings(3000, 5000, 0, 8000, 1), .0025, true);
        var pwm = r.actions("pwm.latch");
        assertEquals(8000, value(pwm.get(1), "boot_us"));
        assertTrue(pwm.get(1).contains("exposed=0.0 "));
        assertEquals(28000, value(pwm.get(2), "boot_us"));
        assertTrue(pwm.get(2).contains("exposed=1.0 "));
    }

    /**
     * Verifies that jitter is reproducible independent of physics steps and obeys millis wait.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void jitterIsReproducibleIndependentOfPhysicsStepsAndObeysMillisWait() throws Exception {
        var timing = new FlightComputerTimingSettings(3000, 3000, 9000, 6371, 42);
        var a = run(timing, .0025, false);
        var b = run(timing, .001, false);
        var repeat = run(timing, .0025, false);
        assertEquals(a.times("fc.loop_begin"), b.times("fc.loop_begin"));
        assertEquals(a.times("fc.loop_begin"), repeat.times("fc.loop_begin"));
        assertEquals(a.times("pwm.latch"), b.times("pwm.latch"));
        assertTrue(a.listener.getTimingSummary().overruns() > 0);
        assertTrue(
                a.listener.getTimingSummary().overruns()
                        < a.listener.getTimingSummary().completedLoops());
        for (String s : a.actions("timing.loop")) {
            long start = value(s, "start_us"), end = value(s, "boot_us");
            assertEquals(Math.max(end, (start / 1000 + 10) * 1000), value(s, "next_start_us"));
            assertTrue(value(s, "execution_us") >= 6000 && value(s, "execution_us") <= 15000);
        }
        assertNotEquals(
                a.times("fc.loop_begin"),
                run(new FlightComputerTimingSettings(3000, 3000, 9000, 6371, 43), .0025, false)
                        .times("fc.loop_begin"));
    }

    /**
     * Verifies that one microsecond deadlines and rk4 do not skip busy phases.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void oneMicrosecondDeadlinesAndRk4DoNotSkipBusyPhases() throws Exception {
        boolean original = info.openrocket.core.simulation.listeners.NewControlStepListener.useRK6;
        try {
            var timing = new FlightComputerTimingSettings(1, 0, 0, 1, 1);
            info.openrocket.core.simulation.listeners.NewControlStepListener.useRK6 = true;
            var rk6 = run(timing, .0025, false);
            info.openrocket.core.simulation.listeners.NewControlStepListener.useRK6 = false;
            var rk4 = run(timing, .0025, false);
            assertEquals(rk6.times("fc.loop_begin"), rk4.times("fc.loop_begin"));
            assertEquals(rk6.times("pwm.latch"), rk4.times("pwm.latch"));
            assertEquals(1, rk4.listener.getTimingSummary().maxExecutionUs());
        } finally {
            info.openrocket.core.simulation.listeners.NewControlStepListener.useRK6 = original;
        }
    }

    /** Verifies that timing configuration rejects invalid values. */
    @Test
    void timingConfigurationRejectsInvalidValues() {
        assertThrows(
                IllegalArgumentException.class,
                () -> new FlightComputerTimingSettings(-1, 0, 0, 0, 1));
        assertThrows(
                IllegalArgumentException.class,
                () -> new FlightComputerTimingSettings(3000, 0, 0, 20000, 1));
    }
}
