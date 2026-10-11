package info.openrocket.core.simulation.flightcomputer;

import java.io.StringReader;
import java.nio.charset.StandardCharsets;
import java.util.function.*;
import java.util.prefs.Preferences;
import org.codehaus.janino.SimpleCompiler;

/**
 * API for an embedded Java board. A new program instance and class loader are created for each run.
 */
public abstract class JavaBoardProgram {
    public static final String MODEL_NOTICE =
            "Custom Java boards are for algorithm experiments. Communication between boards is not accurately simulated: boards share measured inputs directly, without bus delays, packet loss or contention. These results are not authoritative predictions of hardware behavior.";

    /**
     * Runs one custom-board invocation using its measured-input snapshot; instance fields persist
     * only within this simulation run.
     *
     * @param io io
     */
    public abstract void step(Context io);

    public static final class Context {
        private final long timeUs, dtUs;
        private final String state;
        private final ToDoubleFunction<String> signals;
        private final Predicate<String> connected;

        @FunctionalInterface
        public interface Output {
            /**
             * Receives a board output command with its port, optional named physical target and
             * value.
             *
             * @param port logical controller port name
             * @param target destination component or output target
             * @param value replacement or input value
             */
            void accept(String port, String target, double value);
        }

        private final Output output;
        private final Consumer<String> log;

        /**
         * Creates a Context value from the supplied time us, dt us, state, signals, connected,
         * output, log.
         *
         * @param timeUs time in microseconds
         * @param dtUs dt in microseconds
         * @param state current flight phase or state
         * @param signals initial signals
         * @param connected initial connected
         * @param output output snapshot or destination
         * @param log initial log
         */
        public Context(
                long timeUs,
                long dtUs,
                String state,
                ToDoubleFunction<String> signals,
                Predicate<String> connected,
                Output output,
                Consumer<String> log) {
            this.timeUs = timeUs;
            this.dtUs = dtUs;
            this.state = state;
            this.signals = signals;
            this.connected = connected;
            this.output = output;
            this.log = log;
        }

        /**
         * Virtual invocation start; outputs become visible after the board's configured execution
         * cost.
         */
        public long timeUs() {
            return timeUs;
        }

        /**
         * Returns elapsed virtual microseconds since this board's previous start.
         *
         * @return elapsed virtual microseconds since this board's previous start
         */
        public long dtUs() {
            return dtUs;
        }

        /**
         * Returns the design state ID captured when this invocation started.
         *
         * @return the design state ID captured when this invocation started
         */
        public String stateId() {
            return state;
        }

        /**
         * Tests whether the design contains hardware on a named logical output port.
         *
         * @param port logical controller port name
         * @return true when the condition described above holds
         */
        public boolean connected(String port) {
            return connected.test(port);
        }

        /**
         * Reads a named measured FC signal from the invocation snapshot.
         *
         * @param name display name or lookup key
         * @return computed numeric value
         */
        public double signal(String name) {
            return signals.applyAsDouble(name);
        }

        /**
         * Returns measured relative barometric altitude in metres.
         *
         * @return measured relative barometric altitude in metres
         */
        public double altitudeM() {
            return signal("baro_altitude");
        }

        /**
         * Returns the FC's measured longitudinal velocity estimate in metres per second.
         *
         * @return the FC's measured longitudinal velocity estimate in metres per second
         */
        public double velocityMps() {
            return signal("velocity");
        }

        /**
         * Returns the measured longitudinal acceleration signal in metres per second squared.
         *
         * @return the measured longitudinal acceleration signal in metres per second squared
         */
        public double accelerationMps2() {
            return signal("accel_vertical");
        }

        /**
         * Queues a normalized 0–1 airbrake command for publication when this board invocation
         * completes.
         *
         * @param fraction fraction
         */
        public void setAirbrakes(double fraction) {
            range(fraction, 0, 1);
            output.accept("airbrakes", null, fraction);
        }

        /**
         * Queues a roll-tab command in degrees, limited to −90 through +90, for publication at task
         * completion.
         *
         * @param angle angle
         */
        public void setRollDegrees(double angle) {
            range(angle, -90, 90);
            output.accept("roll", null, angle);
        }

        /**
         * Queues a logical recovery-channel firing command for task completion.
         *
         * @param channel logical recovery channel, numbered 0 through 5
         */
        public void fireRecovery(int channel) {
            range(channel, 0, 5);
            output.accept("recovery", null, (double) channel);
        }

        /**
         * Writes a custom-board message to the FC run trace.
         *
         * @param text text
         */
        public void log(String text) {
            log.accept(text);
        }

        /** Exact, case-sensitive rocket component name, not the FC designer node name. */
        public void setRollDegrees(String finSetName, double angle) {
            range(angle, -90, 90);
            output.accept("roll", target(finSetName), angle);
        }

        /**
         * Queues a normalized 0–1 airbrake command for publication when this board invocation
         * completes.
         *
         * @param airbrakeSetName airbrake set name
         * @param fraction fraction
         */
        public void setAirbrakes(String airbrakeSetName, double fraction) {
            range(fraction, 0, 1);
            output.accept("airbrakes", target(airbrakeSetName), fraction);
        }

        /**
         * Validates a named physical target before queuing a board output.
         *
         * @param name display name or lookup key
         * @return formatted or resolved text
         * @throws IllegalArgumentException when rejected: A nonblank rocket component name is
         *     required
         */
        private static String target(String name) {
            if (name == null || name.isBlank() || name.contains("\n") || name.contains("\r"))
                throw new IllegalArgumentException("A nonblank rocket component name is required");
            return name;
        }

        /**
         * Rejects non-finite values and commands outside the supported output interval.
         *
         * @param value replacement or input value
         * @param min lower allowed bound
         * @param max upper allowed bound
         * @throws IllegalArgumentException when rejected: Board output must be in [
         */
        private static void range(double value, double min, double max) {
            if (!Double.isFinite(value) || value < min || value > max)
                throw new IllegalArgumentException(
                        "Board output must be in [" + min + ", " + max + "]");
        }
    }

    /**
     * Returns the starter Java source shown by the custom-board editor.
     *
     * @return the starter Java source shown by the custom-board editor
     */
    public static String template() {
        return """
            import info.openrocket.core.simulation.flightcomputer.JavaBoardProgram;

            public class BoardProgram extends JavaBoardProgram {
                private boolean announced;

                public void step(Context io) {
                    // State is kept in fields for this run only.
                    if (!announced) {
                        io.log("Custom board started");
                        announced = true;
                    }
                    // Sensor values are measured FC inputs, not simulator truth.
                    // double altitude = io.altitudeM();
                    // io.setAirbrakes(0.25);  // fraction 0..1
                    // io.setRollDegrees(10); // all roll tabs, degrees -90..90
                    // io.setRollDegrees("Forward fins", 10); // exact rocket fin-set name
                    // io.setAirbrakes("Aft airbrakes", 0.25); // exact rocket airbrake-set name
                    // io.fireRecovery(2);   // channels 0..5; map devices in the Flight computer tab
                }
            }
            """;
    }

    public static final class Compiled {
        private final Class<? extends JavaBoardProgram> type;

        /**
         * Creates a Compiled value from the supplied type.
         *
         * @param type initial type
         */
        private Compiled(Class<? extends JavaBoardProgram> type) {
            this.type = type;
        }

        /**
         * Creates a fresh program instance from the compiled board class for one run.
         *
         * @return the resulting JavaBoardProgram
         * @throws IllegalArgumentException when rejected: Cannot start Java board:
         */
        public JavaBoardProgram create() {
            try {
                return type.getDeclaredConstructor().newInstance();
            } catch (ReflectiveOperationException | LinkageError e) {
                throw new IllegalArgumentException("Cannot start Java board: " + e.getMessage(), e);
            }
        }
    }

    /** Compilation and inspection do not initialize the class or execute its constructor. */
    public static Compiled compile(String source) {
        if (source.isBlank() || source.length() > 100000)
            throw new IllegalArgumentException(
                    "Java board source must contain 1–100000 characters");
        try {
            var compiler = new SimpleCompiler();
            compiler.setParentClassLoader(JavaBoardProgram.class.getClassLoader());
            compiler.setDebuggingInformation(true, true, false);
            compiler.cook("BoardProgram.java", new StringReader(source));
            var type =
                    Class.forName("BoardProgram", false, compiler.getClassLoader())
                            .asSubclass(JavaBoardProgram.class);
            if (java.lang.reflect.Modifier.isAbstract(type.getModifiers()))
                throw new IllegalArgumentException("BoardProgram must implement step(Context io)");
            type.getConstructor();
            return new Compiled(type);
        } catch (Exception | LinkageError e) {
            throw new IllegalArgumentException("Java board: " + e.getMessage(), e);
        }
    }

    /**
     * Returns the local preference store used to record trusted custom Java source hashes.
     *
     * @return the local preference store used to record trusted custom Java source hashes
     */
    private static Preferences approvals() {
        return Preferences.userNodeForPackage(JavaBoardProgram.class)
                .node("approved-java-board-source-v1");
    }

    /**
     * Calculates the source hash used for local execution approval.
     *
     * @param source source text, model or measured-channel selection
     * @return formatted or resolved text
     */
    private static String hash(String source) {
        return FlightComputerDesign.hash(source.getBytes(StandardCharsets.UTF_8));
    }

    /**
     * Checks whether this exact source has been approved locally for execution.
     *
     * @param source source text, model or measured-channel selection
     * @return true when the condition described above holds
     */
    public static boolean approved(String source) {
        return approvals().getBoolean(hash(source), false);
    }

    /**
     * Records local approval for this exact Java source; edits produce a different approval hash.
     *
     * @param source source text, model or measured-channel selection
     */
    public static void approve(String source) {
        approvals().putBoolean(hash(source), true);
    }

    /**
     * Removes local execution approval for this Java source.
     *
     * @param source source text, model or measured-channel selection
     */
    public static void revoke(String source) {
        approvals().remove(hash(source));
    }

    /**
     * Rejects execution of custom Java source without matching local approval.
     *
     * @param design external flight-computer design snapshot
     * @throws IllegalArgumentException when rejected: Open Java code for \'
     */
    public static void requireApproved(FlightComputerDesign design) {
        for (var n : design.nodes().getValuesAs(jakarta.json.JsonObject.class))
            if (n.getString("type").equals("java_board")) {
                if (!approved(n.getString("program")))
                    throw new IllegalArgumentException(
                            "Open Java code for '"
                                    + n.getString("name")
                                    + "' in the designer and enable this source before running it.");
                compile(n.getString("program"));
            }
    }
}
