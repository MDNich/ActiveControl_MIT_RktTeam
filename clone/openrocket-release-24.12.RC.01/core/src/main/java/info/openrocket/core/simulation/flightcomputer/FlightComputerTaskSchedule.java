package info.openrocket.core.simulation.flightcomputer;

/** Common virtual task rule: never overlap invocations or replay an overrun against future data. */
public final class FlightComputerTaskSchedule {
    /** Prevents instantiation of this utility class. */
    private FlightComputerTaskSchedule() {}

    /**
     * Returns the later of nominal start-plus-period and completion, preventing overlapping
     * invocations of one task.
     *
     * @param start start
     * @param period period
     * @param completion completion status recorded in metadata
     * @return the later of nominal start-plus-period and completion, preventing overlapping
     *     invocations of one task
     */
    public static long nextStart(long start, long period, long completion) {
        return Math.max(Math.addExact(start, period), completion);
    }
}
