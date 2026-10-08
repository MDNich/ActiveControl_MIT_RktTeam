package edu.mit.rocket_team.zephyrus.FC;

/** Virtual execution costs, not measurements of a particular MCU. All values are microseconds. */
public record FlightComputerTimingSettings(int sensorReadUs, int extraWorkUs, int workJitterUs,
                                          int pwmPhaseUs, int randomSeed) {
    // The selected baro.cpp explicitly waits 1500 us twice. Other costs need hardware measurements.
    public static final FlightComputerTimingSettings DEFAULT = new FlightComputerTimingSettings(3000, 0, 0, 0, 1);
    public static final FlightComputerTimingSettings IDEAL = new FlightComputerTimingSettings(0, 0, 0, 0, 1);
    public FlightComputerTimingSettings {
        if (sensorReadUs < 0 || sensorReadUs > 1_000_000 || extraWorkUs < 0 || extraWorkUs > 1_000_000 ||
                workJitterUs < 0 || workJitterUs > 1_000_000)
            throw new IllegalArgumentException("FC execution delays must be between 0 and 1000 ms");
        if (pwmPhaseUs < 0 || pwmPhaseUs >= RTFC.PWM_US)
            throw new IllegalArgumentException("PWM phase must be at least 0 and less than 20 ms");
        if (randomSeed < 0) throw new IllegalArgumentException("FC timing seed must be nonnegative");
    }
}
