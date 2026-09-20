package edu.mit.rocket_team.zephyrus.telemetry;

import java.nio.file.Path;

/** Empty paths select the existing unique per-run directory. */
public record FlightComputerOutputSettings(String csvFile, String logFile) {
    public static final FlightComputerOutputSettings DEFAULT = new FlightComputerOutputSettings("", "");

    public FlightComputerOutputSettings {
        csvFile = validate(csvFile, true);
        logFile = validate(logFile, false);
        if (!csvFile.isEmpty() && csvFile.equals(logFile)) throw new IllegalArgumentException("CSV and log must use different files");
    }
    private static String validate(String value, boolean csv) {
        if (value == null || value.isBlank()) return "";
        Path path = Path.of(value.trim()).toAbsolutePath().normalize();
        if (path.getFileName() == null) throw new IllegalArgumentException("Choose a filename, not only a folder");
        String name = path.getFileName().toString().toLowerCase(java.util.Locale.ROOT);
        if (csv && !name.endsWith(".csv")) throw new IllegalArgumentException("The telemetry filename must end in .csv");
        return path.toString();
    }
}
