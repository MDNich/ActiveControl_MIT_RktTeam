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
    public FlightComputerOutputSettings forRun(String tag) {
        if (tag == null) return this;
        return new FlightComputerOutputSettings(suffix(csvFile, tag), suffix(logFile, tag));
    }
    private static String suffix(String value, String tag) {
        if (value.isEmpty()) return value;
        Path path = Path.of(value);
        String name = path.getFileName().toString();
        int dot = name.lastIndexOf('.');
        if (dot < 0) dot = name.length();
        return path.resolveSibling(name.substring(0,dot) + "-" + tag + name.substring(dot)).toString();
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
