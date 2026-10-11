package edu.mit.rocket_team.telemetry;

import java.nio.file.Path;

/** Empty paths select the existing unique per-run directory. */
public record FlightComputerOutputSettings(String csvFile, String logFile) {
    public static final FlightComputerOutputSettings DEFAULT =
            new FlightComputerOutputSettings("", "");

    /**
     * Creates a FlightComputerOutputSettings after validating the supplied configuration.
     *
     * @param csvFile csv file
     * @param logFile log file
     * @throws IllegalArgumentException when rejected: CSV and log must use different files
     */
    public FlightComputerOutputSettings {
        csvFile = validate(csvFile, true);
        logFile = validate(logFile, false);
        if (!csvFile.isEmpty() && csvFile.equals(logFile))
            throw new IllegalArgumentException("CSV and log must use different files");
    }

    /**
     * Derives distinct output filenames for a member of a multi-run simulation.
     *
     * @param tag tag
     * @return the resulting FlightComputerOutputSettings
     */
    public FlightComputerOutputSettings forRun(String tag) {
        if (tag == null) return this;
        return new FlightComputerOutputSettings(suffix(csvFile, tag), suffix(logFile, tag));
    }

    /**
     * Inserts a run suffix before the output filename extension.
     *
     * @param value replacement or input value
     * @param tag tag
     * @return formatted or resolved text
     */
    private static String suffix(String value, String tag) {
        if (value.isEmpty()) return value;
        Path path = Path.of(value);
        String name = path.getFileName().toString();
        int dot = name.lastIndexOf('.');
        if (dot < 0) dot = name.length();
        return path.resolveSibling(name.substring(0, dot) + "-" + tag + name.substring(dot))
                .toString();
    }

    /**
     * Validates an output path and its required extension before opening any files.
     *
     * @param value replacement or input value
     * @param csv csv
     * @return formatted or resolved text
     * @throws IllegalArgumentException when rejected: Choose a filename, not only a folder
     */
    private static String validate(String value, boolean csv) {
        if (value == null || value.isBlank()) return "";
        Path path = Path.of(value.trim()).toAbsolutePath().normalize();
        if (path.getFileName() == null)
            throw new IllegalArgumentException("Choose a filename, not only a folder");
        String name = path.getFileName().toString().toLowerCase(java.util.Locale.ROOT);
        if (csv && !name.endsWith(".csv"))
            throw new IllegalArgumentException("The telemetry filename must end in .csv");
        return path.toString();
    }
}
