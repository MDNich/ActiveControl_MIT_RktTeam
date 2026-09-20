package info.openrocket.core.simulation.extension.impl;

import edu.mit.rocket_team.zephyrus.telemetry.TelemetryLinkSettings;
import edu.mit.rocket_team.zephyrus.telemetry.FlightComputerOutputSettings;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.simulation.SimulationConditions;
import info.openrocket.core.simulation.exception.SimulationException;
import info.openrocket.core.simulation.extension.AbstractSimulationExtension;
import info.openrocket.core.simulation.extension.SimulationExtension;
import info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener;
import info.openrocket.core.util.Config;

/** Persistent settings for the existing Zephyrus listener. */
public class ZephyrusFlightComputer extends AbstractSimulationExtension {
    public ZephyrusFlightComputer() { super("Zephyrus flight computer"); }

    public boolean isEnabled() { return config.getBoolean("enabled", false); }
    public TelemetryLinkSettings getLinkSettings() { return settings(config); }

    public FlightComputerOutputSettings getOutputSettings() { return outputs(config); }
    private static FlightComputerOutputSettings outputs(Config c) {
        for (String key : new String[]{"csvFile", "logFile"})
            if (c.containsKey(key) && !(c.get(key, null) instanceof String)) throw new IllegalArgumentException("Invalid output path: " + key);
        return new FlightComputerOutputSettings(c.getString("csvFile", ""), c.getString("logFile", ""));
    }
    private static TelemetryLinkSettings settings(Config c) {
        for (String key : new String[]{"version", "downlinkDelayMs", "randomSeed"}) {
            if (c.containsKey(key)) {
                Object value = c.get(key, null);
                if (!(value instanceof Number n) || !Double.isFinite(n.doubleValue()) ||
                        n.doubleValue() != Math.rint(n.doubleValue()) || n.doubleValue() < 0 ||
                        n.doubleValue() > Integer.MAX_VALUE)
                    throw new IllegalArgumentException("Invalid flight computer setting: " + key);
            }
        }
        if (c.getInt("version", 1) != 1) throw new IllegalArgumentException("Unsupported flight computer settings version");
        if (c.containsKey("enabled") && !(c.get("enabled", null) instanceof Boolean))
            throw new IllegalArgumentException("Invalid flight computer enabled setting");
        if (c.containsKey("packetLossFraction") && !(c.get("packetLossFraction", null) instanceof Number))
            throw new IllegalArgumentException("Invalid packet loss setting");
        return new TelemetryLinkSettings(c.getDouble("packetLossFraction", 0.0),
                c.getInt("downlinkDelayMs", 0), c.getInt("randomSeed", 1));
    }

    public void configure(boolean enabled, TelemetryLinkSettings link) { configure(enabled, link, getOutputSettings()); }
    public void configure(boolean enabled, TelemetryLinkSettings link, FlightComputerOutputSettings output) {
        Config c = getConfig();
        c.put("version", 1);
        c.put("enabled", enabled);
        c.put("packetLossFraction", link.packetLossFraction());
        c.put("downlinkDelayMs", link.delayMs());
        c.put("randomSeed", link.randomSeed());
        c.put("csvFile", output.csvFile());
        c.put("logFile", output.logFile());
        setConfig(c);
    }

    @Override public void setConfig(Config c) { settings(c); outputs(c); super.setConfig(c); }

    @Override public void initialize(SimulationConditions conditions) throws SimulationException {
        try {
            TelemetryLinkSettings link = getLinkSettings();
            if (isEnabled()) conditions.getSimulationListenerList().add(new FlightControllerSimulatorListener(link, getOutputSettings()));
        } catch (IllegalArgumentException e) {
            throw new SimulationException(e.getMessage());
        }
    }

    public static boolean isFlightComputer(SimulationExtension extension) {
        return extension instanceof ZephyrusFlightComputer || extension instanceof JavaCode java &&
                FlightControllerSimulatorListener.class.getName().equals(java.getClassName());
    }

    /** Read a legacy entry without modifying the document. */
    public static ZephyrusFlightComputer read(Simulation simulation) {
        for (SimulationExtension e : simulation.getSimulationExtensions())
            if (e instanceof ZephyrusFlightComputer fc) return fc;
        ZephyrusFlightComputer value = new ZephyrusFlightComputer();
        if (simulation.getSimulationExtensions().stream().anyMatch(ZephyrusFlightComputer::isFlightComputer))
            value.configure(true, TelemetryLinkSettings.DEFAULT);
        return value;
    }

    /** Replace only FC entries; unrelated extensions retain their order and settings. */
    public static void apply(Simulation simulation, boolean enabled, TelemetryLinkSettings link) {
        apply(simulation, enabled, link, read(simulation).getOutputSettings());
    }
    public static void apply(Simulation simulation, boolean enabled, TelemetryLinkSettings link, FlightComputerOutputSettings output) {
        ZephyrusFlightComputer fc = new ZephyrusFlightComputer();
        fc.configure(enabled, link, output);
        var extensions = new java.util.ArrayList<>(simulation.getSimulationExtensions());
        int index = extensions.size();
        for (int i = 0; i < extensions.size(); i++) if (isFlightComputer(extensions.get(i))) { index = i; break; }
        extensions.removeIf(ZephyrusFlightComputer::isFlightComputer);
        extensions.add(Math.min(index, extensions.size()), fc);
        simulation.copyExtensionsFrom(extensions);
    }
}
