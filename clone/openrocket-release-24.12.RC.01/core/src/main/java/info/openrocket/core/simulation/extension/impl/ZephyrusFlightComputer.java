package info.openrocket.core.simulation.extension.impl;

import edu.mit.rocket_team.zephyrus.telemetry.TelemetryLinkSettings;
import edu.mit.rocket_team.zephyrus.FC.FlightComputerTimingSettings;
import edu.mit.rocket_team.zephyrus.telemetry.FlightComputerOutputSettings;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.simulation.SimulationConditions;
import info.openrocket.core.simulation.exception.SimulationException;
import info.openrocket.core.simulation.extension.AbstractSimulationExtension;
import info.openrocket.core.simulation.extension.SimulationExtension;
import info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener;
import info.openrocket.core.util.Config;
import info.openrocket.core.simulation.flightcomputer.*;

/** Persistent settings for the existing Zephyrus listener. */
public class ZephyrusFlightComputer extends AbstractSimulationExtension {
    public ZephyrusFlightComputer() { super("Flight computer"); }

    private transient FlightComputerLibrary.Resolved preparedDesign;
    public String getDesignReference() { return config.getString("designFile", ""); }
    public boolean hasDesignFile() { return !getDesignReference().isBlank(); }
    public FlightComputerLibrary.Resolved getPreparedDesign() { return preparedDesign; }
    public boolean prepareDesign() throws SimulationException {
        if(!isEnabled() || !hasDesignFile() || preparedDesign!=null)return false;
        try {
            var loaded=FlightComputerLibrary.load(getDesignReference());
            if(!config.getString("designId",loaded.design().id()).equals(loaded.design().id()))throw new IllegalArgumentException("The referenced FC file was replaced with a different design. Select it again explicitly.");
            JavaBoardProgram.requireApproved(loaded.design());
            preparedDesign=loaded;return true;
        }
        catch(java.io.IOException | IllegalArgumentException e){throw new SimulationException(e.getMessage(),e);}
    }
    public void releaseDesign(){preparedDesign=null;}
    public String currentFingerprint(){
        try {return hasDesignFile()?FlightComputerDesign.read(FlightComputerLibrary.resolve(getDesignReference())).fingerprint():"";}
        catch(Exception e){return "unavailable";}
    }
    public void setDesignReference(String reference) {
        Config old=getConfig(),c=new Config();
        for(String key:old.keySet())if(!java.util.Set.of("sensorReadUs","extraWorkUs","workJitterUs","pwmPhaseUs","timingSeed").contains(key))c.put(key,old.get(key,null));
        c.put("designFile",reference);
        setConfig(c);preparedDesign=null;
    }
    public static void selectFile(Simulation simulation,java.nio.file.Path file) throws java.io.IOException {
        var fc=(ZephyrusFlightComputer)read(simulation).clone();
        var imported=FlightComputerLibrary.importFile(file);
        fc.setDesignReference(FlightComputerLibrary.reference(imported));
        var c=fc.getConfig();var definition=FlightComputerDesign.read(imported);c.put("designId",definition.id());c.put("computer",definition.computer());fc.setConfig(c);
        replace(simulation,fc);
    }
    public boolean isEnabled() { return config.getBoolean("enabled", false); }
    public TelemetryLinkSettings getLinkSettings() { return settings(config); }

    public FlightComputerTimingSettings getTimingSettings() {
        if(preparedDesign!=null)return preparedDesign.design().timing();
        if(hasDesignFile())try{return FlightComputerDesign.read(FlightComputerLibrary.resolve(getDesignReference())).timing();}catch(Exception ignored){}
        return timing(config);
    }
    private static FlightComputerTimingSettings timing(Config c) {
        var d=FlightComputerTimingSettings.DEFAULT;
        String[] keys={"sensorReadUs","extraWorkUs","workJitterUs","pwmPhaseUs","timingSeed"};
        int[] values={d.sensorReadUs(),d.extraWorkUs(),d.workJitterUs(),d.pwmPhaseUs(),d.randomSeed()};
        for(int i=0;i<keys.length;i++) if(c.containsKey(keys[i])) {
            Object value=c.get(keys[i],null);
            if(!(value instanceof Number n) || !Double.isFinite(n.doubleValue()) ||
                    n.doubleValue()!=Math.rint(n.doubleValue()) || n.doubleValue()<0 || n.doubleValue()>Integer.MAX_VALUE)
                throw new IllegalArgumentException("Invalid FC timing setting: "+keys[i]);
            values[i]=((Number)value).intValue();
        }
        return new FlightComputerTimingSettings(values[0],values[1],values[2],values[3],values[4]);
    }
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
        configure(enabled,link,output,getTimingSettings());
    }
    public void configure(boolean enabled, TelemetryLinkSettings link, FlightComputerOutputSettings output, FlightComputerTimingSettings timing) {
        Config c = getConfig();
        if(!hasDesignFile()) {
        c.put("sensorReadUs",timing.sensorReadUs()); c.put("extraWorkUs",timing.extraWorkUs());
        c.put("workJitterUs",timing.workJitterUs()); c.put("pwmPhaseUs",timing.pwmPhaseUs());
        c.put("timingSeed",timing.randomSeed());
        }
        c.put("version", 1);
        c.put("enabled", enabled);
        c.put("packetLossFraction", link.packetLossFraction());
        c.put("downlinkDelayMs", link.delayMs());
        c.put("randomSeed", link.randomSeed());
        c.put("csvFile", output.csvFile());
        c.put("logFile", output.logFile());
        setConfig(c);
    }

    @Override public void setConfig(Config c) {
        for(String key:new String[]{"designFile","designId","computer"})if(c.containsKey(key)&&!(c.get(key,null) instanceof String))throw new IllegalArgumentException("Invalid "+key);
        settings(c); outputs(c); timing(c); super.setConfig(c); preparedDesign=null;
    }

    @Override public void initialize(SimulationConditions conditions) throws SimulationException {
        try {
            TelemetryLinkSettings link = getLinkSettings();
            if (isEnabled()) {
                prepareDesign();
                conditions.getSimulationListenerList().add(new FlightControllerSimulatorListener(link, getOutputSettings(), getTimingSettings()).withDesign(preparedDesign));
            }
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
        apply(simulation, enabled, link, output, read(simulation).getTimingSettings());
    }
    public static void apply(Simulation simulation, boolean enabled, TelemetryLinkSettings link,
                             FlightComputerOutputSettings output, FlightComputerTimingSettings timing) {
        ZephyrusFlightComputer fc = (ZephyrusFlightComputer)read(simulation).clone();
        fc.configure(enabled, link, output, timing);
        replace(simulation,fc);
    }
    private static void replace(Simulation simulation,ZephyrusFlightComputer fc) {
        var extensions = new java.util.ArrayList<>(simulation.getSimulationExtensions());
        int index = extensions.size();
        for (int i = 0; i < extensions.size(); i++) if (isFlightComputer(extensions.get(i))) { index = i; break; }
        extensions.removeIf(ZephyrusFlightComputer::isFlightComputer);
        extensions.add(Math.min(index, extensions.size()), fc);
        simulation.copyExtensionsFrom(extensions);
    }
}
