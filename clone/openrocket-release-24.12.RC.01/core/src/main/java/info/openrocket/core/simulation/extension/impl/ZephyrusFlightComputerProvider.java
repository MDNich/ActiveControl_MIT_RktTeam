package info.openrocket.core.simulation.extension.impl;

import info.openrocket.core.plugin.Plugin;
import info.openrocket.core.simulation.extension.AbstractSimulationExtensionProvider;
import java.util.List;

@Plugin
public class ZephyrusFlightComputerProvider extends AbstractSimulationExtensionProvider {
    public ZephyrusFlightComputerProvider() { super(ZephyrusFlightComputer.class, "Zephyrus flight computer"); }
    // Configured by the dedicated box, not a second Add extensions entry.
    @Override public List<String> getName(String id) { return null; }
}
