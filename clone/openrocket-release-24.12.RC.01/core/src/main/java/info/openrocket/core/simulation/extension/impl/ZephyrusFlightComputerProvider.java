package info.openrocket.core.simulation.extension.impl;

import info.openrocket.core.plugin.Plugin;
import info.openrocket.core.simulation.extension.AbstractSimulationExtensionProvider;
import java.util.List;

@Plugin
public class ZephyrusFlightComputerProvider extends AbstractSimulationExtensionProvider {
    /**
     * Creates a ZephyrusFlightComputerProvider with the supplied dependencies and per-instance
     * state.
     */
    public ZephyrusFlightComputerProvider() {
        super(ZephyrusFlightComputer.class, "Zephyrus flight computer");
    }

    // Configured by the dedicated box, not a second Add extensions entry.
    /**
     * Returns the extension-provider display name for the FC simulation option.
     *
     * @param id stable identifier
     * @return the extension-provider display name for the FC simulation option
     */
    @Override
    public List<String> getName(String id) {
        return null;
    }
}
