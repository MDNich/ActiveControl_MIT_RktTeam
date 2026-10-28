package info.openrocket.core.simulation.mission;

import edu.mit.rocket_team.FC.RTFC;
import info.openrocket.core.simulation.SimulationStatus;
import java.nio.file.Path;

/** Per-run adapter at accepted FC scheduler boundaries, never at trial RK stages. */
public interface MissionFlightHook {
    Path outputDirectory();
    void start(RTFC computer, SimulationStatus status);
    long nextMissionDeadlineUs();
    void before(long bootUs);
    void after(long bootUs);
    void accepted(SimulationStatus status);
    void diagnostic(String line);
    void end(SimulationStatus status, boolean complete);
}
