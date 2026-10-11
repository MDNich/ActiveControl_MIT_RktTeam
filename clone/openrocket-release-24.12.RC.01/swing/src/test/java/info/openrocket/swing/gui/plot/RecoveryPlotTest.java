package info.openrocket.swing.gui.plot;

import info.openrocket.core.document.*;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.flightcomputer.*;
import info.openrocket.core.util.*;
import info.openrocket.swing.util.EnsembleSwingTestCase;
import org.junit.jupiter.api.Test;
import java.nio.file.*;
import java.util.*;
import static org.junit.jupiter.api.Assertions.*;
import static info.openrocket.core.simulation.FlightDataType.*;

class RecoveryPlotTest extends EnsembleSwingTestCase {
    /** Verifies that imported device name and actual deployment drive the3d canopy. */
    @Test
    void importedDeviceNameAndActualDeploymentDriveThe3dCanopy() {
        var branch = new FlightDataBranch("Recovery", TYPE_TIME);
        for (int i = 0; i < 3; i++) {
            branch.addPoint();
            branch.setValue(TYPE_TIME, i);
            branch.setValue(TYPE_POSITION_X, 0);
            branch.setValue(TYPE_POSITION_Y, 0);
            branch.setValue(TYPE_ALTITUDE, 100 - i * 10);
            branch.setValue(TYPE_VELOCITY_X, 0);
            branch.setValue(TYPE_VELOCITY_Y, 0);
            branch.setValue(TYPE_VELOCITY_Z, -10);
            branch.setValue(TYPE_ORIENTATION_QW, 1);
            branch.setValue(TYPE_ORIENTATION_QX, 0);
            branch.setValue(TYPE_ORIENTATION_QY, 0);
            branch.setValue(TYPE_ORIENTATION_QZ, 0);
        }
        var deployment =
                new RecoveryDeployment(
                        UUID.randomUUID(),
                        "Main parachute",
                        2,
                        "java_board:test",
                        500000,
                        500250,
                        1);
        branch.addEvent(
                new FlightEvent(
                        FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT, .50025, null, deployment));
        var trajectory = new TrajectoryData(branch);
        assertFalse(trajectory.at(.5).recovery());
        assertTrue(trajectory.at(.50025).recovery());
        assertEquals("Main parachute", trajectory.markers().get(0).source());
        assertEquals(1, trajectory.at(1).attitude().getW(), 1e-12);
    }

    /**
     * Verifies that digital recovery plots keep original samples and render steps.
     *
     * @throws Exception if the requested operation cannot be completed
     */
    @Test
    void digitalRecoveryPlotsKeepOriginalSamplesAndRenderSteps() throws Exception {
        var id = UUID.randomUUID();
        var deployed = FlightComputerData.recoveryDevice(id.toString(), "Main");
        var branch = new FlightDataBranch("Recovery", TYPE_TIME);
        for (int i = 0; i <= 20; i++) {
            double t = i / 10.;
            branch.addPoint();
            branch.setValue(TYPE_TIME, t);
            branch.setValue(FlightComputerData.RECOVERY_PULSE[2], t >= .5 && t < .8 ? 1 : 0);
            branch.setValue(deployed, t >= .6 ? 1 : 0);
        }
        var data = new FlightData(branch);
        var rocket = TestRockets.makeEstesAlphaIII();
        var simulation =
                new Simulation(
                        null,
                        rocket,
                        Simulation.Status.EXTERNAL,
                        "Recovery timing",
                        new SimulationOptions(),
                        List.of(),
                        data);
        var config = new SimulationPlotConfiguration("Flight computer: recovery timing", TYPE_TIME);
        config.addPlotDataType(FlightComputerData.RECOVERY_PULSE[2], 0);
        config.addPlotDataType(deployed, 0);
        var plot = SimulationPlot.create(simulation, config, false);
        assertEquals(
                21,
                plot.chart.getXYPlot().getDataset(0).getItemCount(0),
                "Do not insert synthetic samples that break tooltips");
        var directory = Path.of("build/fc-recovery-verification");
        Files.createDirectories(directory);
        org.jfree.chart.ChartUtils.saveChartAsPNG(
                directory.resolve("recovery-steps.png").toFile(), plot.chart, 1000, 620);
    }
}
