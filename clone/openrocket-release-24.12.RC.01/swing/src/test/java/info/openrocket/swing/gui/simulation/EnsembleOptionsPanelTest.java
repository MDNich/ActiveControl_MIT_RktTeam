package info.openrocket.swing.gui.simulation;

import info.openrocket.swing.util.EnsembleSwingTestCase;
import info.openrocket.core.document.*;
import info.openrocket.core.util.TestRockets;
import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.nio.file.*;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class EnsembleOptionsPanelTest extends EnsembleSwingTestCase {
    @Test void controlsBindAndFitWithinExistingOptionsPanel() throws Exception {
        SwingUtilities.invokeAndWait(()->{
            try {
                var rocket=TestRockets.makeEstesAlphaIII();var doc=OpenRocketDocumentFactory.createDocumentFromRocket(rocket);
                var simulation=new Simulation(doc,rocket);var panel=new SimulationOptionsPanel(doc,simulation);
                var checkbox=(JCheckBox)find(panel,"ensemble.enabled");assertNotNull(checkbox);checkbox.doClick();
                var thrust=(JSpinner)find(panel,"ensemble.Thrust noise σ (N)");thrust.setValue(12.0);
                var editor=((JSpinner.NumberEditor)thrust.getEditor()).getTextField();
                assertThrows(java.text.ParseException.class,()->editor.getFormatter().stringToValue("-1"));
                var runCount=(JSpinner)find(panel,"ensemble.Number of runs");
                assertThrows(java.text.ParseException.class,()->((JSpinner.NumberEditor)runCount.getEditor()).getTextField().getFormatter().stringToValue("1"));
                assertTrue(simulation.getOptions().getEnsembleSettings().enabled());assertEquals(12,simulation.getOptions().getEnsembleSettings().motorSigma());
                checkbox.doClick();assertFalse(thrust.isEnabled());assertEquals(12,simulation.getOptions().getEnsembleSettings().motorSigma());
                checkbox.doClick();
                panel.setSize(1120,700);layout(panel);
                for(var c:panel.getComponents()) if(c instanceof JScrollPane scroll) {
                    scroll.getViewport().setViewPosition(new Point(0,Math.max(0,scroll.getViewport().getView().getHeight()-scroll.getViewport().getHeight())));
                }
                var image=new BufferedImage(1120,700,BufferedImage.TYPE_INT_RGB);var g=image.createGraphics();panel.printAll(g);g.dispose();
                Path dir=Path.of("build/ensemble-verification");Files.createDirectories(dir);javax.imageio.ImageIO.write(image,"png",dir.resolve("ensemble-options.png").toFile());
            } catch(Exception e) {throw new RuntimeException(e);}
        });
    }
    @Test void progressDialogRunsEnsembleWithoutReadingLockedOptionsOnEdt() throws Exception {
        var r=TestRockets.makeEstesAlphaIII();var doc=OpenRocketDocumentFactory.createDocumentFromRocket(r);
        var simulation=new Simulation(doc,r) {
            @Override public info.openrocket.core.simulation.SimulationOptions getOptions() {
                if (SwingUtilities.isEventDispatchThread() && getEnsembleRunNumber()>0)
                    throw new AssertionError("Progress UI must use its options snapshot while the simulation runs");
                return super.getOptions();
            }
        };
        simulation.setFlightConfigurationId(TestRockets.TEST_FCID_1);
        simulation.getOptions().setMaxSimulationTime(2);simulation.getOptions().setTimeStep(.02);
        simulation.getOptions().setISAAtmosphere(true);simulation.getOptions().setLaunchRodLength(1);
        simulation.getOptions().setEnsembleSettings(new info.openrocket.core.simulation.ensemble.EnsembleSettings(true,2,.1,.05,0,0,0,4));
        java.util.concurrent.atomic.AtomicReference<Throwable> error=new java.util.concurrent.atomic.AtomicReference<>();
        Thread.UncaughtExceptionHandler previous=Thread.getDefaultUncaughtExceptionHandler();
        Thread.setDefaultUncaughtExceptionHandler((thread,failure)->error.compareAndSet(null,failure));
        var dialog=new SimulationRunDialog[1];
        try {
            SwingUtilities.invokeAndWait(()->dialog[0]=new SimulationRunDialog(null,doc,simulation));
            var field=SimulationRunDialog.class.getDeclaredField("simulationWorkers");field.setAccessible(true);
            for(var worker:(SimulationWorker[])field.get(dialog[0])) worker.get(30,java.util.concurrent.TimeUnit.SECONDS);
            SwingUtilities.invokeAndWait(()->{});
            assertNull(error.get());assertNotNull(simulation.getSimulatedData().getEnsembleResult());
        } finally {
            SwingUtilities.invokeAndWait(()->{if(dialog[0]!=null)dialog[0].dispose();});
            Thread.setDefaultUncaughtExceptionHandler(previous);
        }
    }
    private static Component find(Container parent,String name) {
        for(var child:parent.getComponents()) {if(name.equals(child.getName())) return child;if(child instanceof Container c){var found=find(c,name);if(found!=null)return found;}}
        return null;
    }
    private static void layout(Container c) {c.doLayout();for(var child:c.getComponents())if(child instanceof Container nested)layout(nested);}
}
