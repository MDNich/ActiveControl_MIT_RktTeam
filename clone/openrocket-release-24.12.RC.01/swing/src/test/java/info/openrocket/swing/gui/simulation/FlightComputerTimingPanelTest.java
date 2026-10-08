package info.openrocket.swing.gui.simulation;

import edu.mit.rocket_team.zephyrus.FC.FlightComputerTimingSettings;
import info.openrocket.core.document.*;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.util.TestRockets;
import info.openrocket.swing.util.EnsembleSwingTestCase;
import org.junit.jupiter.api.Test;
import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.nio.file.*;
import static org.junit.jupiter.api.Assertions.*;

class FlightComputerTimingPanelTest extends EnsembleSwingTestCase {
    @Test void timingControlsPersistValidateAndKeepAlignedEditors() throws Exception {
        SwingUtilities.invokeAndWait(()->{
            try {
                var rocket=TestRockets.makeEstesAlphaIII();
                var document=OpenRocketDocumentFactory.createDocumentFromRocket(rocket);
                var simulation=new Simulation(document,rocket);
                var panel=new FlightComputerPanel(simulation,()->{});
                assertEquals(FlightComputerTimingSettings.DEFAULT,ZephyrusFlightComputer.read(simulation).getTimingSettings());
                var enabled=(JCheckBox)find(panel,"fc.enabled"); enabled.doClick();
                var work=(JSpinner)find(panel,"fc.work"); work.setValue(8.25);
                var jitter=(JSpinner)find(panel,"fc.jitter"); jitter.setValue(1.125);
                var phase=(JSpinner)find(panel,"fc.phase"); phase.setValue(6.371);
                var timingSeed=(JSpinner)find(panel,"fc.timingSeed"); timingSeed.setValue(2147483647);
                var expected=new FlightComputerTimingSettings(3000,8250,1125,6371,Integer.MAX_VALUE);
                assertEquals(expected,ZephyrusFlightComputer.read(simulation).getTimingSettings());
                var editor=((JSpinner.NumberEditor)phase.getEditor()).getTextField();
                assertThrows(java.text.ParseException.class,()->editor.getFormatter().stringToValue("20"));
                assertThrows(java.text.ParseException.class,()->editor.getFormatter().stringToValue("-0.1"));
                enabled.doClick(); assertFalse(work.isEnabled());
                assertEquals(expected,ZephyrusFlightComputer.read(simulation).getTimingSettings());
                enabled.doClick();
                panel.setSize(670,panel.getPreferredSize().height); layout(panel);
                int right=-1;
                for(String key:new String[]{"loss","delay","seed","sensorRead","work","jitter","phase","timingSeed"}) {
                    var spinner=(JSpinner)find(panel,"fc."+key);
                    int edge=spinner.getX()+spinner.getWidth();
                    if(right>=0) assertEquals(right,edge); right=edge;
                    assertTrue(edge<panel.getWidth());
                }
                var image=new BufferedImage(panel.getWidth(),panel.getHeight(),BufferedImage.TYPE_INT_RGB);
                var graphics=image.createGraphics(); panel.printAll(graphics); graphics.dispose();
                Path directory=Path.of("build/fc-timing-verification"); Files.createDirectories(directory);
                javax.imageio.ImageIO.write(image,"png",directory.resolve("timing-controls.png").toFile());
            } catch(Exception e) {throw new RuntimeException(e);}
        });
    }
    private static Component find(Container parent,String name) {
        for(var child:parent.getComponents()) {if(name.equals(child.getName()))return child;if(child instanceof Container nested){var result=find(nested,name);if(result!=null)return result;}}
        return null;
    }
    private static void layout(Container container) {container.doLayout();for(var child:container.getComponents())if(child instanceof Container nested)layout(nested);}
}
