package info.openrocket.swing.gui.simulation;

import com.google.inject.*;
import com.google.inject.util.Modules;
import info.openrocket.core.database.motor.ThrustCurveMotorSetDatabase;
import info.openrocket.core.document.*;
import info.openrocket.core.motor.ThrustCurveMotor;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.startup.Application;
import info.openrocket.core.preferences.ApplicationPreferences;
import info.openrocket.core.l10n.*;
import info.openrocket.core.util.TestRockets;
import info.openrocket.swing.gui.util.SwingPreferences;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assumptions.assumeFalse;
import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.nio.file.*;

class MotorBatchDialogTest {
    @Test void dialogOffersFourThreadsLevelsAndCSVAndFitsItsControls() throws Exception {
        assumeFalse(GraphicsEnvironment.isHeadless());
        var database=new ThrustCurveMotorSetDatabase();
        Application.setInjector(Guice.createInjector(Modules.override(new info.openrocket.swing.ServicesForTesting()).with(new AbstractModule(){
            @Override protected void configure(){
                bind(ApplicationPreferences.class).toInstance(new SwingPreferences());
                bind(Translator.class).toInstance(new ResourceBundleTranslator("l10n.messages"));
                bind(ThrustCurveMotorSetDatabase.class).toInstance(database);
            }
        }),new info.openrocket.core.plugin.PluginModule()));
        var rocket=TestRockets.makeEstesAlphaIII();var doc=OpenRocketDocumentFactory.createDocumentFromRocket(rocket);
        var source=new Simulation(doc,rocket);source.setFlightConfigurationId(TestRockets.TEST_FCID_1);source.setName("Motor batch UI verification");doc.addSimulation(source);
        for(var c:rocket)if(c instanceof InnerTube mount && mount.isMotorMount())for(var it=mount.getMotorIterator();it.hasNext();)if(it.next().getMotor() instanceof ThrustCurveMotor motor)database.addMotor(motor);
        SwingUtilities.invokeAndWait(()->{
            var dialog=new MotorBatchDialog(null,doc,source);
            try {
                dialog.setModalityType(java.awt.Dialog.ModalityType.MODELESS);dialog.setVisible(true);dialog.validate();
                JSpinner count=find(dialog,JSpinner.class,null);assertNotNull(count);assertEquals(4,count.getValue());
                assertNotNull(find(dialog,JButton.class,"Export CSV…"));assertNotNull(find(dialog,JButton.class,"Run batch"));
                var table=find(dialog,JTable.class,null);assertTrue(table.getRowCount()>0);assertEquals("NAR/Tripoli level",table.getColumnName(2));
                var override=find(dialog,JCheckBox.class,"Ignore component intersections");assertNotNull(override);assertFalse(override.isSelected());
                int ignored=table.getColumn("Ignored intersections").getModelIndex();
                assertEquals("",table.getModel().getValueAt(0,ignored));
                override.doClick();assertTrue(override.isSelected());
                assertTrue(table.getModel().getValueAt(0,ignored).toString().contains("MOTOR_INTERSECTS:"));
                override.doClick();assertEquals("",table.getModel().getValueAt(0,ignored));
                var image=new BufferedImage(dialog.getWidth(),dialog.getHeight(),BufferedImage.TYPE_INT_RGB);
                var graphics=image.createGraphics();dialog.getRootPane().printAll(graphics);graphics.dispose();
                var destination=Path.of("build/reports/motor-batch-dialog.png");Files.createDirectories(destination.getParent());javax.imageio.ImageIO.write(image,"png",destination.toFile());
            }catch(Exception e){throw new RuntimeException(e);}finally{dialog.dispose();}
        });
    }
    private static <T extends Component> T find(Container parent,Class<T> type,String text){
        for(Component c:parent.getComponents()) {
            if(type.isInstance(c)&&(text==null||c instanceof AbstractButton b&&text.equals(b.getText())))return type.cast(c);
            if(c instanceof Container child){T found=find(child,type,text);if(found!=null)return found;}
        }
        return null;
    }
}
