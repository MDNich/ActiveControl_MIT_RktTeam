package info.openrocket.swing.gui.flightcomputer;
import info.openrocket.core.simulation.flightcomputer.*;
import info.openrocket.swing.util.EnsembleSwingTestCase;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.nio.file.*;
import static org.junit.jupiter.api.Assertions.*;
class FlightComputerDesignerTest extends EnsembleSwingTestCase {
    @TempDir Path directory;
    @Test void editorLoadsEditsAndRendersAllThreeViews()throws Exception{
        Path file=directory.resolve("zephyrus.fc");FlightComputerDesign.zephyrus().write(file);
        SwingUtilities.invokeAndWait(()->{try{
            var editor=new FlightComputerDesigner(file,null,p->{});editor.setSize(1280,850);layout(editor);
            assertTrue(editor.getDesign().diagnostics().stream().noneMatch(FlightComputerDesign.Diagnostic::error));
            int before=editor.getDesign().nodes().size();editor.addNode("board",800,400);assertEquals(before+1,editor.getDesign().nodes().size());
            Path output=Path.of("build/fc-designer-verification");Files.createDirectories(output);
            var tabs=findTabs(editor);assertNotNull(tabs);assertEquals(3,tabs.getTabCount());
            for(int i=0;i<3;i++){tabs.setSelectedIndex(i);layout(editor);layout(editor);editor.fitView();var image=new BufferedImage(1280,850,BufferedImage.TYPE_INT_RGB);var g=image.createGraphics();editor.printAll(g);g.dispose();javax.imageio.ImageIO.write(image,"png",output.resolve("designer-"+i+".png").toFile());}
        }catch(Exception e){throw new RuntimeException(e);}});
    }
    @Test void enterAppliesPropertiesTreeUsesRocketStyleAndIntermediateStatesPersist()throws Exception{
        Path file=directory.resolve("editing.fc");FlightComputerDesign.zephyrus().write(file);
        SwingUtilities.invokeAndWait(()->{try{
            var editor=new FlightComputerDesigner(file,null,p->{});editor.setSize(1280,850);layout(editor);layout(editor);
            var name=(JTextField)find(editor,"fc.designName");assertNotNull(name);name.setText("Custom flight computer");name.postActionEvent();assertEquals("Custom flight computer",editor.getDesign().name());
            assertNotNull(editor.getInputMap(JComponent.WHEN_ANCESTOR_OF_FOCUSED_COMPONENT).get(KeyStroke.getKeyStroke("meta S")));
            var tree=(JTree)find(editor,"fc.designer.tree");assertInstanceOf(info.openrocket.swing.gui.components.BasicTree.class,tree);assertNotNull(((JLabel)tree.getCellRenderer().getTreeCellRendererComponent(tree,tree.getModel().getRoot(),false,true,false,0,false)).getIcon());
            editor.addNode("java_board",800,400);var component=(JTextField)find(editor,"fc.componentName");component.setText("Guidance board");component.postActionEvent();assertTrue(editor.getDesign().nodes().getValuesAs(jakarta.json.JsonObject.class).stream().anyMatch(n->n.getString("name").equals("Guidance board")&&n.getString("program","").contains("step(Context io)")));
            editor.insertIntermediateState("apogee","Drogue descent",2000,jakarta.json.Json.createArrayBuilder().add(jakarta.json.Json.createObjectBuilder().add("type","fire_recovery").add("channel",3)).build());editor.getDesign().requireRunnable();assertEquals(6,FlightComputerStateMachine.states(editor.getDesign().stateMachine()).size());
            editor.getDesign().write(file);assertEquals(editor.getDesign().json(),FlightComputerDesign.read(file).json());
        }catch(Exception ex){throw new RuntimeException(ex);}});
    }
    private static Component find(Container c,String name){for(var child:c.getComponents()){if(name.equals(child.getName()))return child;if(child instanceof Container nested){var found=find(nested,name);if(found!=null)return found;}}return null;}
    @Test void pidFieldsRetainPrecisionAndEnterAppliesAllSixGains()throws Exception{
        Path file=directory.resolve("pid.fc");FlightComputerDesign.zephyrus().write(file);
        SwingUtilities.invokeAndWait(()->{try{
            var editor=new FlightComputerDesigner(file,null,p->{});editor.setSize(1280,850);layout(editor);findTabs(editor).setSelectedIndex(1);layout(editor);
            var kp=(JSpinner)find(editor,"fc.parameter.rollKp");assertNotNull(kp);
            assertEquals(.08444,((Number)kp.getValue()).doubleValue(),1e-10);
            var numberFormat=((JSpinner.NumberEditor)kp.getEditor()).getFormat();assertEquals(.08444,numberFormat.parse(numberFormat.format(kp.getValue())).doubleValue(),1e-10);
            for(String key:java.util.List.of("airbrakeKp","airbrakeKi","airbrakeKd","rollKp","rollKi","rollKd"))((JSpinner)find(editor,"fc.parameter."+key)).setValue(.123456);
            ((JSpinner.NumberEditor)kp.getEditor()).getTextField().postActionEvent();
            editor.getDesign().write(file);var saved=FlightComputerDesign.read(file);
            for(String key:java.util.List.of("airbrakeKp","airbrakeKi","airbrakeKd","rollKp","rollKi","rollKd"))assertEquals(.123456,saved.parameter(key),1e-10);
        }catch(Exception ex){throw new RuntimeException(ex);}});
    }
    @Test void replayRejectsFutureOrderAndFilteredTelemetryHeaders()throws Exception{
        Path file=directory.resolve("input.csv");Files.writeString(file,"timestamp,altitude\n0,0\n");assertThrows(java.io.IOException.class,()->FlightComputerReplay.read(file));
        String header="time_us,ax,ay,az,pressure_hpa,temperature_c,gx,gy,gz,latitude,longitude,gps_altitude_m\n";
        Files.writeString(file,header+"0,9.8065,0,0,1013.25,20,0,0,0,42,-71,0\n10000,10,0,0,1013,20,0,0,0,42,-71,1\n");assertEquals(2,FlightComputerReplay.read(file).size());
        Files.writeString(file,header+"10000,9.8,0,0,1013,20,0,0,0,0,0,0\n");assertThrows(java.io.IOException.class,()->FlightComputerReplay.read(file));
    }
    private static void layout(Container c){c.doLayout();for(var child:c.getComponents())if(child instanceof Container nested)layout(nested);}
    private static JTabbedPane findTabs(Container c){for(var child:c.getComponents()){if(child instanceof JTabbedPane tab)return tab;if(child instanceof Container nested){var found=findTabs(nested);if(found!=null)return found;}}return null;}
}
