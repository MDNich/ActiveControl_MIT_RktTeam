import com.google.inject.*;
import com.google.inject.util.Modules;
import info.openrocket.core.startup.Application;
import info.openrocket.core.plugin.PluginModule;
import info.openrocket.core.database.motor.*;
import info.openrocket.swing.startup.GuiModule;
import info.openrocket.swing.gui.util.GUIUtil;
import info.openrocket.swing.gui.plot.*;
import info.openrocket.core.simulation.*;
import info.openrocket.core.util.*;
import static info.openrocket.core.simulation.FlightDataType.*;
import javax.swing.*;
import java.nio.file.*;
import java.util.concurrent.*;

/** Standalone integration/performance fixture for the actual GL panel; no simulated user input. */
public class TrajectoryRenderProbe {
    public static void main(String[] args) throws Exception {
        Path output=Path.of(args[0]); Files.createDirectories(output);
        System.setProperty("openrocket.bypass.presets","true");
        System.setProperty("openrocket.bypass.motors","true");
        GuiModule module=new GuiModule();
        Application.setInjector(Guice.createInjector(Modules.override(module).with(new AbstractModule() {
            @Override protected void configure() {
                var motors=new ThrustCurveMotorSetDatabase();
                bind(MotorDatabase.class).toInstance(motors); bind(ThrustCurveMotorSetDatabase.class).toInstance(motors);
            }
        }),new PluginModule()));
        module.startLoader();
        FlightDataBranch branch=new FlightDataBranch("100,000-sample render fixture",TYPE_TIME);
        for(int i=0;i<100000;i++) {
            double t=i/1000.0, a=t*Math.PI/100;
            branch.addPoint(); branch.setValue(TYPE_TIME,t);
            branch.setValue(TYPE_POSITION_X,10*t); branch.setValue(TYPE_POSITION_Y,300*Math.sin(a)); branch.setValue(TYPE_ALTITUDE,1800*Math.sin(a));
            branch.setValue(TYPE_VELOCITY_X,10); branch.setValue(TYPE_VELOCITY_Y,3*Math.PI*Math.cos(a)); branch.setValue(TYPE_VELOCITY_Z,18*Math.PI*Math.cos(a));
            var q=Quaternion.rotation(new Coordinate(.25,.5,0)).multiplyRight(Quaternion.rotation(new Coordinate(0,0,t*.4)));
            branch.setValue(TYPE_ORIENTATION_QW,q.getW()); branch.setValue(TYPE_ORIENTATION_QX,q.getX()); branch.setValue(TYPE_ORIENTATION_QY,q.getY()); branch.setValue(TYPE_ORIENTATION_QZ,q.getZ());
        }
        branch.addEvent(new FlightEvent(FlightEvent.Type.APOGEE,50,null));
        branch.addEvent(new FlightEvent(FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT,55,null));
        long adapterStart=System.nanoTime(); TrajectoryData data=new TrajectoryData(branch);
        System.out.println("adapter_ms="+(System.nanoTime()-adapterStart)/1e6);
        CountDownLatch finished=new CountDownLatch(1);
        SwingUtilities.invokeLater(()-> {
            GUIUtil.getUITheme().applyTheme();
            JFrame window=new JFrame("Trajectory renderer verification");
            Trajectory3DPanel panel=new Trajectory3DPanel(message->{System.err.println(message); finished.countDown();});
            window.setContentPane(panel); window.pack(); window.setLocationByPlatform(true); window.setVisible(true); panel.setData(data);
            long[] begin={0}; int[] draws={0};
            Timer timer=new Timer(1,event->{
                if(!panel.isAvailable()) { ((Timer)event.getSource()).stop(); window.dispose(); finished.countDown(); return; }
                if(draws[0]==20) begin[0]=System.nanoTime();
                panel.setTime(10+(draws[0]%180)*.4);
                if(++draws[0]==220) {
                    ((Timer)event.getSource()).stop();
                    System.out.println("samples="+data.samples().size()+" measured_frames=200 fps="+200e9/(System.nanoTime()-begin[0])+" mean_draw_ms="+panel.getMeanDrawMillis());
                    panel.setTime(65);
                    panel.snapshot(image->{
                        try { javax.imageio.ImageIO.write(image,"png",output.resolve("trajectory-render-probe.png").toFile()); }
                        catch(Exception failure) { failure.printStackTrace(); }
                        panel.close(); window.dispose(); finished.countDown();
                    });
                }
            }); timer.start();
        });
        if(!finished.await(45,TimeUnit.SECONDS)) throw new IllegalStateException("Renderer did not finish");
        if(!Files.exists(output.resolve("trajectory-render-probe.png"))) System.exit(1);
        System.exit(0);
    }
}
