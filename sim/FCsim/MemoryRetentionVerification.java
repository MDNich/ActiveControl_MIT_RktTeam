import com.google.inject.*;
import com.google.inject.util.Modules;
import info.openrocket.core.database.MotorDatabaseLoader;
import info.openrocket.core.database.motor.*;
import info.openrocket.core.document.*;
import info.openrocket.core.file.*;
import info.openrocket.core.file.motor.GeneralMotorLoader;
import info.openrocket.core.motor.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.motorbatch.*;
import info.openrocket.core.startup.Application;
import info.openrocket.core.plugin.PluginModule;
import info.openrocket.swing.startup.GuiModule;
import info.openrocket.swing.gui.dialogs.motor.thrustcurve.PreferredMotorCurve;
import java.nio.file.*;
import java.util.*;

/** Measure repeated simulations without changing the source rocket file. */
public class MemoryRetentionVerification {
    public static void main(String[] args)throws Exception {
        if(args.length<2)throw new IllegalArgumentException("Usage: ORK OUTPUT_DIR [SIMULATION_INDEX] [REPEATS]");
        System.setProperty("openrocket.bypass.presets","true");System.setProperty("openrocket.bypass.motors","true");
        if(Boolean.getBoolean("openrocket.memory.verifyPolicy"))info.openrocket.swing.startup.StartupMemoryPolicy.configure();
        var diagnostics=System.out;
        System.setProperty("openrocket.fc.telemetryDir",Path.of(args[1]).toAbsolutePath().resolve("telemetry").toString());
        Files.createDirectories(Path.of(args[1]));
        System.setOut(new java.io.PrintStream(java.io.OutputStream.nullOutputStream()));
        ((ch.qos.logback.classic.Logger)org.slf4j.LoggerFactory.getLogger(org.slf4j.Logger.ROOT_LOGGER_NAME)).setLevel(ch.qos.logback.classic.Level.WARN);
        var memory=java.lang.management.ManagementFactory.getMemoryMXBean();
        var history=new ArrayList<String>();
        java.util.function.Consumer<String> sample=label->{var m=memory.getHeapMemoryUsage();String line=label+",used_bytes="+m.getUsed()+",committed_bytes="+m.getCommitted();history.add(line);diagnostics.println(line);};
        sample.accept("startup");
        var loader=new MotorDatabaseLoader();var module=new GuiModule();
        Application.setInjector(Guice.createInjector(Modules.override(module).with(new AbstractModule(){
            @Override protected void configure(){bind(ThrustCurveMotorSetDatabase.class).toProvider(loader::getDatabase).in(Scopes.SINGLETON);bind(MotorDatabase.class).toProvider(loader::getDatabase).in(Scopes.SINGLETON);}
        }),new PluginModule()));
        module.startLoader();loader.startLoading();loader.blockUntilLoaded();var db=loader.getDatabase();
        var file=new GeneralRocketLoader(Path.of(args[0]).toFile());var doc=file.load();
        boolean explicitGC=args.length<5 || !args[4].equals("no-gc");
        sample.accept("loaded");if(explicitGC){System.gc();Thread.sleep(500);sample.accept("loaded_after_gc");}
        diagnostics.println("PID="+ProcessHandle.current().pid()+" warnings="+file.getWarnings());
        for(var sim:doc.getSimulations()) {var data=sim.getSimulatedData();diagnostics.println("SIM="+sim.getName()+" points="+(data==null||data.getBranchCount()==0?0:data.getBranch(0).getLength())+" channels="+(data==null||data.getBranchCount()==0?0:data.getBranch(0).getTypes().length));}
        var old=new ArrayList<java.lang.ref.WeakReference<info.openrocket.core.simulation.FlightData>>();
        int index=args.length>2?Integer.parseInt(args[2]):0,count=args.length>3?Integer.parseInt(args[3]):3;
        var source=doc.getSimulation(index);
        for(int i=0;i<count;i++) {
            old.add(new java.lang.ref.WeakReference<>(source.getSimulatedData()));
            source.simulate();sample.accept("run_"+i);if(explicitGC){System.gc();Thread.sleep(500);sample.accept("run_"+i+"_after_gc");}
            diagnostics.println("old_results_retained="+old.stream().filter(r->r.get()!=null).count()+" points="+source.getSimulatedData().getBranch(0).getLength());
        }
        diagnostics.println("IDLE_FOR_INSPECTION");
        for(int i=0;i<9;i++){Thread.sleep(5000);sample.accept("idle_"+((i+1)*5)+"s");diagnostics.println("old_results_retained="+old.stream().filter(r->r.get()!=null).count());}
        Files.write(Path.of(args[1]).resolve("heap-samples.txt"),history);
        System.exit(0);
    }
}
