package info.openrocket.core.simulation.flightcomputer;

import java.io.StringReader;
import java.nio.charset.StandardCharsets;
import java.util.function.*;
import java.util.prefs.Preferences;
import org.codehaus.janino.SimpleCompiler;

/** API for an embedded Java board. A new program instance and class loader are created for each run. */
public abstract class JavaBoardProgram {
    public static final String MODEL_NOTICE="Custom Java boards are for algorithm experiments. Communication between boards is not accurately simulated: boards share measured inputs directly, without bus delays, packet loss or contention. These results are not authoritative predictions of hardware behavior.";
    public abstract void step(Context io);

    public static final class Context {
        private final long timeUs,dtUs;
        private final String state;
        private final ToDoubleFunction<String> signals;
        private final Predicate<String> connected;
        private final BiConsumer<String,Double> output;
        private final Consumer<String> log;
        public Context(long timeUs,long dtUs,String state,ToDoubleFunction<String> signals,
                       Predicate<String> connected,BiConsumer<String,Double> output,Consumer<String> log) {
            this.timeUs=timeUs;this.dtUs=dtUs;this.state=state;this.signals=signals;
            this.connected=connected;this.output=output;this.log=log;
        }
        /** Virtual invocation start; outputs become visible after the board's configured execution cost. */
        public long timeUs(){return timeUs;}
        public long dtUs(){return dtUs;}
        public String stateId(){return state;}
        public boolean connected(String port){return connected.test(port);}
        public double signal(String name){return signals.applyAsDouble(name);}
        public double altitudeM(){return signal("baro_altitude");}
        public double velocityMps(){return signal("velocity");}
        public double accelerationMps2(){return signal("accel_vertical");}
        public void setAirbrakes(double fraction){range(fraction,0,1);output.accept("airbrakes",fraction);}
        public void setRollDegrees(double angle){range(angle,-90,90);output.accept("roll",angle);}
        public void fireRecovery(int channel){range(channel,0,5);output.accept("recovery",(double)channel);}
        public void log(String text){log.accept(text);}
        private static void range(double value,double min,double max){if(!Double.isFinite(value)||value<min||value>max)throw new IllegalArgumentException("Board output must be in ["+min+", "+max+"]");}
    }

    public static String template(){return """
            import info.openrocket.core.simulation.flightcomputer.JavaBoardProgram;

            public class BoardProgram extends JavaBoardProgram {
                private boolean announced;

                public void step(Context io) {
                    // State is kept in fields for this run only.
                    if (!announced) {
                        io.log("Custom board started");
                        announced = true;
                    }
                    // Sensor values are measured FC inputs, not simulator truth.
                    // double altitude = io.altitudeM();
                    // io.setAirbrakes(0.25);  // fraction 0..1
                    // io.setRollDegrees(10); // degrees -90..90
                    // io.fireRecovery(2);   // channels 0..5; logical output only
                }
            }
            """;}
    public static final class Compiled {
        private final Class<? extends JavaBoardProgram> type;
        private Compiled(Class<? extends JavaBoardProgram> type){this.type=type;}
        public JavaBoardProgram create(){
            try{return type.getDeclaredConstructor().newInstance();}
            catch(ReflectiveOperationException | LinkageError e){throw new IllegalArgumentException("Cannot start Java board: "+e.getMessage(),e);}
        }
    }
    /** Compilation and inspection do not initialize the class or execute its constructor. */
    public static Compiled compile(String source) {
        if(source.isBlank()||source.length()>100000)throw new IllegalArgumentException("Java board source must contain 1–100000 characters");
        try{
            var compiler=new SimpleCompiler();compiler.setParentClassLoader(JavaBoardProgram.class.getClassLoader());
            compiler.setDebuggingInformation(true,true,false);compiler.cook("BoardProgram.java",new StringReader(source));
            var type=Class.forName("BoardProgram",false,compiler.getClassLoader()).asSubclass(JavaBoardProgram.class);
            if(java.lang.reflect.Modifier.isAbstract(type.getModifiers()))throw new IllegalArgumentException("BoardProgram must implement step(Context io)");
            type.getConstructor();return new Compiled(type);
        }catch(Exception | LinkageError e){throw new IllegalArgumentException("Java board: "+e.getMessage(),e);}
    }
    private static Preferences approvals(){return Preferences.userNodeForPackage(JavaBoardProgram.class).node("approved-java-board-source-v1");}
    private static String hash(String source){return FlightComputerDesign.hash(source.getBytes(StandardCharsets.UTF_8));}
    public static boolean approved(String source){return approvals().getBoolean(hash(source),false);}
    public static void approve(String source){approvals().putBoolean(hash(source),true);}
    public static void revoke(String source){approvals().remove(hash(source));}
    public static void requireApproved(FlightComputerDesign design){
        for(var n:design.nodes().getValuesAs(jakarta.json.JsonObject.class))if(n.getString("type").equals("java_board")){
            if(!approved(n.getString("program")))throw new IllegalArgumentException("Open Java code for '"+n.getString("name")+"' in the designer and enable this source before running it.");
            compile(n.getString("program"));
        }
    }
}
