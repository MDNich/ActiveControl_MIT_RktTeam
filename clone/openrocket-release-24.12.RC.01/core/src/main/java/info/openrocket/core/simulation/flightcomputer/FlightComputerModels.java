package info.openrocket.core.simulation.flightcomputer;
import jakarta.json.*;
import java.util.*;

/** Supported properties are listed here and consumed by the runtime, rather than arbitrary cosmetic fields. */
public final class FlightComputerModels {
    private FlightComputerModels(){}
    public record Property(String key,String label,double value,double min,double max,boolean integer){}
    private static Property p(String k,String label,double v,double min,double max,boolean integer){return new Property(k,label,v,min,max,integer);}
    public static final Map<String,Property> PARAMETERS;
    static {
        var map=new LinkedHashMap<String,Property>();
        for(var p:List.of(p("sensorReadUs","Sensor read duration (µs)",3000,0,1000000,true),p("extraWorkUs","Extra work (µs)",0,0,1000000,true),
                p("workJitterUs","Added work jitter, uniform 0–value (µs)",0,0,1000000,true),p("pwmPhaseUs","PWM phase (µs)",0,0,19999,true),
                p("timingSeed","Timing random seed",1,0,Integer.MAX_VALUE,true),p("sensorSeed","Sensor random seed",1,0,Integer.MAX_VALUE,true),
                p("measurementNoisePercent","Measurement Gaussian noise σ (% of reading)",0,0,100,false),
                p("telemetryIntervalMs","Telemetry strict elapsed threshold (ms)",50,1,10000,true),p("powerIntervalMs","Power command strict threshold (ms)",100,1,10000,true),
                p("apogeeLockoutMs","Apogee peak reset / detection lockout (ms)",26000,0,600000,true),
                p("recovery1Ms","Secondary recovery delay (ms)",3000,0,600000,true),p("recovery2Ms","Tertiary recovery delay (ms)",5000,0,600000,true),
                p("airbrakeLimit","Maximum requested airbrake fraction",1,0,1,false),
                p("airbrakeKp","Airbrakes Kp (before gain scaling)",1,-1000000,1000000,false),
                p("airbrakeKi","Airbrakes Ki (before gain scaling)",2,-1000000,1000000,false),
                p("airbrakeKd","Airbrakes Kd (before gain scaling)",0,-1000000,1000000,false),
                p("rollKp","Roll Kp",0.08444,-1000000,1000000,false),
                p("rollKi","Roll Ki",0,-1000000,1000000,false),
                p("rollKd","Roll Kd",0.02111,-1000000,1000000,false)))map.put(p.key(),p);
        PARAMETERS=Collections.unmodifiableMap(map);
    }
    public static final List<String> TYPES=List.of("board","java_board","processor","barometer","accelerometer","gyroscope","gps","radio","storage","power","airbrakes","roll","recovery","connector");
    public static final List<String> REQUIRED_PORTS=List.of("barometer","accelerometer","gyroscope","gps","radio","storage","power","airbrakes","roll","recovery");
    public static final Map<String,String> RULES;
    static {
        var m=new LinkedHashMap<String,String>();
        m.put("preflight","Ground → Preflight");m.put("launch","Preflight → Flight");m.put("apogee","Flight → Apogee (after lockout)");
        m.put("apogeeOverride","Flight → Apogee (override)");m.put("main","Apogee → Main");m.put("end","Main → Ground");
        RULES=Collections.unmodifiableMap(m);
    }
    public static boolean isBoard(String type){return type.equals("board")||type.equals("java_board");}
    public static boolean isControllerHost(String type){return type.equals("processor")||type.equals("java_board");}
    public static String label(String type){return type.equals("java_board")?"Custom Java board":type.equals("gps")?"GPS":Character.toUpperCase(type.charAt(0))+type.substring(1);}
    public static String portType(String port){return REQUIRED_PORTS.contains(port)?port:"invalid";}
    public static List<Property> properties(String type) {
        var list=new ArrayList<Property>();
        if(Set.of("barometer","accelerometer","gyroscope","gps").contains(type)) {
            list.add(p("periodUs",type.equals("gps")?"GPS fix period (µs)":"Minimum sample interval (µs; 0 = each loop)",type.equals("gps")?100000:0,type.equals("gps")?1000:0,1000000,true));
            list.add(p("noise","Gaussian noise σ (channel units)",0,0,1000,false));
            list.add(p("noisePercent","Additional Gaussian noise σ (% of reading)",0,0,100,false));
            list.add(p("bias","Bias (pressure hPa / axis units / GPS altitude m)",0,-1000,1000,false));
            if(type.equals("accelerometer")||type.equals("gyroscope")) for(String axis:List.of("X","Y","Z"))
                list.add(p("rotation"+axis,"Mounting rotation "+axis+" (deg)",0,-180,180,false));
        }
        if(type.equals("barometer"))list.add(p("filterSamples","Altitude moving-average samples",20,1,1000,true));
        if(type.equals("java_board")){
            list.add(p("periodUs","Independent program period (µs)",10000,1000,10000000,true));
            list.add(p("executionUs","Assumed execution cost per call (µs)",0,0,1000000,true));
        }
        return list;
    }
    public static JsonObject newNode(String type,String board,double x,double y) {
        var props=Json.createObjectBuilder(); properties(type).forEach(p->props.add(p.key(),p.value()));
        var node=Json.createObjectBuilder().add("id",UUID.randomUUID().toString()).add("type",type).add("name",label(type))
            .add("board",board).add("properties",props).add("layout",Json.createObjectBuilder().add("x",x).add("y",y).add("collapsed",false));
        if(type.equals("java_board"))node.add("program",JavaBoardProgram.template());
        return node.build();
    }
    public static String capability(String type){return switch(type){case "board","connector"->"Architecture";case "processor"->"Flight controller task host";case "java_board"->"Programmable Java board";case "power","roll","recovery","storage"->"Recorded only";default->"Simulated";};}
}
