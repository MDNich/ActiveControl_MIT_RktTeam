package info.openrocket.core.simulation.flightcomputer;

import info.openrocket.core.simulation.*;
import jakarta.json.*;
import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.regex.*;

/** Read-only import of OR action logs. No source code or instructions from a log are executed. */
public final class FlightComputerLogReader {
    private FlightComputerLogReader(){}
    public record Result(String name,FlightData data,String note){}
    private static final Pattern HEADER=Pattern.compile("(?:ZEPHYRUS|FC) run=(\\S+) boot_us=(\\d+) action=(\\S+) ?(.*)");
    private static final Pattern FIELD=Pattern.compile("(?:^|\\s)([A-Za-z0-9_]+)=([^\\s]+)");
    public static List<Result> read(Path path)throws IOException {
        var runs=new LinkedHashMap<String,Run>();var symbols=FlightComputerData.bySymbol();int lineNo=0;
        try(var reader=Files.newBufferedReader(path)){
            String line;while((line=reader.readLine())!=null){
                if(Thread.currentThread().isInterrupted())throw new java.util.concurrent.CancellationException();
                lineNo++;if(line.length()>1_000_000)throw new IOException("Log line "+lineNo+" exceeds 1 MB");
                var match=HEADER.matcher(line);if(!match.find())continue;
                String runId=match.group(1),action=match.group(3),detail=match.group(4);var run=runs.computeIfAbsent(runId,Run::new);
                if(action.equals("fc.plot_metadata")){
                    try(var json=Json.createReader(new StringReader(detail))){var metadata=json.readObject();if(metadata.getInt("version")!=1)throw new IOException("Unsupported FC plot log version");metadata.getJsonObject("states").forEach((key,value)->run.names.put(key,((JsonString)value).getString()));}
                }else if(action.equals("fc.plot_event")){
                    try(var json=Json.createReader(new StringReader(detail))){var event=json.readObject();var type=FlightEvent.Type.valueOf(event.getString("type"));double time=event.getJsonNumber("time").doubleValue();
                        if(!Double.isFinite(time)||run.events.size()>10000)throw new IOException("Invalid or excessive log events");
                        if(EnumSet.of(FlightEvent.Type.LAUNCH,FlightEvent.Type.IGNITION,FlightEvent.Type.LIFTOFF,FlightEvent.Type.BURNOUT,FlightEvent.Type.APOGEE,FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT,FlightEvent.Type.GROUND_HIT,FlightEvent.Type.STAGE_SEPARATION,FlightEvent.Type.TUMBLE).contains(type))run.events.add(new FlightEvent(type,time,null));
                    }
                }else if(action.equals("fc.plot")){
                    if(!run.modern){run.branch=new FlightDataBranch("FC log "+runId,FlightDataType.TYPE_TIME);run.modern=true;}
                    try(var json=Json.createReader(new StringReader(detail))){var record=json.readObject();if(record.getInt("version")!=1)throw new IOException("Unsupported FC plot log version");var values=record.getJsonObject("values");double time=values.getJsonNumber("t").doubleValue();run.point(time);for(var entry:values.entrySet()){var type=symbols.get(entry.getKey());if(type!=null){double value=((JsonNumber)entry.getValue()).doubleValue();if(!Double.isFinite(value))throw new IOException("Nonfinite log sample");run.branch.setValue(type,value);}}}
                }else if(!run.modern){
                    var fields=new HashMap<String,String>();var pairs=FIELD.matcher(detail);while(pairs.find())fields.put(pairs.group(1),pairs.group(2));run.legacy(action,fields);
                }
            }
        }catch(RuntimeException e){throw new IOException("Invalid FC log at line "+lineNo+": "+e.getMessage(),e);}
        var results=new ArrayList<Result>();for(var run:runs.values())if(run.branch.getLength()>1){
            run.events.forEach(run.branch::addEvent);run.branch.immute();var data=new FlightData(run.branch);data.setFlightComputerProvenance(run.names);
            results.add(new Result(path.getFileName()+" · run "+run.id,data,run.modern?"Imported recorded FC plot samples (10 ms nominal spacing).":"Older log: recovered only recorded sensors, outputs, state and vertical motion. Height is relative to the first physical sample. Horizontal trajectory, attitude and unrecorded physical sensor values are unavailable."));
        }
        if(results.isEmpty())throw new IOException("No FC flight samples found. Choose an OR-produced action log containing fc.plot or physics.accept records.");
        return List.copyOf(results);
    }
    private static final class Run {
        final String id;boolean modern;double firstAltitude=Double.NaN;
        final List<FlightEvent> events=new ArrayList<>();
        FlightDataBranch branch;final Map<FlightDataType,Double> held=new HashMap<>();final Map<String,String> names=new HashMap<>(FlightComputerData.stateNames(null));
        Run(String id){this.id=id;branch=new FlightDataBranch("FC log "+id,FlightDataType.TYPE_TIME);}
        void point(double time)throws IOException{
            if(!Double.isFinite(time)||time<0)throw new IOException("Invalid simulation timestamp");
            if(branch.getLength()>0){double previous=branch.getLast(FlightDataType.TYPE_TIME);if(time<previous)throw new IOException("Log timestamps run backwards; import one execution per file");if(time==previous)return;}
            if(branch.getLength()>=2_000_000)throw new IOException("Log exceeds two million samples per run");branch.addPoint();branch.setValue(FlightDataType.TYPE_TIME,time);
        }
        void value(Map<String,String> f,String key,FlightDataType type,double factor,double offset){if(f.containsKey(key))held.put(type,Double.parseDouble(f.get(key))*factor+offset);}
        void legacy(String action,Map<String,String> f)throws IOException{
            switch(action){
                case "baro.update"->{value(f,"filtered_m",FlightComputerData.BARO_ALTITUDE,1,0);value(f,"pressure_hPa",FlightComputerData.PRESSURE.measured(),100,0);value(f,"temperature_C",FlightComputerData.TEMPERATURE.measured(),1,273.15);}
                case "accel.update"->{value(f,"velocity",FlightComputerData.VELOCITY,1,0);for(int i=0;i<3;i++)value(f,"xyz".substring(i,i+1),FlightComputerData.ACCEL[i].measured(),1,0);}
                case "gyro.update"->{for(int i=0;i<3;i++)value(f,"xyz".substring(i,i+1)+"_dps",FlightComputerData.GYRO[i].measured(),Math.PI/180,0);}
                case "gps.update"->{value(f,"relative_m",FlightComputerData.GPS_ALTITUDE,1,0);value(f,"lat_deg",FlightComputerData.LATITUDE.measured(),Math.PI/180,0);value(f,"lon_deg",FlightComputerData.LONGITUDE.measured(),Math.PI/180,0);}
                case "pwm.latch"->{value(f,"exposed",FlightComputerData.AIRBRAKE,1,0);value(f,"angle_deg",FlightComputerData.AIRBRAKE_ANGLE,Math.PI/180,0);value(f,"roll_deg",FlightComputerData.ROLL_ANGLE,Math.PI/180,0);value(f,"pulse_us",FlightComputerData.PWM[0],1e-6,0);value(f,"servo2_us",FlightComputerData.PWM[1],1e-6,0);value(f,"servo3_us",FlightComputerData.PWM[2],1e-6,0);}
                case "fc.transition"->{String state=f.get("to");int code=List.of("GROUND_TESTING","PRE_FLIGHT","FLIGHT","APOGEE","MAIN","END").indexOf(state);if(code>=0)held.put(FlightComputerData.STATE,(double)code);}
                case "physics.accept"->{
                    if(!f.containsKey("simulation_s"))return;point(Double.parseDouble(f.get("simulation_s")));
                    if(f.containsKey("truth_altitude_m")){double altitude=Double.parseDouble(f.get("truth_altitude_m"));if(!Double.isFinite(firstAltitude))firstAltitude=altitude;branch.setValue(FlightDataType.TYPE_ALTITUDE_ABOVE_SEA,altitude);branch.setValue(FlightDataType.TYPE_ALTITUDE,altitude-firstAltitude);}
                    if(f.containsKey("truth_velocity_mps"))branch.setValue(FlightDataType.TYPE_VELOCITY_Z,Double.parseDouble(f.get("truth_velocity_mps")));
                    held.forEach(branch::setValue);
                }
            }
        }
    }
}
