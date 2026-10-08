package info.openrocket.core.simulation.flightcomputer;

import edu.mit.rocket_team.zephyrus.FC.RTFC;
import info.openrocket.core.simulation.*;
import info.openrocket.core.unit.UnitGroup;
import jakarta.json.*;
import java.util.*;
import static info.openrocket.core.unit.UnitGroup.*;

/** FC plot channels are actual held readings and latched outputs, sampled on accepted simulation points. */
public final class FlightComputerData {
    private FlightComputerData(){}
    private static final List<FlightDataType> TYPES=new ArrayList<>();
    private static FlightDataType type(String name,String symbol,UnitGroup unit){var t=FlightDataType.getType("FC: "+name,"FC_"+symbol,unit);TYPES.add(t);return t;}
    public static final FlightDataType STATE=type("state code","state",UNITS_NONE);
    public static final FlightDataType AIRBRAKE=type("airbrake exposed fraction","airbrake",UNITS_COEFFICIENT);
    public static final FlightDataType AIRBRAKE_ANGLE=type("airbrake servo angle","air_angle",UNITS_ANGLE);
    public static final FlightDataType ROLL_ANGLE=type("roll servo angle","roll_angle",UNITS_ANGLE);
    public static final FlightDataType[] PWM={type("airbrake PWM pulse","pwm0",UNITS_TIME_STEP),type("roll servo 2 PWM pulse","pwm2",UNITS_TIME_STEP),type("roll servo 3 PWM pulse","pwm3",UNITS_TIME_STEP)};
    public static final FlightDataType BARO_ALTITUDE=type("measured barometric height (filtered and zeroed)","baro_h",UNITS_DISTANCE);
    public static final FlightDataType GPS_ALTITUDE=type("measured GPS height (zeroed)","gps_h",UNITS_DISTANCE);
    public static final FlightDataType VELOCITY=type("estimated velocity (integrated sensor X)","velocity",UNITS_VELOCITY);
    public record Pair(String label,FlightDataType measured,FlightDataType truth){}
    private static Pair pair(String label,String symbol,UnitGroup unit){return new Pair(label,type("measured "+label,symbol+"_measured",unit),type("physical "+label,symbol+"_truth",unit));}
    public static final Pair PRESSURE=pair("pressure","pressure",UNITS_PRESSURE),TEMPERATURE=pair("temperature","temperature",UNITS_TEMPERATURE);
    public static final Pair LATITUDE=pair("GPS latitude","latitude",UNITS_ANGLE),LONGITUDE=pair("GPS longitude","longitude",UNITS_ANGLE);
    public static final Pair[] ACCEL={pair("accelerometer X","ax",UNITS_ACCELERATION),pair("accelerometer Y","ay",UNITS_ACCELERATION),pair("accelerometer Z","az",UNITS_ACCELERATION)};
    public static final Pair[] GYRO={pair("gyroscope X","gx",UNITS_ROLL),pair("gyroscope Y","gy",UNITS_ROLL),pair("gyroscope Z","gz",UNITS_ROLL)};
    public static final List<Pair> PAIRS;
    public static final FlightDataType[] ALL_TYPES;
    static{var pairs=new ArrayList<Pair>();pairs.add(PRESSURE);pairs.add(TEMPERATURE);pairs.add(LATITUDE);pairs.add(LONGITUDE);pairs.addAll(List.of(ACCEL));pairs.addAll(List.of(GYRO));PAIRS=List.copyOf(pairs);ALL_TYPES=TYPES.toArray(FlightDataType[]::new);}
    public static int stateCode(FlightComputerDesign design,String id,int phase){
        if(design==null||!design.hasStateMachine())return phase;
        int builtin=FlightComputerStateMachine.PHASES.indexOf(id);if(builtin>=0)return builtin;
        return 100+FlightComputerStateMachine.states(design.stateMachine()).stream().map(s->s.getString("id")).sorted().toList().indexOf(id);
    }
    public static Map<String,String> stateNames(FlightComputerDesign design){
        var names=new LinkedHashMap<String,String>();String[] defaults={"GROUND","PREFLIGHT","FLIGHT","APOGEE","MAIN","END"};
        for(int i=0;i<defaults.length;i++)names.put("state"+i,defaults[i]);names.put("stateMixed","MIXED STATES");
        if(design!=null&&design.hasStateMachine())for(var state:FlightComputerStateMachine.states(design.stateMachine()))names.put("state"+stateCode(design,state.getString("id"),0),state.getString("name"));
        return names;
    }
    public static void record(FlightDataBranch branch,RTFC fc,RTFC.Inputs truth,FlightComputerDesign design){
        var out=fc.getOutput();branch.setValue(STATE,stateCode(design,fc.getDesignStateId(),fc.getState().ID));
        branch.setValue(AIRBRAKE,out.exposedFraction());branch.setValue(AIRBRAKE_ANGLE,Math.toRadians(out.selectedAngle()));branch.setValue(ROLL_ANGLE,Math.toRadians(out.rollAngle()));
        branch.setValue(PWM[0],out.airbrakePulseUs()/1e6);branch.setValue(PWM[1],out.servo2Us()/1e6);branch.setValue(PWM[2],out.servo3Us()/1e6);
        branch.setValue(BARO_ALTITUDE,fc.baro.getFilteredAltitude());branch.setValue(GPS_ALTITUDE,fc.gps.getHeight());branch.setValue(VELOCITY,fc.accel.getIntegratedVelo());
        pair(branch,PRESSURE,fc.baro.getPressure()*100,truth.baro().getPressure()*100);pair(branch,TEMPERATURE,fc.baro.getTemperature()+273.15,truth.baro().getTemperature()+273.15);
        pair(branch,LATITUDE,Math.toRadians(fc.gps.getLatitude()),Math.toRadians(truth.gps().getLatitude()));pair(branch,LONGITUDE,Math.toRadians(fc.gps.getLongitude()),Math.toRadians(truth.gps().getLongitude()));
        var a=truth.accel();var g=truth.gyro();
        double[] av=FlightComputerSensors.rotate(design==null?null:design.active("accelerometer"),a.getAccelX(),a.getAccelY(),a.getAccelZ());
        double[] gv=FlightComputerSensors.rotate(design==null?null:design.active("gyroscope"),g.getGyroX(),g.getGyroY(),g.getGyroZ());
        double[] measuredA={fc.accel.getAccelX(),fc.accel.getAccelY(),fc.accel.getAccelZ()},measuredG={fc.gyro.getGyroX(),fc.gyro.getGyroY(),fc.gyro.getGyroZ()};
        for(int i=0;i<3;i++){pair(branch,ACCEL[i],measuredA[i],av[i]);pair(branch,GYRO[i],Math.toRadians(measuredG[i]),Math.toRadians(gv[i]));}
    }
    private static void pair(FlightDataBranch b,Pair p,double measured,double truth){b.setValue(p.measured(),measured);b.setValue(p.truth(),truth);}
    public static JsonObject snapshot(FlightDataBranch branch){
        var values=Json.createObjectBuilder();for(var t:branch.getTypes()){double v=branch.getLast(t);if(Double.isFinite(v))values.add(t.getSymbol(),v);}
        return Json.createObjectBuilder().add("version",1).add("values",values).build();
    }
    public static Map<String,FlightDataType> bySymbol(){var map=new HashMap<String,FlightDataType>();for(var t:FlightDataType.ALL_TYPES)map.put(t.getSymbol(),t);for(var t:ALL_TYPES)map.put(t.getSymbol(),t);return map;}
}
