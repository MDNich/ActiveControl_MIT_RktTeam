package info.openrocket.core.simulation.flightcomputer;

import edu.mit.rocket_team.zephyrus.FC.RTFC;
import edu.mit.rocket_team.zephyrus.util.data.*;
import jakarta.json.JsonObject;
import java.util.*;

/** Per-flight sensor acquisition at accepted FC sample boundaries, with stable-ID random streams. */
public final class FlightComputerSensors {
    private final FlightComputerDesign design;
    private final Map<String,Random> random=new HashMap<>();
    private final Map<String,Long> last=new HashMap<>();
    public FlightComputerSensors(FlightComputerDesign d){design=d;}
    private double prop(JsonObject n,String key){return n.getJsonObject("properties").containsKey(key)?n.getJsonObject("properties").getJsonNumber(key).doubleValue():0;}
    private boolean due(JsonObject n,long time) {
        String id=n.getString("id");
        if(last.containsKey(id)&&time-last.get(id)<prop(n,"periodUs"))return false;
        last.put(id,time);return true;
    }
    private double value(JsonObject n,double x) {
        var r=random.computeIfAbsent(n.getString("id"),id->new Random((long)design.parameter("sensorSeed") ^ ((long)id.hashCode()<<32)));
        double percent=Math.hypot(design.parameter("measurementNoisePercent"),prop(n,"noisePercent"))/100;
        return x+prop(n,"bias")+Math.hypot(prop(n,"noise"),Math.abs(x)*percent)*r.nextGaussian();
    }
    private double percentValue(JsonObject n,double x){double percent=Math.hypot(design.parameter("measurementNoisePercent"),prop(n,"noisePercent"))/100;return x+Math.abs(x)*percent*random.computeIfAbsent(n.getString("id"),id->new Random((long)design.parameter("sensorSeed") ^ ((long)id.hashCode()<<32))).nextGaussian();}
    public static double[] rotate(JsonObject n,double x,double y,double z){
        double[] v={x,y,z};if(n==null)return v;
        for(int axis=0;axis<3;axis++) {
            double a=Math.toRadians(n.getJsonObject("properties").getJsonNumber("rotation"+"XYZ".charAt(axis)).doubleValue()),c=Math.cos(a),s=Math.sin(a);
            int j=(axis+1)%3,k=(axis+2)%3;double b=v[j],d=v[k];v[j]=c*b-s*d;v[k]=s*b+c*d;
        }
        return v;
    }
    private double[] vector(JsonObject n,double x,double y,double z){var v=rotate(n,x,y,z);for(int i=0;i<3;i++)v[i]=value(n,v[i]);return v;
    }
    public RTFC.Inputs sample(long now,RTFC.Inputs in) {
        RTBaroData baro=null; RTAccelData accel=null; RTGyroData gyro=null; RTGPSData gps=null;
        var n=design.active("barometer");
        if(in.baro()!=null&&due(n,now)) {
            var b=in.baro();double pressure=value(n,b.getPressure());
            pressure=Math.max(.001,pressure);
            baro=new RTBaroData(Float.NaN,Float.NaN,(float)(Math.max(1,percentValue(n,b.getTemperature()+273.15))-273.15),Float.NaN,(float)pressure);
        }
        n=design.active("accelerometer");
        if(in.accel()!=null&&due(n,now)) {var a=in.accel();var v=vector(n,a.getAccelX(),a.getAccelY(),a.getAccelZ());accel=new RTAccelData((float)v[0],(float)v[1],(float)v[2]);}
        n=design.active("gyroscope");
        if(in.gyro()!=null&&due(n,now)) {var g=in.gyro();var v=vector(n,g.getGyroX(),g.getGyroY(),g.getGyroZ());gyro=new RTGyroData((float)v[0],(float)v[1],(float)v[2]);}
        n=design.active("gps");
        if(in.gps()!=null&&due(n,now)) {var g=in.gps();gps=new RTGPSData(Math.max(-90,Math.min(90,percentValue(n,g.getLatitude()))),Math.IEEEremainder(percentValue(n,g.getLongitude()),360),value(n,g.getAltitude()),(double)g.getPDOP(),(double)g.getVDOP(),(double)g.getHDOP(),g.getHasFix());gps.setFixType(g.getFixType());}
        return new RTFC.Inputs(in.acquisitionUs(),accel,baro,gps,gyro);
    }
}
