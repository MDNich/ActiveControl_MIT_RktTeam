package info.openrocket.swing.gui.flightcomputer;
import edu.mit.rocket_team.zephyrus.FC.RTFC;
import edu.mit.rocket_team.zephyrus.util.data.*;
import java.nio.file.*;
import java.io.*;
import java.util.*;
/** Deliberately explicit raw-input format; GS telemetry estimates are not raw sensor measurements. */
final class FlightComputerReplay {
    static NavigableMap<Long,RTFC.Inputs> read(Path path)throws IOException {
        var data=new TreeMap<Long,RTFC.Inputs>();
        try(var reader=Files.newBufferedReader(path)){
            String header=reader.readLine();if(!"time_us,ax,ay,az,pressure_hpa,temperature_c,gx,gy,gz,latitude,longitude,gps_altitude_m".equals(header))throw new IOException("Unexpected replay header; use the raw sensor CSV columns shown in the test view");
            String line;long previous=-1;int row=1;
            while((line=reader.readLine())!=null){row++;try{var cells=line.split(",",-1);if(cells.length!=12)throw new IllegalArgumentException("Expected 12 columns");double[] v=new double[12];for(int i=0;i<12;i++){v[i]=Double.parseDouble(cells[i]);if(!Double.isFinite(v[i]))throw new IllegalArgumentException("Nonfinite value");}long t=Long.parseLong(cells[0]);if(t<=previous||t<0||t>600_000_000L||data.size()>100000)throw new IllegalArgumentException("Times must increase and be within 0–600 s; maximum 100,000 rows");if(data.isEmpty()&&t!=0)throw new IllegalArgumentException("First timestamp must be 0");if(v[4]<=0)throw new IllegalArgumentException("Pressure must be positive");
                data.put(t,new RTFC.Inputs(t,new RTAccelData((float)v[1],(float)v[2],(float)v[3]),new RTBaroData(Float.NaN,Float.NaN,(float)v[5],Float.NaN,(float)v[4]),new RTGPSData(v[9],v[10],v[11],0.0,0.0,0.0,true),new RTGyroData((float)v[6],(float)v[7],(float)v[8])));previous=t;
            }catch(RuntimeException e){throw new IOException("Replay row "+row+": "+e.getMessage(),e);}}
        }
        if(data.isEmpty())throw new IOException("Replay has no samples");return data;
    }
}
