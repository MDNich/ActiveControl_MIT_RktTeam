package info.openrocket.core.simulation.motorbatch;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import static info.openrocket.core.simulation.motorbatch.MotorBatchReport.*;

/** Typed CSV export; rows with failed or unavailable metrics are deliberately retained. */
public final class MotorBatchCSV {
    public enum Scope { ALL_CANDIDATES, MEETS_CRITERIA, SELECTED_ROWS, ALL_DELAY_TRIALS }
    private MotorBatchCSV() { }
    public static void write(Path destination,MotorBatchReport report,Scope scope,Set<String> selected) throws IOException {
        destination=destination.toAbsolutePath();Path temp=Files.createTempFile(destination.getParent(),".motor-batch-",".csv");
        try {
            try(var out=Files.newBufferedWriter(temp,StandardCharsets.UTF_8)){write(out,report,scope,selected);}
            try{Files.move(temp,destination,StandardCopyOption.ATOMIC_MOVE,StandardCopyOption.REPLACE_EXISTING);}
            catch(AtomicMoveNotSupportedException e){Files.move(temp,destination,StandardCopyOption.REPLACE_EXISTING);}
        } finally {Files.deleteIfExists(temp);}
        System.out.println("MOTOR_BATCH csv="+destination+" scope="+scope);
    }
    public static void write(Writer out,MotorBatchReport report,Scope scope,Set<String> selected) throws IOException {
        var headers=new ArrayList<String>(List.of("schema_version","batch_id","candidate_id","trial_number","manufacturer","designation","motor_type","curve_digest",
                "motor_diameter_m","motor_length_m","total_impulse_N_s","average_thrust_N","peak_thrust_N","burn_time_s","motor_launch_mass_kg","motor_burnout_mass_kg",
                "mount_id","mount_name","motor_instances","delay_s_or_plugged","delay_policy","execution_status","assessment","reasons","warnings","error","trials_attempted","trials_planned"));
        for(var metric:MotorBatchMetrics.Metric.values())headers.add(metric.column);
        headers.addAll(List.of("impulse_class","nar_tripoli_motor_level","classification_basis","classification_note","application_version","created_utc","baseline_simulation","configuration_id","input_hash","threads","peak_threads","random_seed",
                "fc_enabled","fc_reference","fc_fingerprint","fc_seed","fc_timing_seed","telemetry_path","log_path","filters_and_criteria_json","launch_conditions_xml",
                "component_intersection_override","ignored_component_intersections"));
        line(out,headers);
        for(Row row:report.rows()) {
            if(scope==Scope.MEETS_CRITERIA&&row.selected().assessment()!=Assessment.MEETS_CRITERIA)continue;
            if(scope==Scope.SELECTED_ROWS&&!selected.contains(row.candidate().id()))continue;
            var trials=scope==Scope.ALL_DELAY_TRIALS&&!row.trials().isEmpty()?row.trials():List.of(row.selected());
            for(Trial t:trials) {
                var m=row.candidate().motor();var input=report.input();
                var fields=new ArrayList<Object>(List.of(1,report.id(),row.candidate().id(),t.number(),m.getManufacturer().getDisplayName(),m.getDesignation(),m.getMotorType().name(),m.getDigest(),
                        m.getDiameter(),m.getLength(),m.getTotalImpulseEstimate(),m.getAverageThrustEstimate(),m.getMaxThrustEstimate(),m.getBurnTimeEstimate(),m.getLaunchMass(),m.getBurnoutMass(),
                        report.settings().mountId().toString(),input.getString("targetName"),input.getInt("motorCount"),
                        t.delay()==Double.POSITIVE_INFINITY?"plugged":t.delay(),report.settings().delayPolicy().name(),t.status().name(),t.assessment().name(),
                        String.join("; ",t.reasons()),t.warnings(),t.error(),row.trials().size(),row.planned()));
                for(var metric:MotorBatchMetrics.Metric.values())fields.add(t.metrics().getOrDefault(metric,Double.NaN));
                String xml=input.getString("xml"),launch="";int a=xml.indexOf("<conditions>"),b=xml.indexOf("</conditions>");
                if(a>=0&&b>a)launch=xml.substring(a,b+"</conditions>".length());
                var certification=MotorCertification.classify(m);
                fields.addAll(List.of(certification.impulseClass(),certification.level().name(),certification.basis(),certification.note(),input.getString("version"),report.created(),input.getString("baseline"),input.getString("configuration"),input.getString("hash"),
                        report.settings().threads(),report.peakThreads(),input.getInt("seed"),input.getBoolean("fcEnabled"),input.getString("fcReference"),input.getString("fcFingerprint"),
                        input.getInt("fcSeed"),input.getInt("fcTimingSeed"),t.telemetryPath(),t.logPath(),MotorBatchIO.settings(report.settings()).toString(),launch,
                        report.settings().ignoreComponentIntersections(),String.join("; ",row.candidate().ignoredIntersections())));
                line(out,fields);
            }
        }
    }
    private static void line(Writer out,List<?> values)throws IOException {
        for(int i=0;i<values.size();i++) {
            if(i>0)out.write(',');Object v=values.get(i);String s;
            if(v instanceof Number n)s=Double.isFinite(n.doubleValue())?n.toString():"";
            else {
                s=String.valueOf(v);String stripped=s.stripLeading();
                if(!stripped.isEmpty()&&"=+-@".indexOf(stripped.charAt(0))>=0)s="'"+s;
            }
            out.write('"');out.write(s.replace("\"","\"\""));out.write('"');
        }
        out.write("\r\n");
    }
}
