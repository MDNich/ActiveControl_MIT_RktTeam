package info.openrocket.core.simulation.mission;

import jakarta.json.*;

/** Time-weighted receiver analysis; acquisition clocks are distinct from arrival/freshness clocks. */
public final class MissionReceptionMetrics {
    private long last=-1_000_000, received=Long.MIN_VALUE, acquisition, first=-1, freshUs, observedUs, trackedUs, saturatedUs;
    private long gaps, gapStart=-1_000_000, longestGap;
    private double previousError=-1, errorIntegral;
    private boolean previousTracking, previousSaturated;
    public void sample(JsonObject sample,long now,boolean beam) {
        var details=sample.getJsonObject("details");if(details.getBoolean("outOfOrder",false))return;
        if(gapStart!=Long.MIN_VALUE){longestGap=Math.max(longestGap,now-gapStart);if(now>gapStart)gaps++;gapStart=Long.MIN_VALUE;}
        received=now;acquisition=Long.parseLong(details.getString("acquisitionTimeUs",Long.toString(now)));
        if(first<0 && now>=0 && beam && sample.getInt("gps_fix",0)>0 && !sample.isNull("latitude") && !sample.isNull("longitude"))first=now;
    }
    public void advance(long now,boolean tracking,double error,boolean saturated) {
        long dt=now-last;if(dt<0)throw new IllegalArgumentException("Metrics clock moved backwards");
        long fresh=received==Long.MIN_VALUE?0:Math.max(0,Math.min(now,received+2_000_000)-last);
        fresh=Math.min(dt,fresh);freshUs+=fresh;
        if(received!=Long.MIN_VALUE){
            // Age integral is bounded in seconds to avoid signed-64 overflow on long missions.
            double start=Math.max(0,last-acquisition)/1e6,end=Math.max(0,now-acquisition)/1e6;
            ageSecondsIntegral+=(start+end)*.5*dt/1e6;ageObservedUs+=dt;maximumAgeSeconds=Math.max(maximumAgeSeconds,end);
        }
        if(fresh<dt && gapStart==Long.MIN_VALUE)gapStart=last+fresh;
        if(previousError>=0){observedUs+=dt;errorIntegral+=previousError*dt/1e6;}
        if(previousTracking)trackedUs+=dt;if(previousSaturated)saturatedUs+=dt;
        previousTracking=tracking;previousSaturated=saturated;previousError=error;last=now;
    }
    private double ageSecondsIntegral,maximumAgeSeconds;
    private long ageObservedUs;
    public JsonObject json(long end) {
        advance(end,previousTracking,previousError,previousSaturated);
        long interval=end+1_000_000;
        return Json.createObjectBuilder().add("profile","mission-receiver-analysis.v1").add("intervalStartUs","-1000000").add("intervalEndUs",Long.toString(end))
                .add("freshnessLimitUs","2000000").add("freshUs",Long.toString(freshUs)).add("intervalUs",Long.toString(interval))
                .add("freshFraction",interval==0?JsonValue.NULL:Json.createValue((double)freshUs/interval))
                .add("firstUsableAcquisitionUs",first<0?JsonValue.NULL:Json.createValue(Long.toString(first)))
                .add("longestOutageUs",Long.toString(Math.max(longestGap,gapStart!=Long.MIN_VALUE?end-gapStart:0)))
                .add("completedOutages",gaps).add("trackingUs",Long.toString(trackedUs)).add("saturatedUs",Long.toString(saturatedUs))
                .add("geometryObservedUs",Long.toString(observedUs)).add("pointingErrorIntegralDegSeconds",errorIntegral)
                .add("meanPointingErrorDeg",observedUs==0?JsonValue.NULL:Json.createValue(errorIntegral/(observedUs/1e6)))
                .add("informationAgeObservedUs",Long.toString(ageObservedUs)).add("informationAgeIntegralSecondsSquared",ageSecondsIntegral)
                .add("meanInformationAgeSeconds",ageObservedUs==0?JsonValue.NULL:Json.createValue(ageSecondsIntegral/(ageObservedUs/1e6)))
                .add("maximumInformationAgeSeconds",maximumAgeSeconds).build();
    }
}
