package info.openrocket.core.simulation.motorbatch;

import info.openrocket.core.motor.*;
import java.util.regex.Pattern;

/** Motor-level category, not verification of current motor approval or launch eligibility.
 * Sources checked 2026-10-11: nar.org/Level1HPRCertificationProcedures and
 * tripoli.org/content.aspx?page_id=22&club_id=795696&module_id=468541.
 */
public final class MotorCertification {
    public enum Level { BELOW_L1, L1, L2, L3, REVIEW }
    public record Classification(String impulseClass,Level level,String basis,String note) { }
    private static final Pattern DESIGNATION=Pattern.compile("^(?:[0-9]+[ -]*)?((?:1/[248])?[A-Z])[0-9].*");
    private MotorCertification() { }
    public static Classification classify(ThrustCurveMotor motor) {
        var matcher=DESIGNATION.matcher(motor.getDesignation().strip());
        String impulse,basis;
        if(matcher.matches()){impulse=matcher.group(1);basis="Manufacturer designation";}
        else {impulse=impulseClass(motor.getTotalImpulseEstimate());basis="Integrated thrust-curve impulse (designation unavailable)";}
        Level level=switch(impulse){case "H","I"->Level.L1;case "J","K","L"->Level.L2;case "M","N","O"->Level.L3;
            case "1/8A","1/4A","1/2A","A","B","C","D","E","F","G"->Level.BELOW_L1;default->Level.REVIEW;};
        String note="Motor category only; current motor certification and complete rocket/cluster requirements are separate.";
        if(level==Level.BELOW_L1) {
            if(motor.getAverageThrustEstimate()>80 || motor.getLaunchMass()-motor.getBurnoutMass()>.125 || motor.getMotorType()==Motor.Type.HYBRID) {
                level=Level.L1;note="Sub-H high-power exception: average thrust >80 N, propellant >125 g, or hybrid.";
            } else note="Below H by impulse. Check sparking propellant, uncertain catalogue data, and complete rocket/cluster requirements.";
        }
        if(level==Level.REVIEW)note="Outside the common NAR/Tripoli H–O categories or missing data; review organization-specific requirements.";
        return new Classification(impulse,level,basis,note);
    }
    public static String impulseClass(double impulse) {
        if(!Double.isFinite(impulse)||impulse<=0)return "Unknown";
        String[] names={"1/8A","1/4A","1/2A","A","B","C","D","E","F","G","H","I","J","K","L","M","N","O"};
        double limit=.3125;for(String name:names){if(impulse<=limit)return name;limit*=2;}return "Above O";
    }
}
