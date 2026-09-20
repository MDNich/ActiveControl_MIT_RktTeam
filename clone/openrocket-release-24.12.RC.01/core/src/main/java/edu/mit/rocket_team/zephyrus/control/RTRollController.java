package edu.mit.rocket_team.zephyrus.control;
import edu.mit.rocket_team.zephyrus.util.RTController;
import edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.Trace;
import edu.mit.rocket_team.zephyrus.util.data.RTFudgedData;
/** Translation of RT_Firmware_Libs/rollcontrol.h/.cpp. Angles are degrees. */
public class RTRollController extends RTController {
    private static final float LAUNCH_ALT = 271.0f;
    private static final float LAUNCH_TEMP = 290.0f;
    private static final float KP = 0.08444f;
    private static final float KD = 0.02111f;
    private static final float Jxx0 = 0.267f;
    private static final float Jxxf = 0.241f;
    private static final float t_b = 2.51f;
    private static final float M_star = 1.24f;
    private static final float h_star = 474.5736f;
    private static final float d_ref = 0.2207f;
    private static final float n_tabs = 2.0f;
    private static final float V_MIN = 20.0f;
    private static final float V_MAX = 1000.0f;

        float T;
        float rho;
        float a;
        float Gd_star_val;
        float v_star;
        float rho_star;
        float T_star;
        float angle;
        float CMx_alpha_val;

    private final Trace trace;
    public RTRollController() { this(new Trace()); }
    public RTRollController(Trace trace) { this.trace=trace; }
    @Override public void setup() { begin(); trace.log("roll.setup", "physical_coupling=false"); }
    @Override public void performLoopAction() { throw new IllegalStateException("Use FC update with explicit inputs"); }
    @Override public void backdoorFudge(RTFudgedData data) { throw new UnsupportedOperationException("Use typed update"); }
    public void atmosphere(float h_m) {
    h_m = h_m + LAUNCH_ALT;

    if (h_m < 0.0f)      h_m = 0.0f;
    if (h_m > 36600.0f)  h_m = 36600.0f;

    float g0 = 9.80665f;
    float R  = 287.05287f;
    float gamma = 1.4f;

    // Layers: (h_base, h_top, lapse_rate)
    float[][] layers = {
        {0.0f,     11000.0f, -0.0065f},
        {11000.0f, 20000.0f,  0.0f},
        {20000.0f, 32000.0f,  0.0010f},
        {32000.0f, 47000.0f,  0.0028f}
    };

    T = LAUNCH_TEMP;  // K
    float P = 101325.0f; // Pa (model starts from sea-level)

    for (int i = 0; i < 4; i++) {
        float h_base = layers[i][0];
        float h_top  = layers[i][1];
        float L      = layers[i][2];

        if (h_m <= h_top) {
            if (L == 0.0f) {
                P *= (float)Math.exp(-g0 * (h_m - h_base) / (R * T));
            } else {
                float T_new = T + L * (h_m - h_base);
                P *= (float)Math.pow((T_new / T), (-g0 / (R * L)));
                T = T_new;
            }
            rho = P / (R * T);
            a = (float)Math.sqrt(gamma * R * T);
            return;
        }

        // Advance to top of layer
        if (L == 0.0f) {
            P *= (float)Math.exp(-g0 * (h_top - h_base) / (R * T));
        } else {
            float T_new = T + L * (h_top - h_base);
            P *= (float)Math.pow((T_new / T), (-g0 / (R * L)));
            T = T_new;
        }
    }

    rho = P / (R * T);
    a = (float)Math.sqrt(gamma * R * T);
}

    public float CMx_alpha(float mach) {
    if (mach < 1) {
        return 2.47539f;
    }
    float p1 = 2.34725224807287f;
    float p2 = 1.04024379248907f;
    float p3 = 1.49368646930894f;
    float p4 = 0.779528950818373f;
    return p1*(float)Math.exp(-p2*(mach - p3)) + p4;
}

    public float Jxx_of_t(float t) {
    if (t <= 0.0f)  return Jxx0;
    if (t >= t_b)  return Jxxf;
    return Jxx0 + (t / t_b) * (Jxxf - Jxx0);
}

    public float Gd(float v, float CMx_alpha_val, float Jxx) {
    if (Jxx == 0.0f) {  // Catch for somehow passing in zero (should never happen)
        return 1.0f;
    }
    return (rho * v * v * CMx_alpha_val) / (2.0f * Jxx);
}

    public void Gd_star() {
    atmosphere(h_star);
    v_star = M_star * a;
    CMx_alpha_val = CMx_alpha(M_star);
    float Jxx_star = Jxx_of_t(t_b);
    Gd_star_val = Gd(v_star, CMx_alpha_val, Jxx_star);
}

    public float K_servo(float v, float mach) {
    float CMxa = CMx_alpha(mach);
    float tau_alpha_proxy =  rho * v * v * CMxa;
    float deprate =  5.5830e-07f;
    return 1.0f - deprate * tau_alpha_proxy;
}

    public void begin() {
    Gd_star();
}

    public void update(float t, float h, float v, float roll, float roll_rate) {
    float v_eff = (v >= V_MIN) ? v : V_MIN;
    v_eff = (v_eff <= V_MAX) ? v_eff : V_MAX;
    atmosphere(h);
    float mach = (a > 0.0f) ? v_eff/a : 0.0f;
    float CMx_a = CMx_alpha(mach);
    float Jxx = Jxx_of_t(t);
    float Gd_val = Gd(v_eff, CMx_a, Jxx);

    float e = -roll;
    float dedt = -roll_rate;
    float K_0 = KP * e + KD * dedt;

    float unscaledAngle = K_0 * Gd_star_val/Gd_val;
    if (unscaledAngle > 10.0f) unscaledAngle = 10.0f;
    if (unscaledAngle < -10.0f) unscaledAngle = -10.0f;

    angle = unscaledAngle * 1.0f / K_servo(v_eff, mach);
    trace.log("roll.update", "flight_s="+t+" altitude_m="+h+" velocity_mps="+v+" roll_deg="+roll+" rate_dps="+roll_rate+" angle_deg="+angle);
}

    public float getAngle() {
    return angle;
}

}
