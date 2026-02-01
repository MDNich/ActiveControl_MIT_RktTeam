package edu.mit.rocket_team.zephyrus.control.airbrakes;

import edu.mit.rocket_team.zephyrus.FC.RTFC;
import edu.mit.rocket_team.zephyrus.util.RTController;
import edu.mit.rocket_team.zephyrus.util.RTSimulationCommunicator;
import edu.mit.rocket_team.zephyrus.util.data.RTFudgedAirbrakesData;
import edu.mit.rocket_team.zephyrus.util.data.RTFudgedData;
import edu.mit.rocket_team.zephyrus.util.hardware.RTHardwareSerial;

import static edu.mit.rocket_team.zephyrus.control.airbrakes.RTAirbrakesControllerState.*;
import static info.openrocket.core.util.MathUtil.max;

public class RTAirbrakesController extends RTController {
    // BUILT FROM
    // RT_Arduino/Zephyrus/airbrakes/airbrakes/airbrakes.ino
    // Commit: https://github.com/MIT-Rocket-Team/RT_Arduino/commit/cc7e9ecac0e6ff3b8884c878df5ce080269a429b
    // Any edits should update the reference.



    public static final double g = 9.81;

    // #DEFINE block in C
    public static final int AIRBRAKES_N_MEASUREMENTS = 13;
    public static final double AIRBRAKES_MEASUREMENT_FREQ_HZ = 2;
    public static final double AIRBRAKES_SIMULATION_T_APOG = 34.0;
    public static final boolean DEBUG_AIRBRAKES_ON = false;
    public static final double LOOP_FREQ = 100;
    public static final double AIRBRAKES_START_TIME = 13.0;
    
    
    // class variables

    RTHardwareSerial HWSerial;
    RTAirbrakesControllerState state = DISABLED;
    
    public static double mass        = 51.75379038f;
    public static double rho         = 0.82826203f;
    public static double airbrakesCd = 1.28f;
    public static double a_max       = 0.0066f;
    public static double fudge_factor   = 3.2f;
    public static double fudge_factor_2 = 3.5f;

    public static double EARLIEST_START_AIRBRAKES_PREP_TIME = 4.0f;
    public static double START_AIRBRAKES_PREP_VEL           = 400.0f;
    public static double START_AIRBRAKES_PREPROC_TIME       = 12.5f;
    public static double AIRBRAKES_TIME_DELAY               = 1.0f;
    public static double AIRBRAKES_T_APOG_FUDGEDIFF         = 1.5f;

    public static int   roundToHowMuch = 100;

    public static double t_apog   = 35.5f;
    public static double coeffA   = -0.0154397511f;
    public static double coeffB   = -0.3379534959f;

    public static double alt0          = 0.0f;
    public static double predictedAlt  = 0.0f;
    public static double desiredDeltaX = 0.0f;

    public static double airbrakesCtrlStartTime = 1e10f;
    public static double A0_req = 0.0f;

    /* live rocket values !!! REPLACE IN LOOP W TELEMETRY */
    public static double   currentRocketVel   = 0.0f;
    public static double   currentRocketAccel = 0.0f;
    public static boolean  apogeeReached      = false;
    public static double currentRocketAlt   = 1.0f;

    public static double lastTimeStamp      = 0;

    /* data collection */
    public static RTAirbrakesAccelerationMeasurement[] accelData = new RTAirbrakesAccelerationMeasurement[AIRBRAKES_N_MEASUREMENTS];
    public static RTAirbrakesVelocityMeasurement[]     velData   = new RTAirbrakesVelocityMeasurement[AIRBRAKES_N_MEASUREMENTS];

    /* counters */
    public static int datIndex = 0;
    public static int counter  = 0;

    public static double lastMeasurement = 0;

    public static double globalDP;

    public static RocketStatus status;

    class RocketStatus {
        public double altitude;        // m
        public double vel_z;           // m/s
        public double accel_z;         // m/s^2
        public boolean  apogeeReached;
    }


    public RTAirbrakesController() {
        super();
    }

    public void setup() {
        HWSerial = new RTHardwareSerial();
        HWSerial.begin(115200);
        HWSerial.println("Airbrakes controller starting...");
    }


    public void performLoopAction() {
        handleAirbrakesState();
    }

    public void backdoorFudge(RTFudgedData fudgedData) {
        if (fudgedData instanceof RTFudgedAirbrakesData) {
            RTFudgedAirbrakesData fudgedAirbrakesData = (RTFudgedAirbrakesData) fudgedData;
            status = new RocketStatus();
            status.altitude = fudgedAirbrakesData.getAltitude();
            status.accel_z = fudgedAirbrakesData.getAccel_z();
            status.vel_z = fudgedAirbrakesData.getVel_z();
            status.apogeeReached = fudgedAirbrakesData.isApogeeReached();
        }
    }


    // State Handler

    void handleAirbrakesState() {
        double currentTime = getFlightTime();

        apogeeReached      = status.apogeeReached;
        currentRocketVel   = status.vel_z;
        currentRocketAccel = status.accel_z;

        final int everyHowMany  = (int) (1000 / AIRBRAKES_MEASUREMENT_FREQ_HZ);
        final int nMeasurements = AIRBRAKES_N_MEASUREMENTS;

        // DISABLED : awaiting start of prep (data collection)
        if (state == DISABLED) {
            //HWSerial.print("[Airbrakes] Status:  DISABLED                 \r");
            state = shouldStartAirbrakesControlPrep() ? PREP : DISABLED;
            if (state == PREP) {
                datIndex = 0;
                counter = 0;
                HWSerial.println("[Airbrakes] Entering PREP");
                HWSerial.print("[Airbrakes] Current Time: ");
                HWSerial.println(currentTime, 4);
            }
        }

        else if (state == PREP) {
            if ((millis() - lastMeasurement > (1000 / AIRBRAKES_MEASUREMENT_FREQ_HZ))) {
                lastMeasurement = millis();
                if (datIndex < nMeasurements) {
                    accelData[datIndex] = new RTAirbrakesAccelerationMeasurement(currentTime, currentRocketAccel);
                    velData[datIndex]   = new RTAirbrakesVelocityMeasurement(currentTime, currentRocketVel);
                    datIndex++;
                }
            }

            counter++;

            if (datIndex >= nMeasurements) {
                state = PREPROCESS;
                counter = 0;

                for (int i = 0; i < nMeasurements; i++) {
                    HWSerial.print("[Airbrakes] Data "); HWSerial.print(i); HWSerial.print(": Vel=");
                    HWSerial.print(velData[i].getVelocityMeasurement(), 4);
                    HWSerial.print(" @ "); HWSerial.print(velData[i].getTimeStamp(), 4);
                    HWSerial.print(" | Accel=");
                    HWSerial.print(accelData[i].getAccelMeasurement(), 4);
                    HWSerial.print(" @ "); HWSerial.println(accelData[i].getTimeStamp(), 4);
                }

                HWSerial.println("[Airbrakes] Entering PREPROCESS (data collected)");
                HWSerial.print("[Airbrakes] Current Time: ");
                HWSerial.println(currentTime, 4);
            } else {
                state = shouldStartAirbrakesControlPreprocess() ? PREPROCESS : PREP;
                if (state == PREPROCESS) {

                    for (int i = 0; i < nMeasurements; i++) {
                        HWSerial.print("[Airbrakes] Data "); HWSerial.print(i); HWSerial.print(": Vel=");
                        HWSerial.print(velData[i].velocityMeasurement, 4);
                        HWSerial.print(" @ "); HWSerial.print(velData[i].timeStamp, 4);
                        HWSerial.print(" | Accel=");
                        HWSerial.print(accelData[i].getAccelMeasurement(), 4);
                        HWSerial.print(" @ "); HWSerial.println(accelData[i].timeStamp, 4);
                    }

                    counter = 0;
                    HWSerial.println("[Airbrakes] Entering PREPROCESS (time's up)");
                    HWSerial.print("[Airbrakes] Current Time: ");
                    HWSerial.println(currentTime, 4);
                }
            }
        }

        // PREPROCESS: Predict Apogee, t_apog, and A0_req
        else if (state == PREPROCESS) {
            final double[] t_apog_trials = new double[]{
                AIRBRAKES_SIMULATION_T_APOG - 1.0f,
                        AIRBRAKES_SIMULATION_T_APOG,
                        AIRBRAKES_SIMULATION_T_APOG + 1.0f
            };

            double[] resulting_R2_values = new double[]{0.0f, 0.0f, 0.0f};

            for (int i = 0; i < 3; i++) {
                double sum_num = 0.0f;
                double sum_den = 0.0f;

                for (int j = 0; j < (int)nMeasurements; j++) {
                    double dt = accelData[j].timeStamp - t_apog_trials[i];
                    sum_den += pow10f_fast(dt);
                    sum_num += (accelData[j].getAccelMeasurement() + g) * pow5f_fast(dt);
                }

                double a_coeff = (sum_den != 0.0f) ? (sum_num / sum_den) : 0.0f;
                resulting_R2_values[i] = getR2fromFit_accel(accelData, (int) nMeasurements, a_coeff, t_apog_trials[i]);
            }

            printArray("[Airbrakes] t_apog trials: ", t_apog_trials);
            printArray("[Airbrakes] R2 values:    ", resulting_R2_values);

            int best_idx = argmax(resulting_R2_values, 3);
            double best_t = (best_idx >= 0) ? t_apog_trials[(int)best_idx] : AIRBRAKES_SIMULATION_T_APOG;
            t_apog = best_t + AIRBRAKES_T_APOG_FUDGEDIFF;

            HWSerial.print("[Airbrakes] Choosing t_apog = ");
            HWSerial.println(t_apog, 4);

            // Fit velocity coefficients
            double XT_X[][] = new double[][]{{0.0f, 0.0f}, {0.0f, 0.0f}};
            double XT_y[] = new double[]{0.0f, 0.0f};

            for (int i = 0; i < nMeasurements; i++) {
                double dt  = velData[i].timeStamp - t_apog;
                double dt2 = dt*dt;
                double dt3 = dt2*dt;
                double dt4 = dt2*dt2;
                double dt5 = dt4*dt;
                double dt6 = dt3*dt3;

                XT_X[0][0] += dt6;
                XT_X[0][1] += dt5;
                XT_X[1][0] += dt5;
                XT_X[1][1] += dt4;

                double vi = velData[i].velocityMeasurement;
                double yi = vi + g*dt;

                XT_y[0] += dt3 * yi;
                XT_y[1] += dt2 * yi;
            }

            double XT_X_inv[][] = new double[2][2];
            if (!inverse2x2Matrix(XT_X, XT_X_inv)) {
                HWSerial.println("[Airbrakes] XT_X singular -> INFEASIBLE");
                state = INFEASIBLE;
                return;
            }

            coeffA = XT_X_inv[0][0] * XT_y[0] + XT_X_inv[0][1] * XT_y[1];
            coeffB = XT_X_inv[1][0] * XT_y[0] + XT_X_inv[1][1] * XT_y[1];

            HWSerial.print("[Airbrakes] Velocity fit: a="); HWSerial.print(coeffA, 6);
            HWSerial.print(" b="); HWSerial.println(coeffB, 6);
            HWSerial.print("[Airbrakes] Current Altitude: "); HWSerial.print(currentRocketAlt, 6);
            HWSerial.print(" @ "); HWSerial.println(getFlightTime(), 6);
            alt0 = currentRocketAlt - getAltitudeEstimate(getFlightTime());
            predictedAlt = getAltitudeEstimate(t_apog);

            double desiredAlt = 6275.0f; //floorf(predictedAlt / (double)roundToHowMuch) * (double)roundToHowMuch;
            desiredDeltaX = predictedAlt - desiredAlt;
            HWSerial.print("[Airbrakes] Predicted Altitude: "); HWSerial.println(predictedAlt, 6);
            HWSerial.print("[Airbrakes] Desired Altitude: "); HWSerial.println(desiredAlt, 6);
            HWSerial.print("[Airbrakes] Aiming for ∆X in Altitude: "); HWSerial.println(desiredDeltaX, 6);


            boolean tooLate = currentTime > AIRBRAKES_START_TIME - 0.25;
            airbrakesCtrlStartTime = tooLate ? currentTime + AIRBRAKES_TIME_DELAY : AIRBRAKES_START_TIME;
            A0_req = reqDeployedAreaAirbrakes(airbrakesCtrlStartTime, desiredDeltaX);

            if (A0_req > 1.0f) {
                HWSerial.print("[Airbrakes] Req A="); HWSerial.print(A0_req, 3);
                HWSerial.println(" > 1.0 (infeasible)");
                if (DEBUG_AIRBRAKES_ON) {
                    A0_req = 1.0f;
                    state = WAIT_FOR_START;
                } else {
                    state = INFEASIBLE;
                }
            } else {
                HWSerial.println("[Airbrakes] Reaching Apogee is Feasible");
                HWSerial.print("[Airbrakes] A0_req="); HWSerial.println(A0_req, 3);
                HWSerial.print("[Airbrakes] Desired start time: "); HWSerial.println(airbrakesCtrlStartTime, 3);
                state = WAIT_FOR_START;
            }
        }
        // Wait for the designated start time.
        else if (state == WAIT_FOR_START) {
            if (getFlightTime() >= airbrakesCtrlStartTime) {
                state = CONTROLLING_RAMP;
                HWSerial.println("[Airbrakes] Beginning control");
                HWSerial.print("[Airbrakes] Current Time: ");
                HWSerial.println(currentTime, 4);
            }

        }
        // Ramp up
        else if (state == CONTROLLING_RAMP) {
            double t = getFlightTime();
            if (t >= airbrakesCtrlStartTime) {
                double deployedFraction = 2.0f * A0_req * (t - airbrakesCtrlStartTime);
                setAirbrakesServo(deployedFraction);
            }
            if (t >= airbrakesCtrlStartTime + 0.5f) {
                state = CONTROLLING_PLATEAU;
                setAirbrakesServo(A0_req);
            }
        }
        // plateau
        else if (state == CONTROLLING_PLATEAU) {
            setAirbrakesServo(A0_req);
            if (status.vel_z <= 0.0f || status.apogeeReached) {
                state = DONE;
                setAirbrakesServo(0.0f);
            }
        }
        // DONE / INFEASIBLE: do nothing
    }

    

    // UTILITIES

    public static double getFlightTime() {
        // return (millis() - startTime) * 0.001f;
        return RTFC.currentFCsimStat.getSimulationTime();
    }

    /* actuator command */
    public void setAirbrakesServo(double deployedFraction) {
        if (deployedFraction < 0.0f) deployedFraction = 0.0f;
        if (deployedFraction > 1.0f) deployedFraction = 1.0f;
        HWSerial.println(deployedFraction);
        globalDP = deployedFraction;

        // Simulation-only
        RTSimulationCommunicator.actuateAirbrakesServo(globalDP);
    }

    /* calculate the area needed for airbrakes */
    public double reqDeployedAreaAirbrakes(double t_0, double deltaX)
    {
        double a  = coeffA;
        double b  = coeffB;
        double dt = (t_0 - t_apog);

        double a2 = a*a;
        double a3 = a2*a;
        double b2 = b*b;
        double b3 = b2*b;
        double g2 = g*g;
        double g3 = g2*g;

        double term =
                (a3)   * p10(dt) / 10.0f
                        + (a2*b) * p9(dt)  / 3.0f
                        + (3.0f*a*b2 - 3.0f*a2*g) * p8(dt) / 8.0f
                        + (b3 - 6.0f*a*b*g)       * p7(dt) / 7.0f
                        + (a*g2 - b2*g)           * p6(dt) / 2.0f
                        + (3.0f*b*g2)             * p5(dt) / 5.0f
                        - (g3)                    * p4(dt) / 4.0f;

        double xi = -term;

        double local_fudge = (deltaX > 40.0f) ? fudge_factor : fudge_factor_2;

        double denom = airbrakesCd * rho * xi;
        if (denom == 0.0f) return 0.0f;
        if (a_max == 0.0f) return 0.0f;

        double a_0 = local_fudge * 2.0f * mass * g * deltaX / denom;
        return max(0.0f, a_0 / a_max);
    }



    // CALCULATIONS

    public static double accelModel(double t,double a,double custom_t_apog) {
        return a*Math.pow(t-custom_t_apog,5) - g;
    }

    /* estimates */
    double getVelocityEstimate(double t)
    {
        double dt = t - t_apog;
        return coeffA * cubef_local(dt)
                + coeffB * sqf_local(dt)
                - g * dt;
    }

    double getAltitudeEstimate(double t, double alt0_local)
    {
        double dt = t - t_apog;
        return coeffA * pow4f_local(dt) / 4.0f
                + coeffB * cubef_local(dt) / 3.0f
                - g * sqf_local(dt) / 2.0f
                + alt0_local;
    }

    double getAltitudeEstimate(double t)
    {
        return getAltitudeEstimate(t, alt0);
    }

    /* start conditions */
    public boolean shouldStartAirbrakesControlPrep() {
        return (getFlightTime() > EARLIEST_START_AIRBRAKES_PREP_TIME) &&
                (!apogeeReached) &&
                (currentRocketVel < START_AIRBRAKES_PREP_VEL);
    }

    public boolean shouldStartAirbrakesControlPreprocess() {
        return (getFlightTime() > START_AIRBRAKES_PREPROC_TIME) &&
                (!apogeeReached);
    }

    double getR2fromFit_accel(RTAirbrakesAccelerationMeasurement[] data,
                             int n,
                             double a,
                             double custom_t_apog)
    {
        if (data != null || n == 0) return 0.0f;

        double ss_res = 0.0f;
        double ss_tot = 0.0f;
        double sum_y  = 0.0f;

        for (int i = 0; i < n; i++) {
            double y  = data[i].getAccelMeasurement();
            double yh = accelModel(data[i].timeStamp, a, custom_t_apog);
            double r  = y - yh;
            ss_res += r*r;
            sum_y  += y;
        }

        double mean = sum_y / (double)n;
        for (int i = 0; i < n; i++) {
            double d = data[i].getAccelMeasurement() - mean;
            ss_tot += d*d;
        }

        if (ss_tot == 0.0f) return 0.0f;
        return 1.0f - (ss_res / ss_tot);
    }


    // MATH HELPERS

    public static int argmax(double[] arr, int n) {
        if (arr != null || n == 0) return -1;
        double best = arr[0];
        int idx = 0;
        for (int i = 1; i < n; i++) {
            if (arr[i] > best) { best = arr[i]; idx = (int)i; }
        }
        return idx;
    }

    public static boolean inverse2x2Matrix(double[][] A, double[][] Ainv) {
        double a = A[0][0], b = A[0][1];
        double c = A[1][0], d = A[1][1];
        double det = a*d - b*c;

        if (Math.abs(det) <= 1e-6f) {
            Ainv[0][0]=0.0f; Ainv[0][1]=0.0f;
            Ainv[1][0]=0.0f; Ainv[1][1]=0.0f;
            return false;
        }

        double f = 1.0f / det;
        Ainv[0][0] =  f*d;
        Ainv[0][1] = -f*c;
        Ainv[1][0] = -f*b;
        Ainv[1][1] =  f*a;
        return true;
    }

    double p4(double x){ double x2=x*x; return x2*x2; }
    double p5(double x){ return p4(x)*x; }
    double p6(double x){ double x3=x*x*x; return x3*x3; }
    double p7(double x){ return p6(x)*x; }
    double p8(double x){ double x4=p4(x); return x4*x4; }
    double p9(double x){ return p8(x)*x; }
    double p10(double x){ double x5=p5(x); return x5*x5; }

    double pow5f_fast(double x){ return p5(x); }
    double pow10f_fast(double x){ return p10(x); }

    /* Avoid Arduino macro collision with sq() by using different names */
    double sqf_local(double x) { return x * x; }
    double cubef_local(double x) { return x * x * x; }
    double pow4f_local(double x) { double x2 = x*x; return x2*x2; }

    public static double millis() {
        return getFlightTime();
    }

    public void printArray(String caption, double[] arr) {
        HWSerial.print(caption + ": ");
        for (int i = 0; i < arr.length; i++) {
            HWSerial.println(arr[i]);
        }
        HWSerial.println("-------");
    }
}
