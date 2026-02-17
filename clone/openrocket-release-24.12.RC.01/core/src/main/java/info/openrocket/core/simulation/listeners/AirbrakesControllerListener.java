package info.openrocket.core.simulation.listeners;

import info.openrocket.core.rocketcomponent.AirbrakeSet;
import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.simulation.AccelerationData;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.simulation.exception.SimulationException;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import static info.openrocket.core.simulation.listeners.AirbrakesControllerState.*;
import static info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener.*;
import static info.openrocket.core.simulation.listeners.NewControlStepListener.*;
import static info.openrocket.core.util.MathUtil.max;
import static java.lang.Math.toDegrees;

public class AirbrakesControllerListener extends AbstractSimulationListener{
    public static SimulationStatus currentStatus;
    public static SimulationStatus initialStatus;
    public static AirbrakeSet theAirbrakes;
    public static Rocket theRocket;


    public static double targetAlt = 5500.0;
    public static double latestTimeStep = -1;
    public static double predictedAlt;
    public static ArrayList<Double> pastOmegaZ;
    public static ArrayList<Double> pastThetaZ;
    public static ArrayList<Double> rktVelMagLog;
    public static ArrayList<Double> rktAltLog;
    public static ArrayList<Double> airbrakesLog;
    public static ArrayList<Double> timeLog;
    public static ArrayList<AirbrakesAccelerationMeasurement> accelMeasurements;
    public static ArrayList<AirbrakesVelocityMeasurement> velMeasurements;
    public static AirbrakesControllerState state = AirbrakesControllerState.DISABLED;
    public static boolean shouldPrep = false;

    public static double mass = 53.48;
    public static double g = 9.81;
    public static double rho = 0.736;
    public static double airbrakesCd = AirbrakeSet.CD_perp;//1.28;
    public static double rocketCd = 0.5859;
    public static double aRef = 0.01929;
    // maximum airbrakes area
    public static double a_max = 0.0066; // TODO get from AirbrakesSet
    public static int counter = 0;
    public static int roundToHowMuch = 100; // desired altitude correction
    public static double desiredDeltaX = 0;

    public static double airbrakesCtrlStartTime = 1e10;
    public static double A0_req = 0;

    public static double EARLIEST_AIRBRAKES_PREP_TIME = 4.0;
    public static double START_AIRBRAKES_PREP_VEL = 400.0;
    public static double START_AIRBRAKES_PREPROC_TIME = 12.0;
    public static double AIRBRAKES_TIME_DELAY = 1.0;
    public static int AIRBRAKES_N_MEASUREMENTS = 13;
    public static int AIRBRAKES_MEASUREMENT_FREQ_HZ = 5;
    public static int AIRBRAKES_SIMULATION_T_APOG = 34;
    public static double AIRBRAKES_T_APOG_FUDGEDIFF = 1.5;

    public static double overriden_A0 = -1;
    public static double overriden_desiredApog = -1.0;


    /*public static double t_apog = 35; //time of apogee
    public static double coeffA = -0.013498522289161072;
    public static double coeffB = -0.27159399218754415;
    public static double alt0 = 6333.741403445408;*/
    public static double t_apog = 35.5; //time of apogee
    public static double coeffA = -0.01543975114159146;
    public static double coeffB = -0.3379534958690176;


    public static double override_t_apog = -1;

    public static double alt0 = 6320.235959562471;
    public static double fudge_factor = 3.2;
    public static double fudge_factor_2 = 3.5;

    public static boolean DEBUG_AIRBRAKES_ON = false;

    public static double K = 1;
    public static double factorK = 1;
    public static double kp_factor = 1;
    public static double ki_factor = 1;

    public static ArrayList<Double> I;
    public static ArrayList<Double> deltaA;
    public static ArrayList<Double> timeStampDeltas;
    public static ArrayList<Double> A;
    public static ArrayList<Double> deltaH;
    public static ArrayList<Double> Hf;
    public static double Astar = 0;
    public static double patchingAltitude = 0;
    public static double velContribFudge = 0.75;
    public static double cFudge = 0.825;

    public static double AIRBRAKES_MEASUREMENT_FUDGE_FACTOR = 1;


    public static double lastControlStepTime =0;
    public static double rate =0.1;

    public AirbrakesControllerListener(){
        super();
        pastOmegaZ = new ArrayList<>();
        pastThetaZ = new ArrayList<>();
        rktVelMagLog = new ArrayList<>();
        rktAltLog = new ArrayList<>();
        airbrakesLog = new ArrayList<>();
        timeLog = new ArrayList<>();
        accelMeasurements = new ArrayList<>();
        velMeasurements = new ArrayList<>();

        I = new ArrayList<>();
        deltaA = new ArrayList<>();
        timeStampDeltas = new ArrayList<>();
        deltaH = new ArrayList<>();
        Hf = new ArrayList<>();
        A = new ArrayList<>();
        I.add(0.0);
        deltaA.add(0.0);
        deltaH.add(0.0);
        //Hf.add(0.0);

    }


    @Override
    public void startSimulation(SimulationStatus status) throws SimulationException {
        super.startSimulation(status);

        /*if (initialStatus != null) {
            status.copySimStatParameters(initialStatus);
        }
        else {
            System.out.println("[JAVA] Initializing simulation status, could not read from Python.");
            initialStatus = status.clone();
        }*/
        theRocket = status.getConfiguration().getRocket();

        /*if (TIME_DELAY_MOTOR > 0) {
            ((MotorClusterState) status.getActiveMotors().toArray()[0]).ignite(status.getSimulationTime()-TIME_DELAY_MOTOR); // 3.5 seconds before
        }
        if (TIME_DELAY_MOTOR > 100) {
            ((MotorClusterState) status.getActiveMotors().toArray()[0]).burnOut(status.getSimulationTime()-TIME_DELAY_MOTOR); // 3.5 seconds before
        }*/

        currentStatus = status.clone();

        theAirbrakes = getAirbrakes(status);
        theAirbrakes.fudgefactor = AIRBRAKES_MEASUREMENT_FUDGE_FACTOR;
    }

    @Override
    public void postStep(SimulationStatus status) throws SimulationException {
        double airbrakesTimeStep = status.getSimulationTime();
        latestTimeStep = airbrakesTimeStep;

        double realVelocity = status.getRocketVelocity().length();

        pastOmegaZ.add(status.getRocketRotationVelocity().z);
        pastThetaZ.add(toDegrees(toEulerAngles_rocketCoord(status.getRocketOrientationQuaternion()).z));
        rktVelMagLog.add(realVelocity);
        airbrakesLog.add(theAirbrakes.getFracExposed());
        timeLog.add(status.getSimulationTime());



        double currentTime = status.getSimulationTime();
        System.out.print("[JAVA] current time " + currentTime + "                     \r");

        mass = getRocketMass(theRocket);
        rocketCd = getRocketCD(theRocket);
        aRef = status.getFlightConfiguration().getReferenceArea();
        rho = status.getSimulationConditions().getAtmosphericModel().getConditions(status.getRocketWorldPosition().getAltitude()).getDensity();




        super.postStep(status);
        this.handleAirbrakesState(status);


        lastIterTime = status.getSimulationTime();
    }

    public void handleAirbrakesState(SimulationStatus status) {
        double currentTime = status.getSimulationTime();
        int everyHowMany = 1000/AIRBRAKES_MEASUREMENT_FREQ_HZ; // 5 Hz -> 1 s/ 5 s^{-1} -> 1/5 times per sec -> 1/5*1000 times per ms
        int nMeasurements = AIRBRAKES_N_MEASUREMENTS;

        // if state is disabled, check if should start prep
        if (state == DISABLED){
            state =  (shouldStartAirbrakesControlPrep(status)) ? PREP : DISABLED;
        }
        else if (state == PREP) {

            if (counter % everyHowMany == 0) {
                List<Double> accelZ = status.getFlightDataBranch().get(FlightDataType.TYPE_ACCELERATION_Z);
                List<Double> velZ = status.getFlightDataBranch().get(FlightDataType.TYPE_VELOCITY_Z);
                AirbrakesAccelerationMeasurement accelDat = new AirbrakesAccelerationMeasurement(currentTime, accelZ.get(accelZ.size() - 1));
                AirbrakesVelocityMeasurement velDat = new AirbrakesVelocityMeasurement(currentTime, velZ.get(velZ.size() - 1));
                accelMeasurements.add(accelDat);
                velMeasurements.add(velDat);
            }
            counter++;
            if (velMeasurements.size() >= nMeasurements) {
                state = PREPROCESS;
                counter = 0;
            }
            state =  (shouldStartAirbrakesControlPreprocess(status)) ? PREPROCESS : PREP;
        }
        else if (state == PREPROCESS){
            // calculate alpha and t_apog from linear fit to accel measurements
            // TODO implement preprocessing to estimate alpha and t_apog

            double[] t_apog_trials = new double[]{AIRBRAKES_SIMULATION_T_APOG - 1.0, AIRBRAKES_SIMULATION_T_APOG, AIRBRAKES_SIMULATION_T_APOG + 1.0};
            double[] resulting_R2_values = new double[3];
            for (int i = 0; i < t_apog_trials.length; i++) {
                // fit the following function to the acceleration data:
                // \ddot{x}(t) = a*Math.pow(t-t_apog_i,5) - g
                // Use linear least squares fit.
                // a = \frac{\sum_i (y_i+g)(t_i-t_{\rm apog})^5}{\sum_i (t_i-t_{\rm apog})^{10}}
                double sum_numerator = 0.0;
                double sum_denominator = 0.0;
                for (int j = 0; j < accelMeasurements.size(); j++) {
                    AirbrakesAccelerationMeasurement accelDat = accelMeasurements.get(j);
                    sum_denominator += Math.pow(accelDat.timeStamp - t_apog_trials[i],10);
                    sum_numerator += (accelDat.accelMeasurement + g)*Math.pow(accelDat.timeStamp - t_apog_trials[i],5);
                }
                double resulting_a_coeff = sum_numerator / sum_denominator;
                resulting_R2_values[i] = getR2fromFit_accel(accelMeasurements,resulting_a_coeff,t_apog_trials[i]);
            }
            System.out.println("[JAVA] Fitted Acceleration model to t_apogs: " + Arrays.toString(t_apog_trials));
            System.out.println("[JAVA] R2 values were: " + Arrays.toString(resulting_R2_values));
            double best_t_apog = t_apog_trials[argmax(resulting_R2_values)] + AIRBRAKES_T_APOG_FUDGEDIFF;
            System.out.println("[JAVA] Choosing t_apog " + best_t_apog);
            t_apog = best_t_apog;
            if (override_t_apog >= 0) {
                t_apog = override_t_apog;
                System.out.println("[JAVA] OVERRIDE t_apog is now " + t_apog);
            }

            // Phase 2: fit velocity.
            // We wish to fit the following function once:
            // \dot{x}(t) = a*(t-t_{\rm apog})^3 + b*(t-t_{\rm apog})^2 - g*(t-t_{\rm apog})
            // two parameters: a,b
            // Linear least squares.
            // X^T X matrix is given by
            /*         __                                                              __
                       |   \sum_i (t_i-t_{\rm apog})^6     \sum_i (t_i-t_{\rm apog})^5   |
               X^T X = |                                                                 |
                       |__ \sum_i (t_i-t_{\rm apog})^5     \sum_i (t_i-t_{\rm apog})^4 __|

             */

            double[][] XT_X = new double[2][2];
            for(int i = 0; i < nMeasurements; i++) {
                double t = velMeasurements.get(i).timeStamp;
                XT_X[0][0] += Math.pow(t - t_apog,6);
                XT_X[0][1] += Math.pow(t - t_apog,5);
                XT_X[1][0] += Math.pow(t - t_apog,5);
                XT_X[1][1] += Math.pow(t - t_apog,4);
            }
            // X^T y vector is given by
            /*          __                                                 __
                       |   (t_1-t_{\rm apog})^3  •••  (t_n-t_{\rm apog})^3   |
                 X^T = |                                                     |
                       |__ (t_1-t_{\rm apog})^2  •••  (t_n-t_{\rm apog})^2 __|

             matrix product with

                 y  =  < \dot{x}_1 + g(t_1-t_{\rm apog}) ••• \dot{x}_n + g(t_1-t_{\rm apog}) >

             which gives
                         ___                                                                 ___
                         |    \sum_i (t_i-t_{\rm apog})^3(\dot{x}_i + g(t_i-t_{\rm apog}))     |
                 X^T y = |                                                                     |
                         |__  \sum_i (t_i-t_{\rm apog})^2(\dot{x}_i + g(t_i-t_{\rm apog}))   __|
             */
            double[] XT_y = new double[2];
            for(int i = 0; i < nMeasurements; i++) {
                double t = velMeasurements.get(i).timeStamp;
                double vi = velMeasurements.get(i).velocityMeasurement;
                XT_y[0] += Math.pow(t - t_apog,3)*(vi + g*(t-t_apog));
                XT_y[1] += Math.pow(t - t_apog,2)*(vi + g*(t-t_apog));
            }
            // the coefficients a and b are given by
            /*
                      ___      ___
                      |     a     |
                      |           |   = ( X^T X )^{-1}X^T y
                      |__   b   __|

             */
            double[][] XT_X_inv = inverse2x2Matrix(XT_X);
            double a = XT_X_inv[0][0] * XT_y[0] + XT_X_inv[0][1] * XT_y[1];
            double b = XT_X_inv[1][0] * XT_y[0] + XT_X_inv[1][1] * XT_y[1];
            // THESE ARE THE COEFFICIENTS.

            System.out.println("[JAVA] Fitted Velocity model to t_apogs; coeffs are a " + a + " and b " + b);
            coeffA = a;
            coeffB = b;






            // calculate apogee height.
            double x0 = status.getRocketWorldPosition().getAltitude();
            double v0 = status.getRocketVelocity().z;
            System.out.println("[JAVA] got coeffs: a = " + coeffA + " ; b = " + coeffB + " ; t_apog = " + t_apog + " ; g = " + g);
            alt0 = x0-getAltitudeEstimate(status.getSimulationTime(),0);
            predictedAlt = getAltitudeEstimate(t_apog);
            System.out.println("[JAVA] predicted apogee: " + predictedAlt + " m");
            double conrad_predict = computeFinalAltitude_Conrad(0,status);
            System.out.println("[JAVA] <Conrad> predicted apogee: " + conrad_predict + " m");
            double desiredAlt = Math.floor(predictedAlt/roundToHowMuch)*roundToHowMuch;
            if (overriden_desiredApog > 0) {
                desiredAlt = overriden_desiredApog;
            }
            targetAlt = desiredAlt;
            System.out.println("[JAVA] desired apogee: " + desiredAlt + " m");
            desiredDeltaX = predictedAlt - desiredAlt;

            A0_req = reqDeployedAreaAirbrakes(currentTime+AIRBRAKES_TIME_DELAY, desiredDeltaX);
            airbrakesCtrlStartTime = currentTime + AIRBRAKES_TIME_DELAY;









            // Verify Conrad algorithm
            Astar = A0_req*a_max;
            double hf_guess2 = computeFinalAltitude_Conrad(Astar,status);
            System.out.println("[JAVA] predicted apogee from Conrad when airbrakes are deployed: " + hf_guess2 + " m");
            A.add(Astar);
            K = computeK(Astar,status);
            K *= factorK;


            if (A0_req > 1.0) {
                System.out.println("[JAVA] Got Req A " + A0_req);
                System.out.println("[JAVA] Cannot reach desired apogee with airbrakes.");
                if(DEBUG_AIRBRAKES_ON) {
                    System.out.println("[JAVA] Trying anyway .... setting A to 1");
                    A0_req = 1.0;
                    state = WAIT_FOR_START;
                }
                else {
                    System.out.println("[JAVA] Disabling Airbrakes.");
                    state = INFEASIBLE;
                }
            }

            else {
                System.out.println("[JAVA] Reaching desired apogee is feasible.");
                System.out.println("[JAVA] Required Airbrakes Deployed Fraction: " + A0_req);
                System.out.println("[JAVA] Desired deploy time: " + airbrakesCtrlStartTime + " s");
                state = WAIT_FOR_START;
            }
        }

        else if (state == WAIT_FOR_START){
            if (status.getSimulationTime() >= airbrakesCtrlStartTime){
                state = CONTROLLING_RAMP;
            }
        }
        else if (state == CONTROLLING_RAMP){
            // implement airbrakes control ramp
            // A(t) = 2A_0(t-t_0) for t_0 <= t < t_0 + 0.5s
            if (status.getSimulationTime() >= airbrakesCtrlStartTime) {
                double deployedFraction = 2.0*A0_req * (status.getSimulationTime() - airbrakesCtrlStartTime);
                theAirbrakes.setFracExposed_fudged_for_simulation(deployedFraction);
            }
            if (status.getSimulationTime() >= airbrakesCtrlStartTime + 0.5){
                state = CONTROLLING_PLATEAU;
                lastControlStepTime = status.getSimulationTime();
            }
        }
        else if (state == CONTROLLING_PLATEAU){
            if (status.getSimulationTime() - lastControlStepTime < rate) {
                // don't actually do a control step.
                System.out.print("[JAVA] Airbrakes controller is waiting for next control step.      \r");
                return;
            }

            // airbrakes control plateau
            // A(t) = A_0 for t >= t_0 + 0.5s
            double samplingTime = 1;//status.getSimulationTime()-lastIterTime; // for the moment
            double a_max_fudged = a_max;
            // Control Scheme
            double Ki = ki_factor*2/K;
            double Kp = kp_factor/K;

            // data collection: A, deltaH, deltaA
            A.add(theAirbrakes.getFracExposed_fudged_for_simulation()*a_max_fudged);
            double lastA = A.get(A.size()-1);
            deltaA.add(lastA-Astar);
            timeStampDeltas.add(status.getSimulationTime());


            double current_hf = computeFinalAltitude_Conrad(lastA,status);
            if (patchingAltitude == 0) {
                patchingAltitude = targetAlt-current_hf;
            }
            current_hf = computeFinalAltitude_Conrad(lastA,status);
            double supposedLinearizationPoint = computeFinalAltitude_Conrad(Astar,status);
            deltaH.add(current_hf-targetAlt);
            Hf.add(current_hf);
            //System.out.println("[JAVA] Predicted Altitude = " + current_hf + " m");
            //System.out.println("[JAVA] Predicted Altitude without change = " + supposedLinearizationPoint + " m");
            //System.out.println("[JAVA] Desired Altitude = " + targetAlt + " m");



            // decide integral term
            double IofKplus1 = 0;
            if (
                    ((lastA >= 1) &&
                            (deltaH.get(deltaH.size()-1) >= 0)) ||
                    (lastA <= 1e-5) &&
                            (deltaH.get(deltaH.size()-1) < 0)) {
                IofKplus1 = I.get(I.size()-1);
            }
            else {
                IofKplus1 = I.get(I.size()-1) + 2/K*samplingTime*(deltaH.get(deltaH.size()-1));
            }
            I.add(IofKplus1);

            // calculate the control step
            double nextDeltaA = Kp* deltaH.get(deltaH.size()-1) +Ki*IofKplus1;

            double nextA = Astar + nextDeltaA;
            // actuate
            /*System.out.println("[JAVA] A[k] = " + lastA/a_max);
            System.out.println("[JAVA] ∆A[k] = " + (lastA-Astar)/a_max);
            System.out.println("[JAVA] ∆h[k] = " + deltaH.get(deltaH.size()-1) + " m");
            System.out.println("[JAVA] I[k+1] = " + IofKplus1);
            System.out.println("[JAVA] ∆A[k+1] = " + nextDeltaA/a_max);
            System.out.println("[JAVA] A[k+1] = " + nextA/a_max);*/
            theAirbrakes.setFracExposed_fudged_for_simulation(nextA/a_max_fudged);
            lastControlStepTime = status.getSimulationTime();


            // check if apogee reached
            if (status.getRocketVelocity().z <= 0 || status.apogeeReached){
                state = DONE;
                theAirbrakes.setFracExposed(0);
            }
        }
    }

    public static AirbrakeSet getAirbrakes(SimulationStatus status){
        ArrayList<AirbrakeSet> sets = new ArrayList<>();
        Rocket rocket = status.getConfiguration().getRocket();
        for(Iterator<RocketComponent> it = rocket.iterator(true); it.hasNext(); ){
            RocketComponent component = it.next();

            if(component instanceof AirbrakeSet){
                sets.add((AirbrakeSet) component);
            }
        }
        return sets.get(0);
    }


    public static double computeFinalAltitude_Conrad(double A,SimulationStatus status) {
        final double fudge_factor_conrad = 3.2;
        final double fudged_alt_diff = 13;


        /*  h0 = alt[np.argmin((t-13)**2)] # alt at 13 seconds
            v0 = vel[np.argmin((t-13)**2)] # vel at 13 seconds
            c = rho*rocketCD*refA/2
            blind_estimate = h0 + mass/(2*c)*np.log(v0**2*(c)/g/mass+1) - fudged_alt_diff
            alpha = rho*1.28*airbrakesCtrl.A0_req*airbrakesCtrl.a_max/2
            c += alpha/fudge_factor_conrad
            blind_estimate_for_airbrakes = h0 + mass/(2*c)*np.log(v0**2*(c)/g/mass+1) - fudged_alt_diff
        */


        double h0 = status.getRocketWorldPosition().getAltitude();
        double v0 = status.getRocketVelocity().z;
        double m = mass;
        double c = rho*rocketCd*aRef/2.0;
        c *= cFudge;
        double alpha = rho*airbrakesCd*A/2.0;

        // Fudging
        alpha /= fudge_factor_conrad;

        double hf = h0 + velContribFudge*m/(2.0*(alpha + c))*Math.log((v0*v0*(alpha+c))/g/m+1);
        return hf - fudged_alt_diff + patchingAltitude;
    }

    public static double computeK(double Astar,SimulationStatus status) {
        final double fudge_factor_conrad = 3.2;
        final double fudged_alt_diff = 13;

        double h0 = status.getRocketWorldPosition().getAltitude();
        double m = mass;
        double c = rho*rocketCd*aRef/2.0;
        double v0 = status.getRocketVelocity().z;
        double alphaStar = rho*airbrakesCd*Astar/2.0;
        // Fudging
        alphaStar /= fudge_factor_conrad;

        double K0 = m/2 * (-1/(c+alphaStar)/(c+alphaStar) * Math.log(v0*v0/g/m*(c+alphaStar)+1) + 1/(c+alphaStar)*v0*v0/g/m/(v0*v0/g/m*(c+alphaStar)+1));

        return K0;
    }

    /* computes A_0/A_max fraction given deploy time and desired delta x*/
    public static double reqDeployedAreaAirbrakes(double t_0, double deltaX){
        double a = coeffA;
        double b = coeffB;
        double t0 = t_0;
        double t1 = t_apog;
        double xi = -(Math.pow(a,3)*Math.pow(t0-t1,10)/10.0 + (a*a*b)*Math.pow(t0-t1,9)/3.0 + (3.0*a*b*b - 3.0*a*a*g)*Math.pow(t0-t1,8)/8.0 + (b*b*b - 6.0*a*b*g)*Math.pow(t0-t1,7)/7.0 + (a*g*g - b*b*g)*Math.pow(t0-t1,6)/2.0 + (3.0*b*g*g)*Math.pow(t0-t1,5)/5.0 - (g*g*g)*Math.pow(t0-t1,4)/4.0);
        double local_fudge_factor = deltaX > 40 ? fudge_factor : fudge_factor_2;
        double a_0 = local_fudge_factor*2*mass*g*deltaX/airbrakesCd/rho/xi;
        if(overriden_A0 >= 0) {
            System.out.println("[JAVA] WARNING: OVERRIDING A0 FROM CALCULATION");
            return overriden_A0;
        }
        return max(0,a_0/a_max);
    }

    public static double getVelocityEstimate(double t){
        return coeffA*Math.pow(t-t_apog,3) + coeffB*Math.pow(t-t_apog,2) - g*(t-t_apog);
    }
    public static double getAltitudeEstimate(double t,double alt0_local){
        return coeffA*Math.pow(t-t_apog,4)/4 + coeffB*Math.pow(t-t_apog,3)/3 - g*Math.pow(t-t_apog,2)/2+alt0_local;
    }
    public static double getAltitudeEstimate(double t){
        return getAltitudeEstimate(t,alt0);
    }


    public static boolean shouldStartAirbrakesControlPrep(SimulationStatus status) {
        if (status.getSimulationTime() > EARLIEST_AIRBRAKES_PREP_TIME && !status.apogeeReached && status.getRocketVelocity().length() < START_AIRBRAKES_PREP_VEL) {
            return true;
        }
        else return false;
    }

    public static boolean shouldStartAirbrakesControlPreprocess(SimulationStatus status) {
        if (status.getSimulationTime() > START_AIRBRAKES_PREPROC_TIME && !status.apogeeReached) {
            return true;
        }
        else return false;
    }

    public static double accelModel(double t,double a,double custom_t_apog) {
        return a*Math.pow(t-custom_t_apog,5) - g;
    }

    public static double getR2fromFit_accel(ArrayList<AirbrakesAccelerationMeasurement> data, double a,double custom_t_apog) {
        double[] residuals =  new double[data.size()];
        double ss_res = 0;
        double ss_tot = 0;
        double sum_accel_dat = 0;
        for (int i = 0; i < residuals.length; i++){
            AirbrakesAccelerationMeasurement dat = data.get(i);
            residuals[i] = dat.accelMeasurement - accelModel(dat.timeStamp,a,custom_t_apog);
            ss_res += Math.pow(residuals[i],2);
            sum_accel_dat +=  dat.accelMeasurement;
        }
        double meanAccel = sum_accel_dat/((double) residuals.length);
        for (int i = 0; i < residuals.length; i++) {
            AirbrakesAccelerationMeasurement dat = data.get(i);
            ss_tot += Math.pow(dat.accelMeasurement - meanAccel,2);
        }
        return 1 - ss_res/ss_tot;
    }

    public static int argmin(double[] arr) {
        double lowestval = arr[0];
        int index = 0;
        for (int i = 1; i < arr.length; i++) {
            if (arr[i] < lowestval) {
                lowestval = arr[i];
                index = i;
            }
        }
        return index;
    }
    public static int argmax(double[] arr) {
        double largestval = arr[0];
        int index = 0;
        for (int i = 1; i < arr.length; i++) {
            if (arr[i] > largestval) {
                largestval = arr[i];
                index = i;
            }
        }
        return index;
    }

    public static double[][] inverse2x2Matrix(double[][] A) {
        // give zero matrix if singular
        double a = A[0][0];
        double b = A[0][1];
        double c = A[1][0];
        double d = A[1][1];

        double det = a*d - b*c;
        if (det == 0) {
            return new double[][]{{0,0},{0,0}};
        }
        else {
            double factor = 1.0/det;
            return new double[][]{{factor*d,-factor*c},{-factor*b,factor*a}};
        }
    }


}
