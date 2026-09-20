package edu.mit.rocket_team.zephyrus.control.airbrakes;

import edu.mit.rocket_team.zephyrus.util.RTController;
import edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.Trace;
import edu.mit.rocket_team.zephyrus.util.data.*;
import static edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.elapsed32;
import static edu.mit.rocket_team.zephyrus.control.airbrakes.RTAirbrakesControllerState.*;

/** Translation of RT_Firmware_Libs/airbrakes.h and airbrakes.cpp (see source manifest). */
public class RTAirbrakesController extends RTController {
    public static final int AIRBRAKES_N_MEASUREMENTS = 20;
    public static final int AIRBRAKES_MEASUREMENT_FREQ_HZ = 5;
    public static final float AIRBRAKES_SIMULATION_T_APOG = 35.0f;
    public static final int DEBUG_AIRBRAKES_ON = 1;
    public static final float AIRBRAKES_START_TIME = 13.0f;
    public static final boolean FLAG_DYNAMIC_DESIRED_ALTITUDE = true;
    public static final float SIM_PREDICTED_ALTITUDE = 5046.0f;

    private final float g = 9.81f;

    private float mass = 34.15380231015455f;
    private float rho = 0.736115423712237f;
    private float airbrakesCd = 1.28f;
    private float rocketCd = 0.4843927669074317f;
    private float a_ref = 0.019289796351014733f;
    private float a_max = 0.0066f;
    private float fudge_factor = 3.2f;
    private float fudge_factor_2 = 3.5f;

    private float EARLIEST_AIRBRAKES_PREP_TIME = 4.0f;
    private float START_AIRBRAKES_PREP_VEL = 400.0f;
    private float START_AIRBRAKES_PREPROC_TIME = 12.5f;
    private float AIRBRAKES_TIME_DELAY = 1.0f;
    private float AIRBRAKES_T_APOG_FUDGEDIFF = 1.5f;

    private int roundToHowMuch = 100;

    private float t_apog = 35.5f;
    private float coeffA = -0.0154397511f;
    private float coeffB = -0.3379534959f;

    private float alt0 = 0.0f;
    private float predictedAlt = 0.0f;
    private float desiredDeltaX = 0.0f;

    private float airbrakesCtrlStartTime = 1e10f;
    private float A0_req = 0.0f;

    private float Astar = 0.0f;
    private float patchingAltitude = 0.0f;
    private float velContribFudge = 1.0f;
    private float cFudge = 0.825f;
    private float K = 1;

    private float lastA = 0;
    private float lastDeltaA = 0;
    private float lastDeltaH = 0;
    private float lastHf = 0;
    private float lastI = 0;
  
    private long lastMeasurementTimeMs = 0;
    private float desiredAlt=4550.0f;

    private RTAirbrakesControllerState state;
    private float deployment;

    private RTAirbrakesAccelerationMeasurement[] accelData = new RTAirbrakesAccelerationMeasurement[AIRBRAKES_N_MEASUREMENTS];
    private RTAirbrakesVelocityMeasurement[] velData = new RTAirbrakesVelocityMeasurement[AIRBRAKES_N_MEASUREMENTS];

    private int datIndex;
    private int counter;


    private final Trace trace;
    private RTFudgedAirbrakesData input = new RTFudgedAirbrakesData(0,0,0,false);
    private float flightTime;
    public RTAirbrakesController() { this(new Trace()); }
    public RTAirbrakesController(Trace trace) {
        this.trace = trace;
        for (int i=0; i<AIRBRAKES_N_MEASUREMENTS; i++) {
            accelData[i]=new RTAirbrakesAccelerationMeasurement(0,0);
            velData[i]=new RTAirbrakesVelocityMeasurement(0,0);
        }
        begin();
    }
    @Override public void setup() { begin(); trace.log("airbrakes.setup", "source=FC-included-library"); }
    @Override public void performLoopAction() { update(flightTime, input); }
    @Override public void backdoorFudge(RTFudgedData data) { input = (RTFudgedAirbrakesData)data; }
    public int getSampleCount() { return datIndex; }
    public float getPredictedAltitude() { return predictedAlt; }
    public float getDesiredAltitude() { return desiredAlt; }
    public float getIntegral() { return lastI; }
    public void begin() {
  state = DISABLED;
  deployment = 0.0f;
  datIndex = 0;
  counter = 0;
  patchingAltitude = 0;
}

/* ------------------ Public ------------------ */
    public void update(float t, RTFudgedAirbrakesData status) {
  this.input = status; this.flightTime = t;
  RTAirbrakesControllerState previous = state;
  handleState(t, status);
  trace.log("airbrakes.update", "flight_s="+t+" state="+state+" samples="+datIndex+" deployment="+deployment+" predicted_m="+predictedAlt+" target_m="+desiredAlt+" integral="+lastI);
  if (state != previous) trace.log("airbrakes.transition", "from="+previous+" to="+state);
}

    public float getDeployment() {
  return deployment;
}

    public RTAirbrakesControllerState getState() {
  return state;
}

/* ------------------ Servo replacement ------------------ */
    public void setAirbrakesServo(float deployedFraction) {
  if (deployedFraction < 0.0f) deployedFraction = 0.0f;
  if (deployedFraction > 1.0f) deployedFraction = 1.0f;
  deployment = deployedFraction;
  trace.log("airbrakes.request", "fraction="+deployment);
}

/* ------------------ Helpers ------------------ */
    public float maxf(float a, float b) {
  return (a > b) ? a : b;
}
    public float minf(float a, float b) {
  return (a < b) ? a : b;
}

/* power funcs */
    public float p4(float x){ float x2=x*x; return x2*x2; }
    public float p5(float x){ return p4(x)*x; }
    public float p6(float x){ float x3=x*x*x; return x3*x3; }
    public float p7(float x){ return p6(x)*x; }
    public float p8(float x){ float x4=p4(x); return x4*x4; }
    public float p9(float x){ return p8(x)*x; }
    public float p10(float x){ float x5=p5(x); return x5*x5; }

    public float pow5f_fast(float x){ return p5(x); }
    public float pow10f_fast(float x){ return p10(x); }

/* accel model */
    public float accelModel(float t, float a, float custom_t_apog) {
  float dt = t - custom_t_apog;
  return a * pow5f_fast(dt) - g;
}

    public float getR2fromFit_accel(RTAirbrakesAccelerationMeasurement[] data,
                                              int n,
                                              float a,
                                              float custom_t_apog) {
  float ss_res = 0.0f, ss_tot = 0.0f, sum_y = 0.0f;

  for (int i = 0; i < n; i++) {
    float y = data[i].accelMeasurement;
    float yh = accelModel(data[i].timeStamp, a, custom_t_apog);
    float r = y - yh;
    ss_res += r * r;
    sum_y += y;
  }

  float mean = sum_y / n;
  for (int i = 0; i < n; i++) {
    float d = data[i].accelMeasurement - mean;
    ss_tot += d * d;
  }

  if (ss_tot == 0.0f) return 0.0f;
  return 1.0f - (ss_res / ss_tot);
}

    public int argmax(float[] arr, int n) {
  float best = arr[0];
  int idx = 0;
  for (int i = 1; i < n; i++) {
    if (arr[i] > best) {
      best = arr[i];
      idx = i;
    }
  }
  return idx;
}

    public boolean inverse2x2Matrix(float[][] A, float[][] Ainv) {
  float det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
  if (Math.abs(det) <= 1e-6f) return false;

  float f = 1.0f/det;
  Ainv[0][1] = -f*A[0][1];
  Ainv[1][0] = -f*A[1][0];
  Ainv[0][0] = f*A[1][1];
  Ainv[1][1] = f*A[0][0];
  return true;
}

/* ------------------ Physics ------------------ */
    public float reqDeployedAreaAirbrakes(float t_0, float deltaX) {
  float a = coeffA;
  float b = coeffB;
  float dt = (t_0 - t_apog);

  float a2=a*a,a3=a2*a,b2=b*b,b3=b2*b,g2=g*g,g3=g2*g;

  float term =
    (a3)*p10(dt)/10.0f +
    (a2*b)*p9(dt)/3.0f +
    (3*a*b2-3*a2*g)*p8(dt)/8.0f +
    (b3-6*a*b*g)*p7(dt)/7.0f +
    (a*g2-b2*g)*p6(dt)/2.0f +
    (3*b*g2)*p5(dt)/5.0f -
    (g3)*p4(dt)/4.0f;

  float xi = -term;

  float local_fudge = (deltaX > 40.0f) ? fudge_factor : fudge_factor_2;

  float denom = airbrakesCd * rho * xi;
  if (denom == 0.0f || a_max == 0.0f) return 0.0f;

  float a_0 = local_fudge * 2.0f * mass * g * deltaX / denom;
  float tentative = minf(maxf(0.0f, a_0 / a_max),1.0f);
  if (tentative == 0.0f) {
    tentative = 1.0f; // to have it deploy : )
  }
  return tentative;
}

    public float computeFinalAltitude_Conrad(float A, float h0, float v0) {
  float m = mass;
  float c = rho * rocketCd * a_ref / 2.0f;
  c *= cFudge;
  float alpha = rho * airbrakesCd * A / 2.0f;

  float hf = h0 + velContribFudge * m / (2.0f * (alpha + c)) *
    (float)Math.log((v0 * v0 * (alpha + c)) / g / m + 1.0f);

  return hf - 13.0f + patchingAltitude;
}

    public float computeK(float Astar, float h0, float v0) {
  float m = mass;
  float c = rho * rocketCd * a_ref / 2.0f;
  float alpha = rho * airbrakesCd * Astar / 2.0f;

  alpha /= 3.2f;

  float K0 = m/2 * (-1/((c+alpha)*(c+alpha))*(float)Math.log(v0*v0/g/m*(c+alpha)+1)
             + 1/(c+alpha)*v0*v0/g/m/(v0*v0/g/m*(c+alpha)+1));

  float ret = Float.isNaN(K0/2.0f) ? 1e10f : K0/2.0f;

  return ret;
}

/* ------------------ Start conditions ------------------ */
    public boolean shouldStartAirbrakesControlPrep(float t, RTFudgedAirbrakesData s) {
  return (t > EARLIEST_AIRBRAKES_PREP_TIME) && (!s.apogeeReached) && (s.vel_z < START_AIRBRAKES_PREP_VEL);
}

    public boolean shouldStartAirbrakesControlPreprocess(float t, RTFudgedAirbrakesData s) {
  return (t > START_AIRBRAKES_PREPROC_TIME) && (!s.apogeeReached);
}

/* ------------------ STATE MACHINE ------------------ */
    public void handleState(float t, RTFudgedAirbrakesData status) {

  if (state == DISABLED) {
    setAirbrakesServo(0.0f);
    state = shouldStartAirbrakesControlPrep(t, status) ? PREP : DISABLED;
    if (state == PREP) { datIndex = 0; counter = 0; }
  }

  else if (state == PREP) {
    long nowMs = trace.millis();
    long periodMs = 1000 / AIRBRAKES_MEASUREMENT_FREQ_HZ;

    if (datIndex < AIRBRAKES_N_MEASUREMENTS &&
        elapsed32(nowMs, lastMeasurementTimeMs) >= periodMs) {

        lastMeasurementTimeMs = nowMs;
        accelData[datIndex].setData(t, status.accel_z);
        velData[datIndex].setData(t, status.vel_z);
        datIndex++;
        trace.log("airbrakes.sample", "index="+(datIndex-1)+" flight_s="+t+" altitude_m="+status.altitude+" velocity_mps="+status.vel_z+" accel_mps2="+status.accel_z);
    }

    if (datIndex >= AIRBRAKES_N_MEASUREMENTS) state = PREPROCESS;
    else state = shouldStartAirbrakesControlPreprocess(t, status) ? PREPROCESS : PREP;
  }

  else if (state == PREPROCESS) {

    float[] t_apog_trials = {34.0f,35.0f,36.0f};
    float[] R2 = new float[3];

    for(int i=0;i<3;i++){
      float sum_num=0,sum_den=0;
      for(int j=0;j<datIndex;j++){
        float dt=accelData[j].timeStamp-t_apog_trials[i];
        sum_den+=pow10f_fast(dt);
        sum_num+=(accelData[j].accelMeasurement+g)*pow5f_fast(dt);
      }
      float a_coeff = (sum_den!=0)?(sum_num/sum_den):0;
      R2[i]=getR2fromFit_accel(accelData,AIRBRAKES_N_MEASUREMENTS,a_coeff,t_apog_trials[i]);
    }

    int best=argmax(R2,3);
    t_apog=t_apog_trials[best]+AIRBRAKES_T_APOG_FUDGEDIFF;

    float conrad=computeFinalAltitude_Conrad(0,status.altitude,status.vel_z);
    patchingAltitude=SIM_PREDICTED_ALTITUDE-conrad;
    predictedAlt=computeFinalAltitude_Conrad(0,status.altitude,status.vel_z);
    if (FLAG_DYNAMIC_DESIRED_ALTITUDE) {
      desiredAlt = (float)Math.floor(predictedAlt/100.0f)*100.0f;
    }
    desiredDeltaX=predictedAlt-desiredAlt;

    boolean tooLate=t> AIRBRAKES_START_TIME-0.25f;
    airbrakesCtrlStartTime= tooLate ? t+AIRBRAKES_TIME_DELAY : AIRBRAKES_START_TIME;

    A0_req=reqDeployedAreaAirbrakes(airbrakesCtrlStartTime,desiredDeltaX);

    Astar=A0_req*a_max;
    lastA=Astar;
    K=computeK(Astar,status.altitude,status.vel_z);

    state=WAIT_FOR_START;
  }

  else if (state == WAIT_FOR_START) {
    if (t>=airbrakesCtrlStartTime) state=CONTROLLING_RAMP;

    if (status.apogeeReached){
      state=DONE;
      setAirbrakesServo(0);
    }
  }

  else if (state == CONTROLLING_RAMP) {

    if (t>=airbrakesCtrlStartTime) {
      float deployed=2.0f*A0_req*(t-airbrakesCtrlStartTime);
      setAirbrakesServo(deployed);
    }

    if (t>=airbrakesCtrlStartTime+0.5f){
      state=CONTROLLING_PLATEAU;
      setAirbrakesServo(A0_req);
    }

    if (status.apogeeReached){
      state=DONE;
      setAirbrakesServo(0);
    }
  }

  else if (state == CONTROLLING_PLATEAU) {

    float Ki=2.0f/K, Kp=1.0f/K;

    lastA=deployment*a_max;
    float hf=computeFinalAltitude_Conrad(lastA,status.altitude,status.vel_z);

    lastDeltaH=hf-desiredAlt;

    float I;
    if(((lastA/a_max>=1.0f)&&(lastDeltaH>=0))||((lastA/a_max<=1e-5)&&(lastDeltaH<0)))
      I=lastI;
    else
      I=lastI+2.0f/K*lastDeltaH;

    lastI=I;

    float nextA=Astar+(Kp*lastDeltaH+Ki*lastI);
    setAirbrakesServo(nextA/a_max);

    if(status.vel_z<=0||status.apogeeReached){
      state=DONE;
      setAirbrakesServo(0);
    }
  }
}
}
