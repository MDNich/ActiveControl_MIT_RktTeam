import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
import matplotlib as mpl
rcParams['font.family'] = 'serif'
rcParams['font.sans-serif'] = ['Computer Modern']
rcParams['text.usetex'] = True
blue = "#08357E"
red = "#7A0101"
lred = "#AB0000"
lblue = "#8AAEE8"
green = "#38761D"
orange = "#ff7f0e"
rcParams['axes.prop_cycle'] = mpl.cycler(color=[blue,red,green,lblue,lred,orange]) 
import scipy.signal as spsig

ts,accel,alt,roll = tuple(np.array(np.genfromtxt("Zphr_Test_Launch_Dummt_Data_100Hz_v1.csv",delimiter=",")).T)
vel = np.array(spsig.savgol_filter(alt,10,2,1))/(ts[10]-ts[1])*10

apog_index = np.nanargmin(vel**2)
t_apog_real = ts[apog_index]
alt_apog_real = alt[apog_index]
print("Got Apogee Time {} at altitude {}".format(t_apog_real,alt_apog_real))

apogeeLog = []
for i in range(len(ts)):
    apogeeLog.append(False if i < apog_index else True)

apogeeLog = np.array(apogeeLog)



### AIRBRAKES ALGORITHM PREP ### 
AIRBRAKES_N_MEASUREMENTS = 20
AIRBRAKES_MEASUREMENT_FREQ_HZ = 5
AIRBRAKES_SIMULATION_T_APOG = 35.0
DEBUG_AIRBRAKES_ON = 1
AIRBRAKES_START_TIME = 13.0
FLAG_DYNAMIC_DESIRED_ALTITUDE = True
EARLIEST_AIRBRAKES_PREP_TIME = 4.0
START_AIRBRAKES_PREP_VEL = 400.0
START_AIRBRAKES_PREPROC_TIME = 12.5
AIRBRAKES_T_APOG_FUDGEDIFF = 1.5

roundToHowMuch = 100

t_apog = 35.5
coeffA = -0.0154397511
coeffB = -0.3379534959

alt0 = 0.0
predictedAlt = 0.0
desiredDeltaX = 0.0

airbrakesCtrlStartTime = 1e10
A0_req = 0.0

Astar = 0.0
patchingAltitude = 0.0
velContribFudge = 1.0
cFudge = 0.825
K = 1

lastA = 0
lastDeltaA = 0
lastDeltaH = 0
lastHf = 0
lastI = 0

mass = 34.15380231015455
rho = 0.736115423712237
airbrakesCd = 1.28
rocketCd = 0.4843927669074317
a_ref = 0.019289796351014733
a_max = 0.008226048064
fudge_factor = 3.2
fudge_factor_2 = 3.5

DISABLED = 0
PREP = 1
PREPROCESS = 2 
WAIT_FOR_START = 3
CONTROLLING_RAMP = 4
CONTROLLING_PLATEAU = 5
DONE = 6
INFEASIBLE = 7

state = DISABLED
lastMeasurementTimeMs = 0
deployment = 0

def shouldStartAirbrakesControlPrep(t,apogeeReached,velocity):
  return (t > EARLIEST_AIRBRAKES_PREP_TIME) & (not apogeeReached) & (velocity < START_AIRBRAKES_PREP_VEL)

def shouldStartAirbrakesControlPreprocess(t,apogeeReached):
    return (t > START_AIRBRAKES_PREPROC_TIME) & (not apogeeReached)

def accelModel(t, a, custom_t_apog):
    dt = t - custom_t_apog
    return a * np.power(dt,5) - g


def getR2fromFit_accel(data,n,a,custom_t_apog):
    ss_res = 0.0 
    ss_tot = 0.0
    sum_y = 0.0
    for i in range(n):
        y = data[i][1]
        yh = accelModel(data[i][0], a, custom_t_apog)
        r = y - yh
        ss_res += r * r
        sum_y += y
    

    mean = sum_y / n
    for i in range(n):
        d = data[i][1] - mean
        ss_tot += d * d
    

    if (ss_tot == 0.0):
        return 0.0

    return 1.0 - (ss_res / ss_tot)

datIndex = 0
counter = 0
accelData = [[0,0]] * AIRBRAKES_N_MEASUREMENTS
velData = [[0,0]] * AIRBRAKES_N_MEASUREMENTS
g = 9.81
patchingAltitude = 0

airbrakesServoVal = 0
def setAirbrakesServo(time,val):
    if val > 1:
        val = 1
    if val < 0:
        val = 0
    global airbrakesServoVal
    global deployment
    airbrakesServoVal = val
    deployment = val

def handleAirbrakesState(time, altitude, velocity, acceleration, apogeeReached):
    global state, accelData, velData,datIndex,patchingAltitude, lastMeasurementTimeMs,airbrakesCtrlStartTime,A0_req,K,deployment,desiredAlt,lastI,lastA,lastHf,Astar,lastDeltaH
    currentTime = time
    t = time
    everyHowMany = 1000/AIRBRAKES_MEASUREMENT_FREQ_HZ # 5 Hz -> 1 s/ 5 s^{-1} -> 1/5 times per sec -> 1/5*1000 times per ms
    nMeasurements = AIRBRAKES_N_MEASUREMENTS

    # if state is disabled, check if should start prep
    if (state == DISABLED):
        setAirbrakesServo(time, 0.0)
        state = PREP if shouldStartAirbrakesControlPrep(time, apogeeReached, velocity) else DISABLED
        if (state == PREP):
            datIndex = 0 
            counter = 0
    

    elif (state == PREP):
        nowMs = time*1000
        periodMs = 1000 / AIRBRAKES_MEASUREMENT_FREQ_HZ

        if (datIndex < AIRBRAKES_N_MEASUREMENTS) & ((nowMs - lastMeasurementTimeMs) >= periodMs):
            lastMeasurementTimeMs = nowMs
            accelData[datIndex]   = [acceleration, t]
            velData[datIndex]     = [velocity, t]
            datIndex += 1
            

        if (datIndex >= AIRBRAKES_N_MEASUREMENTS): 
            state = PREPROCESS
        else:
            state = PREPROCESS if shouldStartAirbrakesControlPreprocess(time, apogeeReached) else PREP
    

    elif (state == PREPROCESS):
        t_apog_trials = [27.0,28.0,29.0]
        R2 = [0,0,0]

        for i in range(3):
            sum_num=0
            sum_den=0
            for j in range(datIndex):
                dt=accelData[j][0]-t_apog_trials[i]
                sum_den += np.power(dt,10)
                sum_num += (accelData[j][1]+g)*np.power(dt,5)
            
            a_coeff = (sum_num/sum_den) if (sum_den!=0) else 0
            R2[i] = getR2fromFit_accel(accelData,AIRBRAKES_N_MEASUREMENTS,a_coeff,t_apog_trials[i])
        

        best = np.argmax(R2)
        t_apog = t_apog_trials[best] + AIRBRAKES_T_APOG_FUDGEDIFF

        conrad = computeFinalAltitude_Conrad(0,altitude,velocity)
        patchingAltitude = 4637 - conrad
        predictedAlt = computeFinalAltitude_Conrad(0,altitude,velocity)
        if (FLAG_DYNAMIC_DESIRED_ALTITUDE):
            desiredAlt = np.floor(predictedAlt/100.0)*100.0
        
        desiredDeltaX = predictedAlt - desiredAlt

        tooLate = time > AIRBRAKES_START_TIME - 0.25
        airbrakesCtrlStartTime = time + AIRBRAKES_TIME_DELAY if tooLate else AIRBRAKES_START_TIME

        A0_req = reqDeployedAreaAirbrakes(airbrakesCtrlStartTime,desiredDeltaX)

        Astar = A0_req*a_max
        lastA = Astar
        K = computeK(Astar,altitude,velocity)
        print("END OF PREPROCESS:")
        print("A0_req = {}".format(A0_req))
        print("K = {}".format(K))
        print("predictedAlt = {}".format(predictedAlt))
        print("desiredAlt = {}".format(desiredAlt))
        print("desiredDeltaX = {}".format(desiredDeltaX))
        print("t_apog = {}".format(t_apog))

        state = WAIT_FOR_START

    elif (state == WAIT_FOR_START):
        if (t >= airbrakesCtrlStartTime):
            state = CONTROLLING_RAMP

        if (apogeeReached):
            state = DONE
            setAirbrakesServo(time,0)
        
    

    elif (state == CONTROLLING_RAMP):
        if (t >= airbrakesCtrlStartTime):
            deployed = 2.0*A0_req*(t-airbrakesCtrlStartTime)
            setAirbrakesServo(time,deployed)
        

        if (t >= airbrakesCtrlStartTime + 0.5):
            state = CONTROLLING_PLATEAU
            setAirbrakesServo(time,A0_req)
        

        if (apogeeReached):
            state = DONE
            setAirbrakesServo(time,0)
        
    

    elif (state == CONTROLLING_PLATEAU):
        Ki = 2.0/K
        Kp = 1.0/K

        lastA = deployment*a_max
        hf = computeFinalAltitude_Conrad(lastA,altitude,velocity)

        lastDeltaH = hf - desiredAlt

        I = 0
        if ((lastA/a_max>=1.0) & (lastDeltaH>=0)) | ((lastA/a_max<=1e-5)&(lastDeltaH<0)):
            I = lastI
        else:
            I = lastI+2.0/K*lastDeltaH

        lastI = I

        nextA = Astar + (Kp*lastDeltaH + Ki*lastI)
        setAirbrakesServo(time,nextA/a_max)

        if (velocity <= 0) | apogeeReached:
            state = DONE
            setAirbrakesServo(time,0)
        
    
    
def computeFinalAltitude_Conrad(A, h0, v0):
    global patchingAltitude
    m = mass
    c = rho * rocketCd * a_ref / 2.0
    c *= cFudge
    alpha = rho * airbrakesCd * A / 2.0

    hf = h0 + velContribFudge * m / (2.0 * (alpha + c)) * np.log((v0 * v0 * (alpha + c)) / g / m + 1.0)

    return hf - 13.0 + patchingAltitude


def computeK(Astar, h0, v0):
    print("Called COMPUTE_K for Astar {} h0 {} v0 {}".format(Astar,h0,v0))
    m = mass
    c = rho * rocketCd * a_ref / 2.0
    alpha = rho * airbrakesCd * Astar / 2.0

    alpha /= 3.2

    K0 = m/2 * (-1/((c+alpha)*(c+alpha))*np.log(v0*v0/g/m*(c+alpha)+1) + 1/(c+alpha)*v0*v0/g/m/(v0*v0/g/m*(c+alpha)+1))

    print("Obtained prelim K {}".format(K0))
    ret = 1e10 if (np.isnan(K0/2.0)) else K0/2.0

    return ret

def reqDeployedAreaAirbrakes(t_0, deltaX):
    a = coeffA
    b = coeffB
    dt = (t_0 - t_apog)

    a2=a*a
    a3=a2*a
    b2=b*b
    b3=b2*b
    g2=g*g
    g3=g2*g

    term = (a3)*np.power(dt,10)/10.0 + (a2*b)*np.power(dt,9)/3.0 + \
           (3*a*b2-3*a2*g)*np.power(dt,8)/8.0 + (b3-6*a*b*g)*np.power(dt,7)/7.0 + \
           (a*g2-b2*g)*np.power(dt,6)/2.0 + (3*b*g2)*np.power(dt,5)/5.0 - \
           (g3)*np.power(dt,4)/4.0

    xi = -term

    local_fudge = fudge_factor if (deltaX > 40.0) else fudge_factor_2

    denom = airbrakesCd * rho * xi
    if (denom == 0.0) | (a_max == 0.0):
        return 0.0

    a_0 = local_fudge * 2.0 * mass * g * deltaX / denom
    return max(0.0, a_0 / a_max)

####

### AIRBRAKES ALGO RUN ###

stateLog = []
airbrakesLog = []
for i in range(len(ts)):
    # Simulate the Airbrakes

    handleAirbrakesState(ts[i],alt[i],vel[i],accel[i],apogeeLog[i])
    stateLog.append(state)
    airbrakesLog.append(airbrakesServoVal)


stateLog = np.array(stateLog)
airbrakesLog = np.array(airbrakesLog)

np.savetxt("airbrakesLog.csv", np.column_stack((stateLog, airbrakesLog)), delimiter=",", header="STATE,AirbrakesDeployment", comments="")





fig,axs = plt.subplots(nrows=3,sharex='col',figsize=(8.5,8.5))
ax1,ax2,ax3 = tuple(axs)

ax1.plot(ts,alt)
ax1.hlines(alt_apog_real,0,200,linestyle='dotted')
ax2.plot(ts,vel)
ax3.plot(ts,airbrakesLog)
ax4 = ax3.twinx()
ax4.plot(ts,stateLog,c=red)

[ax.set_ylim(*ax.get_ylim()) for ax in axs]
[ax.set_xlim(0,t_apog_real) for ax in axs]
#[ax.vlines(t_apog_real,*ax.get_ylim(),linestyle='dotted') for ax in axs]


ax1.set_ylabel("Alt (m)")
ax2.set_ylabel("Vel (m/s)")
ax3.set_ylabel("Airbrakes Deployment Percentage")
ax4.set_ylabel("Airbrakes State")
ax3.set_xlabel("Time (s)")
plt.tight_layout()
plt.show()
