#!/usr/local/python3

# MUST BE RUN FROM TOP LEVEL WORKING DIRECTORY: 'sim'.
import os
from time import sleep

import scipy.signal

from src.py.test.Simplified_plant_dynamics import *
import numpy as np
import jpype
from src.py.orhelper.util import *
from src.py.orhelper.ORpy import *

#os.chdir('../../../')
import asyncio
import src.py.orhelper._orhelper as orhelper
from src.py.orhelper._enums import FlightDataType as dT
from src.py.etc.logcoloring import ColorHandler
from matplotlib import rcParams
rcParams['font.family'] = 'serif'
rcParams['font.sans-serif'] = ['Computer Modern']
rcParams['text.usetex'] = True


import logging
LOG = logging.getLogger()
LOG.setLevel(logging.DEBUG)
for handler in LOG.handlers:
    LOG.removeHandler(handler)

logging.getLogger().addHandler(ColorHandler())

# match your system as needed.
#os.environ['JAVA_HOME'] = '/opt/homebrew/Cellar/openjdk/23.0.2'
os.environ['CLASSPATH'] = './out/OpenRocket.jar'

"""
KP_VEL = 3
KI_VEL = 0#.1#0.75
KD_VEL = .1
# ANG PID
KP_ANG = 2
KI_ANG = 0#.1#0.75
KD_ANG = 1
"""



'''
KP_ANG = 0.15
KI_ANG = 0.0005#.1#0.75
KD_ANG = 2.1
'''

'''
KP_ANG = 7e3
KI_ANG = 2.8
KD_ANG = 1e5
'''


'''
KP_ANG = 1e3
KI_ANG = 4e0#2.8
KD_ANG = 1e4
'''

'''
KP_ANG = 0.1
KI_ANG = 0#2.8#.1#0.75
KD_ANG = 1'''





'''
GOOD FOR OLD JXX
KP_ANG = 0.194
KI_ANG = 0
KD_ANG = 0.0314



KP_ANG = 0.2
KI_ANG = 0
KD_ANG = 0.0175
'''''
import threading

JXX = 0.016
OVERRIDE_JXX_CALC = True

VELMINTHRESH = 20
TURBULENCE = 0
USE_TABS = True
CONST_FIXED = 0
# VEL PID
KP_VEL = 10
KI_VEL = 0.1#.1#0.75
KD_VEL = 1
# ANG PID
KP_ANG = 0.189
KI_ANG = 0
KD_ANG = 0.0377
# OTHER PARAMS
INI_ROT_VEL = 0
DESIRED_ROT_VEL = 0
DESIRED_ROT_ANG = 0
overrideI = False
getPID_from_plant = False
useVelocityPID = False
usePositionPID = True
showControlCutoffLine = useVelocityPID or usePositionPID#True
USE_RK6 = True
ROUND_5 = False
SERVO_STEP_COUNT = int(1024)#int(1024) # Large number means very small steps, effectively continuous.
DEBUG_JAVA_MSG = False
DEG_PER_SEC = 300
REFRESH_TIME = 2e-2
ALTITUDE_NOISE_AMPLITUDE = 0
MASTER_NOISE_OVERRIDE = True


WIND_EVENT_1_TIMESTAMP = 0.7
WIND_EVENT_2_TIMESTAMP = 1.5
WIND_EVENT_3_TIMESTAMP = 3
WIND_EVENT_1_GUST = 5e0
WIND_EVENT_2_GUST = -7e0
WIND_EVENT_3_GUST = 5e0


if usePositionPID:
    DESIRED_ROT_VEL = 0 # If using position PID, we want to stabilize at 0 rad/s, so no rotation.
    #if KP_VEL == 10:
    #	KP_VEL = 7


if not usePositionPID:
    KP_ANG = 0
    KI_ANG = 0
    KD_ANG = 0

if not useVelocityPID:
    KP_VEL = 0
    KI_VEL = 0
    KD_VEL = 0

figPath = 'dat/zephy_testlaunch/pdf/turb{}_Tabs{}_VEL_PID_KP{}_KI{}_KD{}_desiredVel{}_iniVel{}_ANG_PID_KP{}_KI{}_KD{}_constInput{}_desiredPos{}_servoSteps{}.pdf'.format(TURBULENCE,'YES' if USE_TABS else 'NO',KP_VEL,KI_VEL,KD_VEL,DESIRED_ROT_VEL,INI_ROT_VEL,KP_ANG,KI_ANG,KD_ANG,CONST_FIXED,DESIRED_ROT_ANG,SERVO_STEP_COUNT)
CSVSAVEPATH = 'dat/zephy_testlaunch/csv/run_turb{}_Tabs{}_VEL_PID_KP{}_KI{}_KD{}_desiredVel{}_iniVel{}_ANG_PID_KP{}_KI{}_KD{}_constInput{}_desiredPos{}_servoSteps{}.csv'.format(TURBULENCE,'YES' if USE_TABS else 'NO',KP_VEL,KI_VEL,KD_VEL,DESIRED_ROT_VEL,INI_ROT_VEL,KP_ANG,KI_ANG,KD_ANG,CONST_FIXED,DESIRED_ROT_ANG,SERVO_STEP_COUNT)


# Start
instance, orh, or_obj = startOR()

# Import classes
simStatClass = or_obj.simulation.SimulationStatus
coordClass = or_obj.util.Coordinate
worldCoordClass = or_obj.util.WorldCoordinate
quatClass = or_obj.util.Quaternion

# Load rocket
doc, rktObj = loadRocket(orh, 'zephy_testlaunch.ork')

newCtrl = or_obj.simulation.listeners.NewControlStepListener
airbrakesCtrl = or_obj.simulation.listeners.AirbrakesControllerListener

newCtrl.theRocket = rktObj
airbrakesCtrl.theRocket = rktObj


# load flight conf
flightConfig = doc.getSelectedConfiguration()
logging.info("Motor identifier: ")
logging.info(flightConfig)

# load sim
sim = doc.getSimulation(0)
logging.warning("loaded document + simulation")

datPath = 'dat/simResults/zephy_testlaunch_out_long.csv'
#figPath = 'dat/simResults/zephy_testlaunch_out_long.pdf'


# Get all components, filter for fins.
def get_fins(rocket):
    fins = []
    for component in rocket.iterator(True):  # True = depth-first search
        if isinstance(component, jpype.JClass("info.openrocket.core.rocketcomponent.FinSet")):
            fins.append(component)
    return fins

finList = get_fins(rktObj)
#print("Got Fins from Rocket:")
#for fin in finList:
#    print(f"""
#        Fin Name: {fin.getName()}
#        Number: {fin.getFinCount()}
#        Span: {fin.getSpan()} m
#        Thickness: {fin.getThickness()} m
#        Cant Angle: {fin.getCantAngle()} rad
#        """)


finNames = [fin.getName() for fin in finList]
index = 0
theFinIndexToModify = -1
for fni in finNames:
    if not USE_TABS:
        if fni == "FINS_TWIST":
            theFinIndexToModify = index
            break
    else:
        if fni == "Swept Fins (Tab Ctrlled)":
            theFinIndexToModify = index
            break
    index += 1

finToPlayWith = finList[theFinIndexToModify]




verboseMode = False

logging.error("RUNNING A NEW SIMULATION")

import pandas as pd


if True:
    # startParams
    alt0 = 0.01 # m
    v0x = 0 # m/s
    v0y = 0 # m/s
    v0z = 0.1 # m/s

    rotAxis = coordClass(0,0,1) # z axis, for now
    angle = 0.0

    omega0x = 0.0
    omega0y = 0.0
    omega0z = INI_ROT_VEL


    initialPropDict = {
        "positionX" : 0,
        "positionY" : 0,
        "positionZ" : alt0,
        "positionW" : 12500,
        "position": coordClass(0,0,alt0,12500), # Coordinate object
        "positionPrint": coordClass(0,0,alt0,12500).pythonOutputStr(), # Coordinate object
        "worldPos": worldCoordClass(28.61,-80.6,100.125), # WorldCoordinate object
        "velocityX": v0x, # Coordinate object
        "velocityY": v0y, # Coordinate object
        "velocityZ": v0z, # Coordinate object
        "velocityW": 0, # Coordinate object
        "velocity": coordClass(v0x,v0y,v0z,0), # Coordinate object
        "velocityPrint": coordClass(v0x,v0y,v0z,0).pythonOutputStr(), # Coordinate object
        "orientRotAxisX" : rotAxis.x,
        "orientRotAxisY" : rotAxis.y,
        "orientRotAxisZ" : rotAxis.z,
        "orientRotAxis" : rotAxis,
        "orientAngle" : angle,
        "orient"  : quatClass.rotation(rotAxis, angle), # Quaternion object
        "orientPrint"  : quatClass.rotation(rotAxis, angle).printAxisAngle(), # Quaternion object
        "rotVelX"  : omega0x, # Coordinate object
        "rotVelY"  : omega0y, # Coordinate object
        "rotVelZ"  : omega0z, # Coordinate object
        "rotVel"  : coordClass(omega0x,omega0y,omega0z), # Coordinate object
        "liftoff"  : True, # boolean
        "apogee"   : False, # boolean
        "motorIgn" : True, # boolean
        "lnchRdClr": True, # boolean
        "time" : 0.0, # double
    }

    dictList = [initialPropDict]
    currenPropDict = initialPropDict.copy()
    times = [0]
    heightTime = [initialPropDict["positionZ"]]
    vertVelTime = [initialPropDict["velocityZ"]]

    runTime = 7 # s
    prefDt = 1e-3 # s/cycle
    likelyDt = prefDt
    dtList = []

    import time
    iniTime = time.time()
    logging.info("Begin simulation prep.")
    apogeeFound = False
    motorEnded = False
    motorEndLoc = 0
    timeStep=prefDt
    #listener_array = [newCtrl()]



    # provide initial conditions

    simStatClass = or_obj.simulation.SimulationStatus

    simOptions = sim.getOptions()

    simOptions.setWindTurbulenceIntensity(TURBULENCE)


    theNextSimulationStatus = simStatClass(flightConfig, sim.getOptions().toSimulationConditions())

    theNextSimulationStatus.simulationConditions.setTimeStep(timeStep)

    propDict = initialPropDict
    theNextSimulationStatus.setRocketPosition(propDict["position"])
    theNextSimulationStatus.setRocketWorldPosition(propDict["worldPos"])
    theNextSimulationStatus.setRocketVelocity(propDict["velocity"])
    theNextSimulationStatus.setRocketOrientationQuaternion(propDict["orient"])
    theNextSimulationStatus.setRocketRotationVelocity(propDict["rotVel"])
    theNextSimulationStatus.liftoff = propDict["liftoff"]
    theNextSimulationStatus.apogeeReached = propDict["apogee"]
    theNextSimulationStatus.motorIgnited = propDict["motorIgn"]
    theNextSimulationStatus.launchRodCleared = propDict["lnchRdClr"]

    if(verboseMode):
        print("== INITIAL CONDITIONS ==")
        for key in propDict.keys():
            print("Key {}:".format(key))
            print(propDict[key])
        print("== END INITIAL CONDITIONS ==")

    #newCtrl.initialStat = theNextSimulationStatus
    #newCtrl.iniVel = v0z

    airbrakesCtrl.initialStatus = theNextSimulationStatus




    # Need to do this otherwise exact same numbers will be generated for each identical run
    sim.getOptions().randomizeSeed()

    newCtrl.theFinsToModify = finToPlayWith
    newCtrl.simulateUsingTabs = USE_TABS

    newCtrl.desiredRotVel = DESIRED_ROT_VEL
    newCtrl.desiredRotAng = DESIRED_ROT_ANG
    newCtrl.IdecayFactor = 1


    newCtrl.amplitude_randomness_size = ALTITUDE_NOISE_AMPLITUDE

    newCtrl.WIND_EVENT_1_TIMESTAMP = WIND_EVENT_1_TIMESTAMP
    newCtrl.WIND_EVENT_2_TIMESTAMP = WIND_EVENT_2_TIMESTAMP
    newCtrl.WIND_EVENT_3_TIMESTAMP = WIND_EVENT_3_TIMESTAMP
    newCtrl.WIND_EVENT_1_GUST = WIND_EVENT_1_GUST
    newCtrl.WIND_EVENT_2_GUST = WIND_EVENT_2_GUST
    newCtrl.WIND_EVENT_3_GUST = WIND_EVENT_3_GUST

    newCtrl.MASTER_NOISE_OVERRIDE = MASTER_NOISE_OVERRIDE

    newCtrl.velocityPIDon = useVelocityPID
    newCtrl.positionPIDon = usePositionPID
    newCtrl.SERVO_REFRESH_TIME = REFRESH_TIME
    newCtrl.roundToNearest5 = ROUND_5

    newCtrl.flagPrintDebugMsg = DEBUG_JAVA_MSG

    newCtrl.FLAG_OVERRIDE_JXX = OVERRIDE_JXX_CALC
    newCtrl.OVERRIDEN_JXX = JXX


    newCtrl.constFixed = CONST_FIXED
    newCtrl.degPerSec = DEG_PER_SEC

    # get PID coefficients from algorithm generating.

    ts_des = 0.8 # Desired settling time of 0.5 seconds
    Mp_des = 0.2 # Desired max peak of less than 20%
    s0 = set_dominant_poles(ts_des, Mp_des)
    #print(f'Desired Poles: {s0} and its conjugate')

    gamma = 2 # Assume ratio between two compensator zeros.  Between 1-3 said to be a heuristic range
    KP_ANG_gen, KI_ANG_gen, KD_ANG_gen = return_PID_coeffs(G_plant, gamma, s0, show=True)
    #print("Obtained from PLANT dynamics: KP_ANG: {}, KI_ANG: {}, KD_ANG: {}".format(KP_ANG_gen,KI_ANG_gen,KD_ANG_gen))
    newCtrl.useRK6 = USE_RK6

    if overrideI:
        KI_VEL = 0
        KI_ANG = 0
        KI_ANG_gen = 0

    if useVelocityPID:
        newCtrl.kP_VEL = KP_VEL
        newCtrl.kI_VEL = KI_VEL
        newCtrl.kD_VEL = KD_VEL

    if not getPID_from_plant:
        #print("Override: using ANG PID coeffs KP_ANG: {}, KI_ANG: {}, KD_ANG: {}".format(KP_ANG_gen,KI_ANG_gen,KD_ANG_gen))
        newCtrl.kP_ANG = KP_ANG
        newCtrl.kI_ANG = KI_ANG
        newCtrl.kD_ANG = KD_ANG

    else:
        #print("Using PID coefficients from plant dynamics.")

        if usePositionPID:
            figPath = 'dat/zephy_testlaunch/pdf/turb{}_Tabs{}_VEL_PID_KP{}_KI{}_KD{}_desiredVel{}_iniVel{}_ANG_PID_KP{}_KI{}_KD{}_constInput{}_desiredPos{}_servoSteps{}.pdf'.format(TURBULENCE,'YES' if USE_TABS else 'NO',KP_VEL,KI_VEL,KD_VEL,DESIRED_ROT_VEL,INI_ROT_VEL,KP_ANG_gen,KI_ANG_gen,KD_ANG_gen,CONST_FIXED,DESIRED_ROT_ANG,SERVO_STEP_COUNT)
            CSVSAVEPATH = 'dat/zephy_testlaunch/csv/run_turb{}_Tabs{}_VEL_PID_KP{}_KI{}_KD{}_desiredVel{}_iniVel{}_ANG_PID_KP{}_KI{}_KD{}_constInput{}_desiredPos{}_servoSteps{}.csv'.format(TURBULENCE,'YES' if USE_TABS else 'NO',KP_VEL,KI_VEL,KD_VEL,DESIRED_ROT_VEL,INI_ROT_VEL,KP_ANG_gen,KI_ANG_gen,KD_ANG_gen,CONST_FIXED,DESIRED_ROT_ANG,SERVO_STEP_COUNT)

            newCtrl.kP_ANG = KP_ANG_gen
            newCtrl.kI_ANG = KI_ANG_gen
            newCtrl.kD_ANG = KD_ANG_gen
        else:
            print("Controller set up but not engaged.")



    newCtrl.servoStepCount = float(SERVO_STEP_COUNT)
    newCtrl.velMinThresh = VELMINTHRESH

    rocket = sim.getRocket()

    rocketCD = newCtrl.getRocketCD(rocket)
    refA = newCtrl.getRefAreaFromSimulation(sim)
    rho = newCtrl.getDensityAtAltitude(sim,5000)
    mass = newCtrl.getRocketMass(rocket)
    #print("Rocket CD is {}".format(rocketCD))
    #print("Rocket Reference area is {} m^2".format(refA))
    #print("air density at 5000m is {} kg/m^3".format(rho))
    #print("mass is {} kg".format(mass))

    #B = rocketCD*rho*refA/2/mass
    #print("B is {}".format(B))









    print('FINISHED INIT PHASE')
    #exit(0)

    airbrakesCtrl.EARLIEST_AIRBRAKES_PREP_TIME = 4.0
    airbrakesCtrl.START_AIRBRAKES_PREP_VEL = 400.0
    airbrakesCtrl.START_AIRBRAKES_PREPROC_TIME = 12.0
    #airbrakesCtrl.AIRBRAKES_TIME_DELAY = 1.0
    #airbrakesCtrl.roundToHowMuch = 100
    airbrakesCtrl.overriden_A0 = -1#.999
    airbrakesCtrl.overriden_desiredApog = 4600#.999
    airbrakesCtrl.AIRBRAKES_SIMULATION_T_APOG = 31
    #airbrakesCtrl.override_t_apog = 33

    airbrakesCtrl.fudge_factor = 1.35
    airbrakesCtrl.fudge_factor_2 = 0

    airbrakesCtrl.fudge_factor_conrad = 1
    airbrakesCtrl.adjusting = 0

    airbrakesCtrl.factorK = 0.5#0.5
    airbrakesCtrl.kp_factor = 1
    airbrakesCtrl.ki_factor = 1
    airbrakesCtrl.velContribFudge = 1.#1.04 

    """airbrakesCtrl.cFudge = 0.825 # for 6275
    airbrakesCtrl.cFudge = 0.72 # for 6225
    airbrakesCtrl.cFudge = 0.7725 # for 6250
    airbrakesCtrl.cFudge = 0.9 # for 6300"""
    airbrakesCtrl.cFudge = 1#getCfudge(6320-4634+airbrakesCtrl.overriden_desiredApog)
    airbrakesCtrl.AIRBRAKES_MEASUREMENT_FUDGE_FACTOR = 0.875#0.75
    airbrakesCtrl.rate = 1

    airbrakesCtrl.DEBUG_AIRBRAKES_ON = True
    #airbrakesCtrl.t_apog = 35#.999
    # 0.0985 for 95% airbrakes


    sim.simulate()
    logging.info("Simulation done, plotting.")


    #omegaZ = np.array(newCtrl.pastOmegaZ)
    omegaZ = np.array(airbrakesCtrl.pastOmegaZ)
    #thetaZ = np.array(newCtrl.pastThetaZ)
    thetaZ = np.array(airbrakesCtrl.pastThetaZ)



    #Qlog = np.array(newCtrl.Qlog)
    #Qlog = np.array(airbrakesCtrl.Qlog)
    #CldArefDLog = np.array(newCtrl.CldArefDLog)
    #CldArefDLog = np.array(airbrakesCtrl.CldArefDLog)
    #CldLog = np.array(newCtrl.CldLog)
    #CldLog = np.array(airbrakesCtrl.CldLog)




    realThetaZ = []
    for i in range(len(omegaZ)):
        #realThetaZ.append(np.sum(omegaZ[:i])*newCtrl.latestTimeStep)
        realThetaZ.append(np.sum(omegaZ[:i])*airbrakesCtrl.latestTimeStep)

    realThetaZ = np.array(realThetaZ)
    #finCantLog = np.array(newCtrl.finCantLog)
    #finTabAngleLog = np.array(newCtrl.finTabAngleLog)
    #smoothTabAngleHist = np.array(newCtrl.desiredFinTabAngleLog)

    #finHistory = finCantLog if not USE_TABS else finTabAngleLog

    data = orh.get_timeseries(sim, [dT.TYPE_ALTITUDE,dT.TYPE_VELOCITY_Z,dT.TYPE_TIME,dT.TYPE_VELOCITY_TOTAL,dT.TYPE_ACCELERATION_Z])

    t = np.array(data[dT.TYPE_TIME].tolist())
    alt = np.array(data[dT.TYPE_ALTITUDE].tolist())
    vel = np.array(data[dT.TYPE_VELOCITY_Z].tolist())
    #velMagnitude = np.array(newCtrl.rktVelMagLog)
    velMagnitude = np.array(data[dT.TYPE_VELOCITY_TOTAL].tolist())
    accelZ = np.array(data[dT.TYPE_ACCELERATION_Z].tolist())
    airbrakesLog = np.array(airbrakesCtrl.airbrakesLog)
    timeLog = np.array(airbrakesCtrl.timeLog)
    #velMagnitude = np.array(data[dT.TYPE_VELOCITY_TOTAL].tolist())

    # patch
    finHistory = vel

    if not MASTER_NOISE_OVERRIDE:
        #alt = np.array(newCtrl.rktAltLog)
        alt = np.array(airbrakesCtrl.rktAltLog)



    logger = logging.getLogger()
    logger.setLevel(level=logging.ERROR)




    # SAVE DATA TO CSV


    import matplotlib.pyplot as plt

    apogeeInd = alt.argmax()
    np.set_printoptions(legacy='1.13')
    velMagProcess = velMagnitude[int(len(t)/4):apogeeInd]
    #controlCutoffIndex = np.argmin(np.abs(velMagProcess - VELMINTHRESH)) + int(len(t)/4)
    #controlCutoffTime = t[controlCutoffIndex]


    #plt.subplots()
    #motorendedIndex = np.argmax(velMagnitude)
    #plt.plot(Qlog[:motorendedIndex],CldArefDLog[:motorendedIndex]/omegaZ[:motorendedIndex])#/vel[:motorendedIndex]/vel[:motorendedIndex])
    #averaging = 2.2e-5
    #plt.hlines(averaging,Qlog[0],Qlog[motorendedIndex],color='red',linestyle='dashed',label='Avg. CldArefD until motor end')
    #plt.plot(Qlog[motorendedIndex:apogeeInd],CldArefDLog[motorendedIndex:apogeeInd]/omegaZ[motorendedIndex:apogeeInd])#/vel[motorendedIndex:apogeeInd]/vel[motorendedIndex:apogeeInd])
    #plt.show()



#print("Got cutoff time: {} s at index {}, velMag: {}".format(controlCutoffTime,controlCutoffIndex,velMagnitude[controlCutoffIndex]))
    #print(finCantLog[:apogeeInd])

    #dataArr = np.array([t[:apogeeInd],alt[:apogeeInd],vel[:apogeeInd],velMagnitude[:apogeeInd],accelZ[:apogeeInd]])
    #np.savetxt(CSVSAVEPATH, dataArr.T, delimiter=',', header='Time (s),Altitude (m),Velocity (m/s),Velocity Magnitude (m/s),Accel <Z> (rad/s)', comments='')
    #print("Saved data to {}".format(CSVSAVEPATH))
    apogeeHappened = np.array(list(np.zeros(apogeeInd)) + list(np.ones(len(t)-apogeeInd)))


    dataArr = np.array([t[:apogeeInd+5],alt[:apogeeInd+5],vel[:apogeeInd+5],accelZ[:apogeeInd+5],apogeeHappened[:apogeeInd+5],airbrakesLog[:apogeeInd+5]])
    CSVSAVEPATH = "dat/zephy_testlaunch/csv/{}/airbrakes_input.csv".format(int(airbrakesCtrl.overriden_desiredApog))
    np.savetxt(CSVSAVEPATH, dataArr.T, delimiter=',', header='Time,Altitude,Velocity,Acceleration,Apogee,Predicted DP', comments='')
    print("Saved data to {}".format(CSVSAVEPATH))







    #fig, axs = plt.subplots(nrows=3,sharex='col',figsize=(8.5,11/2))
    """fig, axs = plt.subplot_mosaic(
        [['A'],['A'],
         ['B'],['B'],['C'],['C'],['C']],figsize=(7,11/2)
    )
    plt.subplots_adjust(right=0.91,left=0.1,bottom=0.08,hspace=0.3)
    ax = axs['A']
    ax2 = axs['B']
    ax9 = axs['C']
    ax.set_xticks([])
    ax2.set_xticks([])
    plt.close()"""
    fig,axs = plt.subplots(nrows=2,sharex='col',figsize=(8.5,11))
    ax = axs[0]#plt.gca()


    import scipy.optimize as spopt
    t = np.array(t)
    vel = np.array(vel)

    g = 9.81

    def vel_from_accel(t,alpha,t_apog):
        return -g*(t-t_apog) + alpha*((t -t_apog)**2)/2

    def vel_fit(t, a, b, t_apog):
        return a*(t-t_apog)**3 + b*(t-t_apog)**2  - g*(t - t_apog)

    def vel_fit_hardcoded_t_apog(t, a, b):
        global shared_t_apog
        return a*(t-shared_t_apog)**3 + b*(t-shared_t_apog)**2  - g*(t - shared_t_apog)


    def altitude_model(t, a, b, t_apog, x0):
        return a*(t-t_apog)**4/4 + b*(t-t_apog)**3/3  - g*(t - t_apog)**2/2+x0

    def altitude_model_short_term(t, a, b, x0):
        global shared_t_apog
        return a*(t-shared_t_apog)**3 + b*(t-shared_t_apog)**2  - g*(t - shared_t_apog)+x0

    def altitude_model_short_term2(t, a, b, x0):
        global shared_t_apog2
        # e^{-\left(x-t_{apog}\right)^{2.16}}
        return a*(t-shared_t_apog2)**2 + x0*np.exp(b*(shared_t_apog2-t)**1.75)

    def accel_model(t, a, b, t_apog):
        return 3*a*(t-t_apog)**2 + 2*b*(t-t_apog) - g


    def accel_quintic(t,a):
        global shared_t_apog
        return a*(t-shared_t_apog)**5 - g


    def R2(func,y_coords,x_coords,*opt0):
        residuals = y_coords - np.round(func(x_coords, *opt0),3)
        ss_res = np.sum(residuals**2)
        ss_tot = np.sum((y_coords-np.mean(y_coords))**2)
        correlation = 1 - (ss_res / ss_tot)
        return correlation


    def actualPositionModel(t,b,c1,c2):
        a = g
        # b = hardcoded C_D*A*rho/2
        return np.log(np.cos(np.sqrt(a*b)*(c1+t)))/b + c2




    index0 = np.argmin((vel-400)**2)
    index1 = np.argmin((t-12)**2)
    fit_T = t[index0:index1][::200][:20]
    fit_V = vel[index0:index1][::200][:20]
    fit_A = accelZ[index0:index1][::200][:20]
    fit_alt = alt[index0:index1][::200][:20]
    #print("Fitting a timeseries of {} points.".format(len(fit_V)))

    global shared_t_apog
    
    shared_t_apogs = [29,30,31]
    R2s = []
    optsA = []
    #print("quintic fit for accel")
    for shared_t_apog in shared_t_apogs:
        opt_Q_A,pcov = spopt.curve_fit(accel_quintic,fit_T,fit_A,maxfev=10000000)
        R2_Q_A = R2(accel_quintic,fit_A,fit_T,*opt_Q_A)
        #print("Got fit with R2 {} for t_apog ".format(np.round(R2_Q_A,5), shared_t_apog))
        R2s.append(R2_Q_A)
        optsA.append(opt_Q_A)

    best_t_apog = shared_t_apogs[np.argmax(R2s)]
    opt_A_best = optsA[np.argmax(R2s)]

    shared_t_apog = best_t_apog# - 0.5
    print("Using t_apog {}".format(shared_t_apog))

    optA,pcovA = spopt.curve_fit(vel_fit_hardcoded_t_apog, fit_T, fit_V, maxfev=1000000, 
        p0=(5, 5),bounds=([-np.inf,-np.inf],[np.inf,np.inf]))
    a,b = optA
    R2_A = R2(vel_fit_hardcoded_t_apog,fit_V,fit_T,*optA)
    #print("normal fit for velocity for t_apog {}".format(shared_t_apog))
    #print("Got fit with R2 {}".format(np.round(R2_A,5)))
    opt0 = a,b,shared_t_apog
    #print("parameters are: a {} b {}".format(a,b))
    t_apog = shared_t_apog


    integratedVel = vel_fit(t, *opt0)
    x0 = (alt[index1] - altitude_model(t[index1], a, b, t_apog, 0))#*1.04
    #print("Got x0 {}".format(x0))
    maxAlt = altitude_model(t_apog, a, b, t_apog, x0)
    maxAltReal = np.max(alt)
    print("Got apogee {}".format(maxAltReal))
    print("Airbrakes Deployed at {}".format(airbrakesCtrl.START_AIRBRAKES_PREPROC_TIME + 1))
    print("Velocity at deployment: {}".format(vel[np.argmin((t-airbrakesCtrl.START_AIRBRAKES_PREPROC_TIME + 1)**2)]))
    #print("Predicted apogee without airbrakes {}".format(maxAlt))
    #print("Diff from airbrakeless prediction: {}".format(np.abs(maxAltReal-maxAlt)))
    noAirbrakesAlt = 4634
    diff = np.abs(maxAltReal-airbrakesCtrl.overriden_desiredApog)
    print("Airbrakes Goal Diff: {}, {}%".format(np.round(diff,4), int(diff/(noAirbrakesAlt-airbrakesCtrl.overriden_desiredApog)*100)))
    #exit(0)


    index2 = np.argmin((t-16)**2)
    index3 = np.argmin((t-18.8)**2)
    fit_T2 = t[index2:index3][::100][:13]
    fit_V2 = vel[index2:index3][::100][:13]
    fit_alt2 = alt[index2:index3][::100][:13]
    fit_A2 = accelZ[index2:index3][::100][:13]
    optA2,pcovA2 = spopt.curve_fit(altitude_model_short_term, fit_T2, fit_alt2, maxfev=1000000)
    optA3,pcovA3 = spopt.curve_fit(actualPositionModel, t[np.argmin((t-14)**2):np.argmin((t-21)**2)], alt[np.argmin((t-14)**2):np.argmin((t-21)**2)], maxfev=1000000,p0=(0.00012,-32.72,6275))
    a2,b2,x02 = optA2
    x022 = (alt[index3] - altitude_model_short_term(t[index3], a2,b2, 0))#*1.04
    print(x022-x02)
    print("Estimate from 16 to 19 seconds: predicted modified apogee {}".format(x022))
    print("Estimate from closed form solution: {}".format(list(optA3)[-1]))
    #print("b: {}".format(list(optA3)[0]))
    #print("c1: {}".format(list(optA3)[1]))



    # Conrad's Formula

    fudge_factor_conrad = 3.2    
    fudged_alt_diff = 13


    #print("mass is {}".format(mass))
    h0 = alt[np.argmin((t-12)**2)] # alt at 12 seconds
    v0 = vel[np.argmin((t-12)**2)] # vel at 12 seconds
    c = rho*rocketCD*refA/2 
    blind_estimate = h0 + mass/(2*c)*np.log(v0**2*(c)/g/mass+1) - fudged_alt_diff
    alpha = rho*1.28*airbrakesCtrl.A0_req*airbrakesCtrl.a_max/2
    c += alpha/fudge_factor_conrad
    blind_estimate_for_airbrakes = h0 + mass/(2*c)*np.log(v0**2*(c)/g/mass+1) - fudged_alt_diff
    print("Estimate of no-airbrakes alt from Conrad: {}".format(blind_estimate))
    print("Estimate of chosen-airbrakes-deploy alt from Conrad: {}".format(blind_estimate_for_airbrakes))


    deltaA = np.array(airbrakesCtrl.deltaA)
    times = np.array(airbrakesCtrl.timeStampDeltas)
    deltaH = np.array(airbrakesCtrl.deltaH)
    Hf = np.array(airbrakesCtrl.Hf)

    ax.vlines(t[index0],0,10000,linestyle='dotted',color='k')
    ax.vlines(t[index1],0,10000,linestyle='dotted',color='k')
    print("First Measurement time is between {} and {}".format(t[index0],t[index1]))
    ax.vlines(t[index2],0,10000,linestyle='dotted',color='purple')
    ax.vlines(t[index3],0,10000,linestyle='dotted',color='purple')
    print("Second Measurement time is between {} and {}".format(t[index2],t[index3]))

    ax.scatter(fit_T2, fit_alt2,marker='^',color='purple')
    ax.scatter(fit_T, fit_V,marker='^',color='blue')


    ax.plot(t[:apogeeInd],vel[:apogeeInd],label="Velocity",color='blue')
    ax.plot(t[:apogeeInd],velMagnitude[:apogeeInd],label="Velocity Mag.",color='steelblue',linewidth=3,alpha=0.4,zorder=-1)
    ax.set_xlim(t[0],t[apogeeInd-1])
    ax.plot(t[:apogeeInd],alt[:apogeeInd],label="Altitude",color='purple')
    ax.plot(times,Hf,label="Mid-Flt Predict Alt",color='magenta')



    ax.plot(t,integratedVel,label="Velocity Fit",color='blue',linewidth=5,alpha=0.2,zorder=-1)
    ax.plot(t, altitude_model(t, a, b, t_apog, x0), label="Altitude Fit", color='purple', linewidth=5, alpha=0.2)
    ax.plot(t, altitude_model_short_term(t, a2, b2, x02), label="Altitude Fit 2", color='steelblue', linewidth=5, alpha=0.2)
    #ax.plot(t, actualPositionModel(t, *optA3), label="Altitude Fit global", color='red', linewidth=5, alpha=0.2)


    ax0 = ax.twinx()
    #ax0.scatter(fit_T, fit_A,marker='^',color='r')
    ax.set_ylim(0,7000)
    ax0.plot(t[np.argmin(accelZ):],accelZ[np.argmin(accelZ):],color='red',label='Z Acceleration ($\\rm m/s^2$)')
    ax0.plot(t[np.argmin(accelZ):], accel_model(t[np.argmin(accelZ):], *opt0), label="Fitted Z Acceleration", color='red', linewidth=5, alpha=0.2)
    #ax0.plot(t[np.argmin(accelZ):], accel_quintic(t[np.argmin(accelZ):], *opt_A_best), label="Fitted Z Acceleration", color='orange', linewidth=5, alpha=0.5)

    ax.plot([-1],[-1],color='red',label='Z Acceleration ($\\rm m/s^2$)')
    ax.plot([-1],[-1],color='red',label='Fitted Z Acceleration',linewidth=5, alpha=0.2)

    ax.legend(loc='center right',bbox_to_anchor=(1, 0.61))

    ax0.spines['right'].set_color('red')
    ax0.spines['left'].set_color('blue')
    ax0.yaxis.label.set_color('red')
    ax.yaxis.label.set_color('blue')
    ax0.tick_params(axis='y', colors='red')
    ax.tick_params(axis='y', colors='blue')
    ax.set_ylabel("Velocity (m/s) and Altitude (m)")
    ax0.set_ylabel("Z Accel ($\\rm m/s^2$)")
    ax.hlines(VELMINTHRESH,*ax.get_xlim(),color='k',linestyle='dotted')
    ax0.set_ylim(np.min(accelZ[:apogeeInd]),np.max(accelZ[:apogeeInd]))
    ax.set_xlabel("Time (s)")

    #plt.show()


    axs[1].plot(times[1:],airbrakesCtrl.Astar/0.0066 + deltaA[2:]/0.0066,color='b',label="$A(t)$") 
    ax22 = axs[1].twinx()
    ax22.plot(times[1:],deltaH[2:],color='r',label="$\\Delta h$")

    axs[1].legend(loc='upper left')
    ax22.legend(loc='upper right')



    plt.savefig(figPath)
    plt.show()

    dataArr = np.array([t,alt,vel,accelZ,apogeeHappened,airbrakesLog])
    CSVSAVEPATH = "dat/zephy_testlaunch/csv/{}/airbrakes_input.csv".format(int(airbrakesCtrl.overriden_desiredApog))
    np.savetxt(CSVSAVEPATH, dataArr.T, delimiter=',', header='Time,Altitude,Velocity,Acceleration,Apogee,Predicted DP', comments='')
    print("Saved data to {}".format(CSVSAVEPATH))


    exit(0)












































    ax2.set_xlim(t[0],t[apogeeInd-1])
    ax9.set_xlim(t[0],t[apogeeInd-1])

    if showControlCutoffLine:
        ax.vlines(controlCutoffTime,0,ax.get_ylim()[1],color='black',linestyle='dashed',linewidth=0.75,alpha=0.5)
        ax9.vlines(controlCutoffTime,-1e10,1e10,color='black',linestyle='dashed',linewidth=0.75,alpha=0.5)


    ax3 = ax9.twinx()#9#.twinx()


    ax4 = ax9#.twinx()

    """ax4.plot(t[:apogeeInd],omegaZ[:apogeeInd],label="Ang Velocity (rad/s)",color='blue',alpha=0.5,linewidth=2)
    ax4.set_ylabel("Ang. Velocity (rad/s)")
    ax4.yaxis.label.set_color('blue')
    ax4.tick_params(axis='y', colors='blue')
    ax3.tick_params(axis='y', colors='purple')
    ax4.set_ylim(-60,60)
    ax3.plot([-1],[-1],label="Ang Velocity (rad/s)",color='blue',alpha=0.5,linewidth=2)"""

    FLAG = True
    ax3.plot([-1],[-1],label="Ang. Pos. (deg)",color='blue',alpha=0.5,linewidth=2)
    ax3.plot(t[:apogeeInd],omegaZ[:apogeeInd],label="Ang. Vel. (rad/s)",color='red',alpha=0.5,linewidth=2)
    ax4.plot(t[:apogeeInd],thetaZ[:apogeeInd],label="Ang. Pos. (deg)",color='blue',alpha=0.5,linewidth=2)
    #ax4.plot(t[:apogeeInd],realThetaZ[:apogeeInd],label="Ang. Pos. (deg)",color='blue',alpha=0.5,linewidth=2)
    #ax4.plot(t[:apogeeInd],sgf(omegaZ[:apogeeInd],10,2)*10,label="Ang. Accel. (rad/s$^2$)",color='red',alpha=1,linewidth=2)
    ax4.set_ylabel("Ang. Pos. (deg)")
    ax4.yaxis.label.set_color('blue')
    ax2.yaxis.label.set_color('blue')
    ax4.tick_params(axis='y', colors='blue')
    ax2.tick_params(axis='y', colors='blue')
    ax4.set_ylim(-180,180)
    ax4.set_yticks(np.arange(-180,181,45))
    #ax4.set_ylim(-1,1)
    ax4.hlines(DESIRED_ROT_ANG,*ax3.get_xlim(),color='blue',linestyle='dotted',label='Desired Angle')
    ax3.set_ylabel("Ang. Vel. (rad/s)")
    ax3.yaxis.label.set_color('red')
    ax3.tick_params(axis='y', colors='red')
    if WIND_EVENT_1_TIMESTAMP > 0:
        ax3.hlines(WIND_EVENT_1_GUST,ax3.get_xlim()[1]*0.9,ax3.get_xlim()[1],color='red',linestyle='dotted')
        ax3.hlines(WIND_EVENT_1_GUST,WIND_EVENT_1_TIMESTAMP,ax3.get_xlim()[1],color='red',linestyle='dotted',alpha=0.2)
    if WIND_EVENT_2_TIMESTAMP > 0:
        ax3.hlines(WIND_EVENT_2_GUST,ax3.get_xlim()[1]*0.9,ax3.get_xlim()[1],color='red',linestyle='dotted')
        ax3.hlines(WIND_EVENT_2_GUST,WIND_EVENT_1_TIMESTAMP,ax3.get_xlim()[1],color='red',linestyle='dotted',alpha=0.2)
    if WIND_EVENT_3_TIMESTAMP > 0:
        ax3.hlines(WIND_EVENT_3_GUST,ax3.get_xlim()[1]*0.9,ax3.get_xlim()[1],color='red',linestyle='dotted')
        ax3.hlines(WIND_EVENT_3_GUST,WIND_EVENT_1_TIMESTAMP,ax3.get_xlim()[1],color='red',linestyle='dotted',alpha=0.2)

    #ax3.plot([-1],[-1],label="Ang. Vel. (rad/s)",color='red',alpha=0.5,linewidth=2)

    ax2.plot(t[:apogeeInd],smoothTabAngleHist[:apogeeInd],label="Controller Output",color='blue',linewidth=0.5,alpha=1)
    ax2.plot(t[:apogeeInd],finHistory[:apogeeInd],label="Fin Cant (deg)" if not USE_TABS else 'Fin Tab Angle (deg)',color='purple',alpha=1,linewidth=1)
    ax3.plot([-1],[-1],color='blue',linestyle='dotted',label='Desired Angle')

    ax2.legend()
    if DESIRED_ROT_VEL == 0:
        if not FLAG:
            ax3.plot(t[:apogeeInd],thetaZ[:apogeeInd],label="Angular Position",color='purple',linestyle='dotted',alpha=0.7)
    #ax2.set_ylim(-1e2,1e2)
    if showControlCutoffLine:
        ax2.vlines(controlCutoffTime,-1e4,1e4,color='black',linestyle='dashed',linewidth=0.75,alpha=0.5)
    #ax3.set_yscale('symlog',linthresh=1e-1)
    #ax2.set_ylim(-3e1,3e1)
    if DESIRED_ROT_VEL == 0:
        ax2.set_ylabel("Fin Cant, Angular Position" if not USE_TABS else 'Fin Tab Angle (deg)')
    else:
        ax2.set_ylabel("Fin Cant (deg)" if not USE_TABS else 'Fin Tab Angle (deg)')
    #ax2.spines['left'].set_color('purple')
    #ax2.spines['right'].set_color('blue')
    #ax2.yaxis.label.set_color('purple')



    maxLim2 = max(*np.abs(ax2.get_ylim()))
    maxLim3 = max(*np.abs(ax3.get_ylim()))
    #ax2.set_ylim(-maxLim2,maxLim2)
    ax2.set_ylim(-12,12)
    ax3.set_ylim(-70,70)
    ax3.set_yticks(np.arange(-60,61,30))

    #ax2.set_xlim(*ax2.get_xlim())
    #ax2.set_ylim(-50,50)
    #ax2.hlines(0,*ax2.get_xlim(),color='k',linestyle='dotted')



    ax2.vlines(WIND_EVENT_1_TIMESTAMP, -1e4,1e4,color='green',linewidth=1,label='Wind Gust Events')
    ax2.vlines(WIND_EVENT_2_TIMESTAMP, -1e4,1e4,color='green',linewidth=1)
    ax2.vlines(WIND_EVENT_3_TIMESTAMP, -1e4,1e4,color='green',linewidth=1)
    ax.vlines(WIND_EVENT_1_TIMESTAMP, -1e4,1e4,color='green',linewidth=1,label='Wind Gust Events')
    ax.vlines(WIND_EVENT_2_TIMESTAMP, -1e4,1e4,color='green',linewidth=1)
    ax.vlines(WIND_EVENT_3_TIMESTAMP, -1e4,1e4,color='green',linewidth=1)
    ax9.vlines(WIND_EVENT_1_TIMESTAMP, -1e4,1e4,color='green',linewidth=1,label='Wind Gust Events')
    ax9.vlines(WIND_EVENT_2_TIMESTAMP, -1e4,1e4,color='green',linewidth=1)
    ax9.vlines(WIND_EVENT_3_TIMESTAMP, -1e4,1e4,color='green',linewidth=1)



    ax9.set_xlabel("Time (s)")
    ax3.legend(loc='lower right',ncol=3)
   # ax9.legend(loc='lower right',fontsize=8,ncol=3)
    titleFirstLine = "\\begin{center}\\noindent \\sc Tab-Ctrlled Flight" if USE_TABS else "Cant-Ctrlled Flight"
    titleTurbLine = "; Turb. ${}\\%$".format(TURBULENCE)
    title_vPID_Line = "\\\\[0.25em] $\\omega$PID: $\\tt [{},{},{}]$ ".format(*np.round([KP_VEL,KI_VEL,KD_VEL],2)) if useVelocityPID else ""
    title_aPID_Line = "; $\\varphi$PID: $\\tt [{},{},{}]$ ".format(*np.round([KP_ANG,KI_ANG,KD_ANG],2)) if usePositionPID else ""
    title_fixed_Line = "; $\\alpha_0={}$".format(np.round(CONST_FIXED,2)) if CONST_FIXED != 0 else ""
    title_suppLine = ("; $\\omega_0={}$".format(np.round(DESIRED_ROT_VEL,2)) if useVelocityPID else "") + ("; $\\varphi_0={}$ ".format(np.round(DESIRED_ROT_ANG,2)) if usePositionPID else "")
    title_windLine = ("\\\\[0.25em] Wind Gusts: {}rad/s at {}s, {}rad/s at {}s, {}rad/s at {}s".format(WIND_EVENT_1_GUST,WIND_EVENT_1_TIMESTAMP,WIND_EVENT_2_GUST,WIND_EVENT_2_TIMESTAMP,WIND_EVENT_3_GUST,WIND_EVENT_3_TIMESTAMP) if (WIND_EVENT_1_TIMESTAMP > 0 or WIND_EVENT_2_TIMESTAMP > 0 or WIND_EVENT_3_TIMESTAMP > 0) else "")
    plt.gcf().suptitle(titleFirstLine +titleTurbLine + title_vPID_Line + title_aPID_Line + title_fixed_Line + title_suppLine + title_windLine + "\\end{center}", fontsize=12)

    plt.savefig(figPath)
    #plt.show()

    logger.setLevel(level=logging.INFO)


    exit(0)







