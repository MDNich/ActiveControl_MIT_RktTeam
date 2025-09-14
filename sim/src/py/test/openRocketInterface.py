#!/usr/local/python3

# MUST BE RUN FROM TOP LEVEL WORKING DIRECTORY: 'sim'.
import os
from time import sleep
from Simplified_plant_dynamics import *
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



import threading

VELMINTHRESH = 15
TURBULENCE = 0
USE_TABS = True
CONST_FIXED = 0
# VEL PID
KP_VEL = 3
KI_VEL = 0#.1#0.75
KD_VEL = .1
# ANG PID
KP_ANG = 5
KI_ANG = 0#.1#0.75
KD_ANG = 0.1
# OTHER PARAMS
INI_ROT_VEL = 0
DESIRED_ROT_VEL = 0
DESIRED_ROT_ANG = 0
overrideI = True
getPID_from_plant = False
useVelocityPID = True
usePositionPID = True
showControlCutoffLine = useVelocityPID or usePositionPID#True
USE_RK6 = True
ROUND_5 = False
SERVO_STEP_COUNT = int(1024) # Large number means very small steps, effectively continuous.
DEBUG_JAVA_MSG = False



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

figPath = 'dat/demonstrator_4/pdf/turb{}_Tabs{}_VEL_PID_KP{}_KI{}_KD{}_desiredVel{}_iniVel{}_ANG_PID_KP{}_KI{}_KD{}_constInput{}_desiredPos{}_servoSteps{}.pdf'.format(TURBULENCE,'YES' if USE_TABS else 'NO',KP_VEL,KI_VEL,KD_VEL,DESIRED_ROT_VEL,INI_ROT_VEL,KP_ANG,KI_ANG,KD_ANG,CONST_FIXED,DESIRED_ROT_ANG,SERVO_STEP_COUNT)
CSVSAVEPATH = 'dat/demonstrator_4/csv/run_turb{}_Tabs{}_VEL_PID_KP{}_KI{}_KD{}_desiredVel{}_iniVel{}_ANG_PID_KP{}_KI{}_KD{}_constInput{}_desiredPos{}_servoSteps{}.csv'.format(TURBULENCE,'YES' if USE_TABS else 'NO',KP_VEL,KI_VEL,KD_VEL,DESIRED_ROT_VEL,INI_ROT_VEL,KP_ANG,KI_ANG,KD_ANG,CONST_FIXED,DESIRED_ROT_ANG,SERVO_STEP_COUNT)


# Start
instance, orh, or_obj = startOR()

# Import classes
simStatClass = or_obj.simulation.SimulationStatus
coordClass = or_obj.util.Coordinate
worldCoordClass = or_obj.util.WorldCoordinate
quatClass = or_obj.util.Quaternion

# Load rocket
doc, rktObj = loadRocket(orh, 'demonstrator_4.ork')

newCtrl = or_obj.simulation.listeners.NewControlStepListener

newCtrl.theRocket = rktObj


# load flight conf
flightConfig = doc.getSelectedConfiguration()
logging.info("Motor identifier: ")
logging.info(flightConfig)

# load sim
sim = doc.getSimulation(3)
logging.warning("loaded document + simulation")

datPath = 'dat/simResults/demonstrator_4_out_long.csv'
#figPath = 'dat/simResults/demonstrator_4_out_long.pdf'


# Get all components, filter for fins.
def get_fins(rocket):
    fins = []
    for component in rocket.iterator(True):  # True = depth-first search
        if isinstance(component, jpype.JClass("info.openrocket.core.rocketcomponent.FinSet")):
            fins.append(component)
    return fins

finList = get_fins(rktObj)
print("Got Fins from Rocket:")
for fin in finList:
    print(f"""
        Fin Name: {fin.getName()}
        Number: {fin.getFinCount()}
        Span: {fin.getSpan()} m
        Thickness: {fin.getThickness()} m
        Cant Angle: {fin.getCantAngle()} rad
        """)


finNames = [fin.getName() for fin in finList]
index = 0
theFinIndexToModify = -1
for fni in finNames:
    if not USE_TABS:
        if fni == "FINS_TWIST":
            theFinIndexToModify = index
            break
    else:
        if fni == "FINS_TABBED":
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
    v0z = 0 # m/s

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
    listener_array = [newCtrl()]


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

    newCtrl.initialStat = theNextSimulationStatus
    newCtrl.iniVel = v0z



    # Need to do this otherwise exact same numbers will be generated for each identical run
    sim.getOptions().randomizeSeed()

    newCtrl.theFinsToModify = finToPlayWith
    newCtrl.simulateUsingTabs = USE_TABS

    newCtrl.desiredRotVel = DESIRED_ROT_VEL
    newCtrl.desiredRotAng = DESIRED_ROT_ANG
    newCtrl.IdecayFactor = 1

    newCtrl.velocityPIDon = useVelocityPID
    newCtrl.positionPIDon = usePositionPID

    newCtrl.roundToNearest5 = ROUND_5

    newCtrl.flagPrintDebugMsg = DEBUG_JAVA_MSG


    newCtrl.constFixed = CONST_FIXED

    # get PID coefficients from algorithm generating.

    ts_des = 0.8 # Desired settling time of 0.5 seconds
    Mp_des = 0.2 # Desired max peak of less than 20%
    s0 = set_dominant_poles(ts_des, Mp_des)
    print(f'Desired Poles: {s0} and its conjugate')

    gamma = 2 # Assume ratio between two compensator zeros.  Between 1-3 said to be a heuristic range
    KP_ANG_gen, KI_ANG_gen, KD_ANG_gen = return_PID_coeffs(G_plant, gamma, s0, show=True)
    print("Obtained from PLANT dynamics: KP_ANG: {}, KI_ANG: {}, KD_ANG: {}".format(KP_ANG_gen,KI_ANG_gen,KD_ANG_gen))
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
        print("Override: using ANG PID coeffs KP_ANG: {}, KI_ANG: {}, KD_ANG: {}".format(KP_ANG_gen,KI_ANG_gen,KD_ANG_gen))
        newCtrl.kP_ANG = KP_ANG
        newCtrl.kI_ANG = KI_ANG
        newCtrl.kD_ANG = KD_ANG

    else:
        print("Using PID coefficients from plant dynamics.")

        if usePositionPID:
            figPath = 'dat/demonstrator_4/pdf/turb{}_Tabs{}_VEL_PID_KP{}_KI{}_KD{}_desiredVel{}_iniVel{}_ANG_PID_KP{}_KI{}_KD{}_constInput{}_desiredPos{}_servoSteps{}.pdf'.format(TURBULENCE,'YES' if USE_TABS else 'NO',KP_VEL,KI_VEL,KD_VEL,DESIRED_ROT_VEL,INI_ROT_VEL,KP_ANG_gen,KI_ANG_gen,KD_ANG_gen,CONST_FIXED,DESIRED_ROT_ANG,SERVO_STEP_COUNT)
            CSVSAVEPATH = 'dat/demonstrator_4/csv/run_turb{}_Tabs{}_VEL_PID_KP{}_KI{}_KD{}_desiredVel{}_iniVel{}_ANG_PID_KP{}_KI{}_KD{}_constInput{}_desiredPos{}_servoSteps{}.csv'.format(TURBULENCE,'YES' if USE_TABS else 'NO',KP_VEL,KI_VEL,KD_VEL,DESIRED_ROT_VEL,INI_ROT_VEL,KP_ANG_gen,KI_ANG_gen,KD_ANG_gen,CONST_FIXED,DESIRED_ROT_ANG,SERVO_STEP_COUNT)

            newCtrl.kP_ANG = KP_ANG_gen
            newCtrl.kI_ANG = KI_ANG_gen
            newCtrl.kD_ANG = KD_ANG_gen
        else:
            print("Controller set up but not engaged.")



    newCtrl.servoStepCount = float(SERVO_STEP_COUNT)
    newCtrl.velMinThresh = VELMINTHRESH



    simThread = threading.Thread(target=lambda: sim.simulate(listener_array))
    simThread.start()

    simThread.join()
    logging.info("Simulation done, plotting.")


    omegaZ = np.array(newCtrl.pastOmegaZ)
    thetaZ = np.array(newCtrl.pastThetaZ)
    finCantLog = np.array(newCtrl.finCantLog)
    finTabAngleLog = np.array(newCtrl.finTabAngleLog)
    smoothTabAngleHist = finTabAngleLog#np.array(newCtrl.desiredFinTabAngleLog)

    finHistory = finCantLog if not USE_TABS else finTabAngleLog


    data = orh.get_timeseries(sim, [dT.TYPE_ALTITUDE,dT.TYPE_VELOCITY_Z,dT.TYPE_TIME])

    t = np.array(data[dT.TYPE_TIME].tolist())
    alt = np.array(data[dT.TYPE_ALTITUDE].tolist())
    vel = np.array(data[dT.TYPE_VELOCITY_Z].tolist())
    velMagnitude = np.array(newCtrl.rocketVelMagnitudeLog)



    logger = logging.getLogger()
    logger.setLevel(level=logging.ERROR)




    # SAVE DATA TO CSV


    import matplotlib.pyplot as plt

    apogeeInd = alt.argmax()
    np.set_printoptions(legacy='1.13')
    velMagProcess = velMagnitude[int(len(t)/4):apogeeInd]
    controlCutoffIndex = np.argmin(np.abs(velMagProcess - VELMINTHRESH)) + int(len(t)/4)
    controlCutoffTime = t[controlCutoffIndex]
    #print("Got cutoff time: {} s at index {}, velMag: {}".format(controlCutoffTime,controlCutoffIndex,velMagnitude[controlCutoffIndex]))
    #print(finCantLog[:apogeeInd])

    dataArr = np.array([t[:apogeeInd],alt[:apogeeInd],vel[:apogeeInd],omegaZ[:apogeeInd],finHistory[:apogeeInd], thetaZ[:apogeeInd]])
    label = 'Fin Cant Angle (deg)' if not USE_TABS else 'Fin Tab Angle (deg)'
    np.savetxt(CSVSAVEPATH, dataArr.T, delimiter=',', header='Time (s),Altitude (m),Velocity (m/s),Angular Velocity (rad/s),{},Angular Position (deg)'.format(label), comments='')
    print("Saved data to {}".format(CSVSAVEPATH))


    fig, axs = plt.subplots(nrows=2,sharex='col')
    plt.subplots_adjust(right=0.89)
    ax = axs[0]
    ax2 = axs[1]
    ax.plot(t[:apogeeInd],vel[:apogeeInd],label="Velocity",color='blue')
    ax.plot(t[:apogeeInd],velMagnitude[:apogeeInd],label="Velocity Mag.",color='steelblue',linewidth=3,alpha=0.4,zorder=-1)
    ax.set_xlim(t[0],t[apogeeInd-1])
    ax.set_ylim(0,ax.get_ylim()[1])
    ax.plot([-1],[-1],label="Altitude",color='red')
    ax.legend(loc='center right')
    ax0 = ax.twinx()
    ax0.plot(t[:apogeeInd],alt[:apogeeInd],label="Altitude",color='red')
    ax0.set_ylim(0,ax0.get_ylim()[1])
    if showControlCutoffLine:
        ax.vlines(controlCutoffTime,0,ax.get_ylim()[1],color='black',linestyle='dashed',linewidth=0.75,alpha=0.5)

    ax0.spines['right'].set_color('red')
    ax0.spines['left'].set_color('blue')
    ax0.yaxis.label.set_color('red')
    ax.yaxis.label.set_color('blue')
    ax0.tick_params(axis='y', colors='red')
    ax.tick_params(axis='y', colors='blue')
    ax.set_ylabel("Velocity (m/s)")
    ax0.set_ylabel("Altitude (m)")
    ax.hlines(VELMINTHRESH,*ax.get_xlim(),color='k',linestyle='dotted')



    ax2.plot(t[:apogeeInd],omegaZ[:apogeeInd],label="Ang Velocity (Rad/s)",color='red')
    ax2.set_ylabel("Ang Velocity")
    ax3 = ax2.twinx()
    ax3.plot(t[:apogeeInd],finHistory[:apogeeInd],label="Fin Cant (deg)" if not USE_TABS else 'Fin Tab Angle (deg)',color='purple',alpha=0.7)
    ax3.plot(t[:apogeeInd],smoothTabAngleHist[:apogeeInd],label="Controller Output",color='blue',linewidth=1)
    if DESIRED_ROT_VEL == 0:
        ax3.plot(t[:apogeeInd],thetaZ[:apogeeInd],label="Angular Position",color='purple',linestyle='dotted',alpha=0.7)
    ax3.set_ylim(-1e2,1e2)
    if showControlCutoffLine:
        ax2.vlines(controlCutoffTime,-1e4,1e4,color='black',linestyle='dashed',linewidth=0.75,alpha=0.5)
    ax3.set_yscale('symlog',linthresh=1e-1)
    if DESIRED_ROT_VEL == 0:
        ax3.set_ylabel("Fin Cant, Angular Position" if not USE_TABS else 'Fin Tab Angle, Angular Position (deg)')
    else:
        ax3.set_ylabel("Fin Cant (deg)" if not USE_TABS else 'Fin Tab Angle (deg)')
    ax3.spines['right'].set_color('purple')
    ax3.spines['left'].set_color('red')
    ax3.yaxis.label.set_color('purple')
    ax2.yaxis.label.set_color('red')
    ax2.tick_params(axis='y', colors='red')
    ax3.tick_params(axis='y', colors='purple')
    ax2.set_yscale('symlog',linthresh=1e-2)

    maxLim2 = max(*np.abs(ax2.get_ylim()))
    maxLim3 = max(*np.abs(ax3.get_ylim()))
    ax2.set_ylim(-maxLim2,maxLim2)
    ax3.set_ylim(-maxLim3,maxLim3)
    ax2.set_xlim(*ax2.get_xlim())
    ax2.set_ylim(-50,50)
    ax2.hlines(0,*ax2.get_xlim(),color='k',linestyle='dotted')

    ax2.set_xlabel("Time (s)")
    ax3.legend(loc='upper right')
    titleFirstLine = "\\begin{center}\\noindent \\sc Tab-Ctrlled Flight" if USE_TABS else "Cant-Ctrlled Flight"
    titleTurbLine = "; Turb. ${}\\%$".format(TURBULENCE)
    title_vPID_Line = "\\\\[0.25em] $\\omega$PID: $\\tt [{},{},{}]$ ".format(*np.round([KP_VEL,KI_VEL,KD_VEL],2)) if useVelocityPID else ""
    title_aPID_Line = "; $\\varphi$PID: $\\tt [{},{},{}]$ ".format(*np.round([KP_ANG,KI_ANG,KD_ANG],2)) if usePositionPID else ""
    title_fixed_Line = "; $\\alpha_0={}$".format(np.round(CONST_FIXED,2)) if CONST_FIXED != 0 else ""
    title_suppLine = ("; $\\omega_0={}$".format(np.round(DESIRED_ROT_VEL,2)) if useVelocityPID else "") + ("; $\\varphi_0={}$ ".format(np.round(DESIRED_ROT_ANG,2)) if usePositionPID else "")
    plt.gcf().suptitle(titleFirstLine +titleTurbLine + title_vPID_Line + title_aPID_Line + title_fixed_Line + title_suppLine + "\\end{center}", fontsize=12)

    plt.savefig(figPath)
    #plt.show()

    logger.setLevel(level=logging.INFO)


    exit(0)







