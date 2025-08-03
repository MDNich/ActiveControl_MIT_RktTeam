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


import threading

VELMINTHRESH = 15
TURBULENCE = 2
KP = 10
KI = 0#.1#0.75
KD = 0.1
INI_ROT_VEL = 0
DESIRED_ROT_VEL = -1
overrideI = True
getPID_from_plant = False
figPath = 'dat/demonstrator_1/pdf/turb{}_PID_KP{}_KI{}_KD{}_desired{}_ini{}.pdf'.format(TURBULENCE,KP,KI,KD,DESIRED_ROT_VEL,INI_ROT_VEL)
CSVSAVEPATH = 'dat/demonstrator_1/csv/run_turb{}_PID_KP{}_KI{}_KD{}_desired{}_ini{}.csv'.format(TURBULENCE,KP,KI,KD,DESIRED_ROT_VEL,INI_ROT_VEL)



# Start
instance, orh, or_obj = startOR()

# Import classes
simStatClass = or_obj.simulation.SimulationStatus
coordClass = or_obj.util.Coordinate
worldCoordClass = or_obj.util.WorldCoordinate
quatClass = or_obj.util.Quaternion

# Load rocket
doc, rktObj = loadRocket(orh, 'demonstrator_1.ork')

newCtrl = or_obj.simulation.listeners.NewControlStepListener

newCtrl.theRocket = rktObj


# load flight conf
flightConfig = doc.getSelectedConfiguration()
logging.info("Motor identifier: ")
logging.info(flightConfig)

# load sim
sim = doc.getSimulation(0)
logging.warning("loaded document + simulation")

datPath = 'dat/simResults/demonstrator_1_out_long.csv'
#figPath = 'dat/simResults/demonstrator_1_out_long.pdf'


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
	if fni == "FINS_TWIST":
		theFinIndexToModify = index
		break
	index += 1

finToPlayWith = finList[theFinIndexToModify]




##### CONTROL ALGORITHM #####


def giveNextControlStep(or_obj,doc,rktObj,pastPropDictArray,currentCantAngle):
	### TODO: PID controller ???
	kP = -1
	kD = 0.01
	kI = 0

	lastCantAngle = pastPropDictArray[-1]["finCantAngleDeg"]
	lastAngularVel = pastPropDictArray[-1]["rotVel"].z

	desiredAngVel = 0
	err = lastAngularVel-desiredAngVel

	newCantAngle = err*kP #+ (currentCantAngle-lastCantAngle)*kD


	return 0#newCantAngle








#exit(0)





verboseMode = False


dataImport = False

if dataImport:
	logging.error("READING FROM A FILE: {}".format(datPath))
else:
	logging.error("RUNNING A NEW SIMULATION")

import pandas as pd
if dataImport:
	df = pd.read_csv(datPath)

	times = np.array(df["time"])
	heightTime = np.array(df["positionZ"])
	vertVelTime = np.array(df["velocityZ"])
	motorEndLoc = np.argmax(vertVelTime)
	motorEndTime = times[motorEndLoc]

else:
	# startParams
	alt0 = 10 # m
	v0x = 0 # m/s
	v0y = 0 # m/s
	v0z = 0 # m/s

	rotAxis = coordClass(0,0,1) # z axis, for now
	angle = 0.0

	omega0x = 0.0
	omega0y = 0.0
	omega0z = INI_ROT_VEL#10.0


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
	midCtrl = or_obj.simulation.listeners.MidControlStepLauncher
	midCtrl.theTimeStep = prefDt
	#theTimeStep
	nrunsPredict = int(runTime/likelyDt)

	import time
	iniTime = time.time()
	logging.info("Begin simulation with {} timeSteps for total runTime of {} with dt {}.".format(nrunsPredict,runTime,prefDt))
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

	newCtrl.desiredRotVel = DESIRED_ROT_VEL
	newCtrl.IdecayFactor = 1

	if(newCtrl.desiredRotVel == 0):
		newCtrl.kP = 3e-1
	else:
		newCtrl.kP = 1

	#newCtrl.kI = 1e-3
	#newCtrl.kD = 0.1
	newCtrl.constFixed = 0
	if(newCtrl.desiredRotVel == 0):
		newCtrl.kVelRocket = 3.5
		newCtrl.kVel2Rocket = 1
		newCtrl.kVel3Rocket = 2
	#newCtrl.kAccelRocket = 1e-2

	# get PID coefficients from algorithm generating.

	ts_des = 0.8 # Desired settling time of 0.5 seconds
	Mp_des = 0.2 # Desired max peak of less than 20%
	s0 = set_dominant_poles(ts_des, Mp_des)
	print(f'Desired Poles: {s0} and its conjugate')

	gamma = 2 # Assume ratio between two compensator zeros.  Between 1-3 said to be a heuristic range
	kP, kI, kD = return_PID_coeffs(G_plant, gamma, s0, show=True)
	print("Obtained from PLANT dynamics: kP: {}, kI: {}, kD: {}".format(kP,kI,kD))
	newCtrl.useRK6 = True

	if overrideI:
		kI = 0

	if not getPID_from_plant:
		print("Override: using PID coeffs kP: {}, kI: {}, kD: {}".format(KP,KI,KD))
		newCtrl.kP = KP#2#100#kP
		newCtrl.kI = KI#.75#kI/10
		newCtrl.kD = KD#.1
	else:
		print("Using PID coefficients from plant dynamics.")

		figPath = 'dat/demonstrator_1/pdf/turb{}_PID_KP{}_KI{}_KD{}_desired{}_ini{}.pdf'.format(TURBULENCE,*np.round([kP,kI,kD],2),DESIRED_ROT_VEL,INI_ROT_VEL)
		CSVSAVEPATH = 'dat/demonstrator_1/csv/run_turb{}_PID_KP{}_KI{}_KD{}_desired{}_ini{}.csv'.format(TURBULENCE,*np.round([kP,kI,kD],2),DESIRED_ROT_VEL,INI_ROT_VEL)
		newCtrl.kP = kP
		newCtrl.kI = kI
		newCtrl.kD = kD
	newCtrl.kVelRocket = 0

	newCtrl.servoStepCount = 4095.0
	newCtrl.velMinThresh = VELMINTHRESH

	newCtrl.invVelSqCoeff = 5e6 # roughly maxvel^2

	print("Calculated PID Coeffs P: {} I: {} D: {}".format(kP,kI,kD))
	print("Setting PID Coeffs P: {} I: {} D: {}".format(newCtrl.kP,newCtrl.kI,newCtrl.kD))

	simThread = threading.Thread(target=lambda: sim.simulate(listener_array))
	simThread.start()

	simThread.join()
	logging.info("Simulation done, plotting.")


	omegaZ = np.array(newCtrl.pastOmegaZ)
	finCantLog = np.array(newCtrl.finCantLog)


	data = orh.get_timeseries(sim, [dT.TYPE_ALTITUDE,dT.TYPE_VELOCITY_Z,dT.TYPE_TIME])

	t = np.array(data[dT.TYPE_TIME].tolist())
	alt = np.array(data[dT.TYPE_ALTITUDE].tolist())
	vel = np.array(data[dT.TYPE_VELOCITY_Z].tolist())


	logger = logging.getLogger()
	logger.setLevel(level=logging.ERROR)




	# SAVE DATA TO CSV


	import matplotlib.pyplot as plt

	apogeeInd = alt.argmax()
	np.set_printoptions(legacy='1.13')
	print(finCantLog[:apogeeInd])

	dataArr = np.array([t[:apogeeInd],alt[:apogeeInd],vel[:apogeeInd],omegaZ[:apogeeInd],finCantLog[:apogeeInd]])
	np.savetxt(CSVSAVEPATH, dataArr.T, delimiter=',', header='Time (s),Altitude (m),Velocity (m/s),Angular Velocity (rad/s),Fin Cant Angle (deg)', comments='')
	print("Saved data to {}".format(CSVSAVEPATH))


	fig, axs = plt.subplots(nrows=2,sharex='col')
	ax = axs[0]
	ax2 = axs[1]
	ax.plot(t[:apogeeInd],vel[:apogeeInd],label="Velocity",color='blue')
	ax.set_xlim(t[0],t[apogeeInd-1])
	ax.set_ylim(0,ax.get_ylim()[1])
	ax.plot([-1],[-1],label="Altitude",color='red')
	ax0 = ax.twinx()
	ax0.plot(t[:apogeeInd],alt[:apogeeInd],label="Altitude",color='red')
	ax0.set_ylim(0,ax0.get_ylim()[1])

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
	ax3.plot(t[:apogeeInd],finCantLog[:apogeeInd],label="Fin Cant",color='purple',alpha=0.7)
	ax3.set_ylabel("Fin Cant")
	ax3.spines['right'].set_color('purple')
	ax3.spines['left'].set_color('red')
	ax3.yaxis.label.set_color('purple')
	ax2.yaxis.label.set_color('red')
	ax2.tick_params(axis='y', colors='red')
	ax3.tick_params(axis='y', colors='purple')
	ax2.set_yscale('symlog',linthresh=1e-2)
	#ax.vlines(t[apogeeInd],*ax.get_xlim(),color='k',linestyle='dotted')

	maxLim2 = max(*np.abs(ax2.get_ylim()))
	maxLim3 = max(*np.abs(ax3.get_ylim()))
	ax2.set_ylim(-maxLim2,maxLim2)
	ax3.set_ylim(-maxLim3,maxLim3)
	ax2.set_xlim(*ax2.get_xlim())
	ax2.set_ylim(-50,50)
	ax2.hlines(0,*ax2.get_xlim(),color='k',linestyle='dotted')

	ax2.set_xlabel("Time (s)")
	#ax2.legend(loc='upper right')

	plt.savefig(figPath)
	#plt.show()

	logger.setLevel(level=logging.INFO)


	exit(0)





	exit(0)













	# sim
	print("Started sim")
	# the simulation will have run just one step; we need to save data before allowing it to continue.
	i = 0
	while not apogeeFound:
		if(i%(int(nrunsPredict/10)) == 0):
			print("approx " + str(int(np.round(i/nrunsPredict*10,0)*10)) + "% done, simulation time "+ str(times[-1]) + " s")

		newParamDict, deltaDict = runOneStep(or_obj, flightConfig, sim, currenPropDict,timeStep=prefDt,verbose=verboseMode)
		print("OneStep results saved")
		heightTime.append(newParamDict["position"].z)
		vertVelTime.append(newParamDict["velocity"].z)
		actualdt = or_obj.simulation.listeners.MidControlStepLauncher.theTimeStep
		#print("Got actualdt {}".format(actualdt))
		dtList.append(actualdt)
		times.append(times[-1]+dtList[-1])
		dictList[-1]["time"] = times[-1]
		dictList.append(newParamDict.copy())
		currenPropDict = newParamDict.copy()
		if not apogeeFound:
			if(np.abs(np.argmax(heightTime)-i) > 5):
				logging.error("APOGEE REACHED at iter {} time {}".format(i,times[-1]))
				apogeeFound = True

		if not motorEnded:
			if(np.abs(np.argmax(vertVelTime)-i) > 5):
				logging.error("MOTOR ENDED at iter {} time {}".format(i-5,times[-1]))
				motorEnded = True
				motorEndLoc = i-5
		i += 1







		midCtrl.readyToProceed = True
		print("Python says GO")
		if(times[-1] > runTime):
			break
	#logging.warning("COMPLETED CYCLE {}.".format(i+1))
	endTime = time.time()
	logging.info("Finished in {} s.".format(endTime-iniTime))
	logging.info("Average dt: {} s".format(np.mean(dtList)))
	logging.info("Average time per cycle: {} ms.".format((endTime-iniTime)*1e3/i))

	if(verboseMode):
		logging.warning("=== FINAL RESULTS ===")
		for key in dictList[-1].keys():
			logging.info("Key {}:".format(key))
			logging.info(dictList[-1][key])
		logging.warning("=== END FINAL RESULTS ===")

	deltaDictFinal = calculateDeltDict(or_obj, dictList[-1], dictList[0],toPrint=False)






	df = pd.DataFrame(dictList)
	df.to_csv(datPath, index=False)

	"""
    try:
        f = open(datPath,"x")
    except:
        f = open(datPath,"w")
    f.write(str(times))
    f.write("\n")
    f.write(str(heightTime))
    f.write("\n")
    f.write(str(vertVelTime))
    f.close()"""
	orhelper.logger.info("File Saved at {}".format(datPath))

	times = np.array(times)
	heightTime = np.array(heightTime)
	vertVelTime = np.array(vertVelTime)







logger = logging.getLogger()
logger.setLevel(level=logging.ERROR)




import matplotlib.pyplot as plt

fig, ax = plt.subplots()
ax.plot(times,np.array(heightTime),label="Height",color='red')
ax.set_xlim(*ax.get_xlim())
ax.set_ylim(0,7500)
ax.plot([-1],[-1],label="Vert Velocity", color='blue')
ax.vlines(times[motorEndLoc],*ax.get_ylim(),color='green',linestyle="dotted",label='Motor End')
ax.vlines(times[~np.isnan(np.array(times))][-1],*ax.get_ylim(),color='k',linestyle="dotted",label='Apogee')
ax.legend(ncol=1)

ax2 = ax.twinx()
ax2.plot(times,np.array(vertVelTime),color='blue')#,label="Vert Velocity",color='blue')
ax2.set_ylim(0,200)

ax2.spines['right'].set_color('b')
ax2.spines['left'].set_color('r')
ax2.yaxis.label.set_color('b')
ax.yaxis.label.set_color('r')
ax.tick_params(axis='y', colors='r')

ax.set_ylabel("Height (m)")
ax2.set_ylabel("Vertical Velocity (m/s)")
ax.set_xlabel("Time (s)")
plt.savefig(figPath)
plt.show()




logger.setLevel(level=logging.INFO)

"""



simStatClass = or_obj.simulation.SimulationStatus

theNextSimulationStatus = simStatClass(flightConfig, sim.getOptions().toSimulationConditions())

coordClass = or_obj.util.Coordinate
worldCoordClass = or_obj.util.WorldCoordinate
quatClass = or_obj.util.Quaternion


alt0 = 100 # m
v0x = 0.3 # m/s
v0y = 0.1 # m/s
v0z = 100.0 # m/s

rotAxis = coordClass(0,0,1) # z axis, for now
angle = 0.0

omega0x = 0.0
omega0y = 0.0
omega0z = 0.0


propDict = {
    "position0": coordClass(0,0,alt0,12500), # Coordinate object
    "positionPrint0": coordClass(0,0,alt0,12500).pythonOutputStr(), # Coordinate object
    "worldPos0": worldCoordClass(28.61,-80.6,100.125), # WorldCoordinate object
    "velocity0": coordClass(v0x,v0y,v0z,0), # Coordinate object
    "velocityPrint0": coordClass(v0x,v0y,v0z,0).pythonOutputStr(), # Coordinate object
    "orient0"  : quatClass.rotation(rotAxis, angle), # Quaternion object
    "orientPrint0"  : quatClass.rotation(rotAxis, angle).printAxisAngle(), # Quaternion object
    "rotVel0"  : coordClass(omega0x,omega0y,omega0z), # Coordinate object
    "liftoff"  : True, # boolean
    "apogee"   : False, # boolean
    "motorIgn" : True, # boolean
    "lnchRdClr": True, # boolean
}
logging.warning("== INITIAL CONDITIONS ==")
for key in propDict.keys():
    logging.info("Key {}:".format(key))
    logging.info(propDict[key])

logging.warning("== END INITIAL CONDITIONS ==")

# Set according to propdict.
theNextSimulationStatus.setRocketPosition(propDict["position0"])
theNextSimulationStatus.setRocketWorldPosition(propDict["worldPos0"])
theNextSimulationStatus.setRocketVelocity(propDict["velocity0"])
theNextSimulationStatus.setRocketOrientationQuaternion(propDict["orient0"])
theNextSimulationStatus.setRocketRotationVelocity(propDict["rotVel0"])
theNextSimulationStatus.liftoff = propDict["liftoff"]
theNextSimulationStatus.apogeeReached = propDict["apogee"]
theNextSimulationStatus.motorIgnited = propDict["motorIgn"]
theNextSimulationStatus.launchRodCleared = propDict["lnchRdClr"]

listenerClass = or_obj.simulation.listeners.MidControlStepLauncher
listenerClass.provideSimStat(theNextSimulationStatus)

listener_array = [listenerClass()]

try:
    # Need to do this otherwise exact same numbers will be generated for each identical run
    sim.getOptions().randomizeSeed()

    # sim
    sim.simulate(listener_array)
    logging.warning("Simulation finished")
except Exception as e:
    logging.error("Java Error: {}".format(str(e)))
    logging.error("Caught it !")

theEndingSimulationStatus = listenerClass.getFinStat()
#theEndingSimulationStatus.storeData()
#datBranch = theEndingSimulationStatus.getFlightDataBranch()

outDict = {
    "position1": theEndingSimulationStatus.getRocketPosition(), # Coordinate object
    "positionPrint1": theEndingSimulationStatus.getRocketPosition().pythonOutputStr(), # Coordinate object
    "worldPos1": theEndingSimulationStatus.getRocketWorldPosition(), # WorldCoordinate object
    "velocity1": theEndingSimulationStatus.getRocketVelocity(), # Coordinate object
    "velocityPrint1": theEndingSimulationStatus.getRocketVelocity().pythonOutputStr(), # Coordinate object
    "orient1"  : theEndingSimulationStatus.getRocketOrientationQuaternion(), # Quaternion object
    "orientPrint1"  : theEndingSimulationStatus.getRocketOrientationQuaternion().printAxisAngle(), # Quaternion object
    "rotVel1"  : theEndingSimulationStatus.getRocketRotationVelocity(), # Coordinate object
    "liftoff"  : theEndingSimulationStatus.isLiftoff(), # boolean
    "apogee"   : theEndingSimulationStatus.isApogeeReached(), # boolean
    "motorIgn" : theEndingSimulationStatus.isMotorIgnited(), # boolean
    "lnchRdClr": theEndingSimulationStatus.isLaunchRodCleared(), # boolean
}


logging.warning("== FINAL CONDITIONS ==")
for key in outDict.keys():
    logging.info("Key {}:".format(key))
    logging.info(outDict[key])
logging.warning("== END FINAL CONDITIONS ==")


# calculate deltas

changeInPosition_java = outDict["position1"].clone().sub(propDict["position0"])
changeInVelocity_java = outDict["velocity1"].clone().sub(propDict["velocity0"])
changeInOrientationAxis_java = outDict["orient1"].getAxis().sub(propDict["orient0"].getAxis())
changeInOrientationAngle = outDict["orient1"].getAngle() - propDict["orient0"].getAngle()
changeInRotationVelocity_java = outDict["rotVel1"].clone().sub(propDict["rotVel0"])

# python objects

print(changeInPosition_java.pythonOutputStr())

changeInPosition = parseFromString(str(changeInPosition_java.pythonOutputStr()))
changeInVelocity = parseFromString(str(changeInVelocity_java.pythonOutputStr()))
changeInOrientationAxis = parseFromString(str(changeInOrientationAxis_java.pythonOutputStr()))
changeInRotationVelocity = parseFromString(str(changeInRotationVelocity_java.pythonOutputStr()))
changeInOrientationAngleDeg = 180*changeInOrientationAngle/(np.pi)

logging.warning("== CHANGES ==")
print("changeInPosition: ")
print(changeInPosition)
print("changeInVelocity: ")
print(changeInVelocity)
print("changeInOrientationAxis: ")
print(changeInOrientationAxis)
print("changeInOrientationAngle: ")
print(str(changeInOrientationAngleDeg) + "°")
print("changeInRotationVelocity: ")
print(changeInRotationVelocity)
logging.warning("== END CHANGES ==")








#print(theEndingSimulationStatus.toEventDebug())

"""

"""
Desired simulation start settings:

altitude : 200m
speed : 100 m/s


Parameters to set :

```java

public SimulationStatus(FlightConfiguration configuration, SimulationConditions simulationConditions) {

    this.simulationConditions = simulationConditions;
    this.configuration = configuration;

    this.time = 0;
    this.position = this.simulationConditions.getLaunchPosition();
    this.velocity = this.simulationConditions.getLaunchVelocity();
    this.worldPosition = this.simulationConditions.getLaunchSite();

    // Initialize to roll angle with least stability w.r.t. the wind
    Quaternion o;
    FlightConditions cond = new FlightConditions(this.configuration);
    double angle = -cond.getTheta() - (Math.PI / 2.0 - this.simulationConditions.getLaunchRodDirection());
    o = Quaternion.rotation(new Coordinate(0, 0, angle));

    // Launch rod angle and direction
    o = o.multiplyLeft(Quaternion.rotation(new Coordinate(0, this.simulationConditions.getLaunchRodAngle(), 0)));
    o = o.multiplyLeft(Quaternion.rotation(new Coordinate(0, 0, Math.PI / 2.0 - this.simulationConditions.getLaunchRodDirection())));
    
    this.orientation = o;
    this.rotationVelocity = Coordinate.NUL;

    /*
     * Calculate the effective launch rod length taking into account launch lugs.
     * If no lugs are found, assume a tower launcher of full length.
     */
    double length = this.simulationConditions.getLaunchRodLength();
    double lugPosition = Double.NaN;
    for (RocketComponent c : this.configuration.getActiveComponents()) {
        if (c instanceof LaunchLug) {
            double pos = c.toAbsolute(new Coordinate(c.getLength()))[0].x;
            if (Double.isNaN(lugPosition) || pos > lugPosition) {
                lugPosition = pos;
            }
        }
    }
    if (!Double.isNaN(lugPosition)) {
        double maxX = 0;
        for (Coordinate c : this.configuration.getBounds()) {
            if (c.x > maxX)
                maxX = c.x;
        }
        if (maxX >= lugPosition) {
            length = Math.max(0, length - (maxX - lugPosition));
        }
    }
    this.effectiveLaunchRodLength = length;

    this.simulationStartWallTime = System.nanoTime();

    this.motorIgnited = false;
    this.liftoff = false;
    this.launchRodCleared = false;
    this.apogeeReached = false;

    this.populateMotors();
    this.warnings = new WarningSet();
}
```

"""


exit(0)






data = orh.get_timeseries(sim, [dT.TYPE_PITCH_RATE,dT.TYPE_VELOCITY_TOTAL,dT.TYPE_TIME])

t = np.array(data[dT.TYPE_TIME].tolist())
pR = np.array(data[dT.TYPE_PITCH_RATE].tolist())
vel = np.array(data[dT.TYPE_VELOCITY_TOTAL].tolist())










