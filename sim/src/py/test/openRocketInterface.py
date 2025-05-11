#!/usr/local/python3

# MUST BE RUN FROM TOP LEVEL WORKING DIRECTORY: 'sim'.

import os
import numpy as np
import jpype
from src.py.orhelper.util import *
from src.py.orhelper.ORpy import *

#os.chdir('../../../')

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
os.environ['JAVA_HOME'] = '/opt/homebrew/Cellar/openjdk/23.0.2'
os.environ['CLASSPATH'] = './out/OpenRocket.jar'

# Start
instance, orh, or_obj = startOR()

# Import classes
simStatClass = or_obj.simulation.SimulationStatus
coordClass = or_obj.util.Coordinate
worldCoordClass = or_obj.util.WorldCoordinate
quatClass = or_obj.util.Quaternion

# Load rocket
doc, rktObj = loadRocket(orh, 'canard1.ork')


# load flight conf
flightConfig = doc.getSelectedConfiguration()
logging.info("Motor identifier: ")
logging.info(flightConfig)

# load sim
sim = doc.getSimulation(0)
logging.warning("loaded document + simulation")

datPath = 'dat/simResults/canard1_out_long.csv'
figPath = 'dat/simResults/canard1_out_long.pdf'



#rktObj.getTopmostStage().




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
	v0x = 0.3 # m/s
	v0y = 0.1 # m/s
	v0z = 100 # m/s

	rotAxis = coordClass(0,0,1) # z axis, for now
	angle = 0.0

	omega0x = 0.0
	omega0y = 0.0
	omega0z = 0.0


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

	runTime = 80 # s
	prefDt = 0.0025 # s/cycle
	likelyDt = 0.0024
	dtList = []
	or_obj.simulation.listeners.MidControlStepLauncher.theTimeStep = prefDt
	#theTimeStep
	nrunsPredict = int(runTime/likelyDt)

	import time
	iniTime = time.time()
	logging.info("Begin simulation with {} timeSteps for total runTime of {} with dt {}.".format(nrunsPredict,runTime,prefDt))
	apogeeFound = False
	motorEnded = False
	motorEndLoc = 0

	i = 0
	while not apogeeFound:
		if(i%(int(nrunsPredict/10)) == 0):
			print("approx " + str(int(np.round(i/nrunsPredict*10,0)*10)) + "% done, simulation time "+ str(times[-1]) + " s")
		newParamDict, deltaDict = runOneStep(or_obj, flightConfig, sim, currenPropDict,timeStep=prefDt,verbose=verboseMode)
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

theStartingSimulationStatus = simStatClass(flightConfig, sim.getOptions().toSimulationConditions())

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
theStartingSimulationStatus.setRocketPosition(propDict["position0"])
theStartingSimulationStatus.setRocketWorldPosition(propDict["worldPos0"])
theStartingSimulationStatus.setRocketVelocity(propDict["velocity0"])
theStartingSimulationStatus.setRocketOrientationQuaternion(propDict["orient0"])
theStartingSimulationStatus.setRocketRotationVelocity(propDict["rotVel0"])
theStartingSimulationStatus.liftoff = propDict["liftoff"]
theStartingSimulationStatus.apogeeReached = propDict["apogee"]
theStartingSimulationStatus.motorIgnited = propDict["motorIgn"]
theStartingSimulationStatus.launchRodCleared = propDict["lnchRdClr"]

listenerClass = or_obj.simulation.listeners.MidControlStepLauncher
listenerClass.provideSimStat(theStartingSimulationStatus)

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

















