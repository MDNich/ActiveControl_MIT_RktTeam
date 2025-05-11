#!/usr/local/python3

# MUST BE RUN FROM TOP LEVEL WORKING DIRECTORY: 'sim'.

import os
import numpy as np
import jpype

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
with orhelper.OpenRocketInstance(os.environ['CLASSPATH'], 'OFF') as instance:
	orh = orhelper.Helper(instance)
	or_obj = jpype.JPackage("info").openrocket.core

	doc = orh.load_doc('dat/ork/canard1.ork')
	rktObj = doc.getRocket()


	flightConfig = doc.getSelectedConfiguration()
	print("Motor identifier: ")
	print(flightConfig)

	sim = doc.getSimulation(0)
	print("loaded document + simulation")

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



	#print(theEndingSimulationStatus.toEventDebug())



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









	data = orh.get_timeseries(sim, [dT.TYPE_PITCH_RATE,dT.TYPE_VELOCITY_TOTAL,dT.TYPE_TIME])

	t = np.array(data[dT.TYPE_TIME].tolist())
	pR = np.array(data[dT.TYPE_PITCH_RATE].tolist())
	vel = np.array(data[dT.TYPE_VELOCITY_TOTAL].tolist())

	writePath = 'dat/simResults/canard1_out.txt'
	figPath = 'dat/simResults/canard1_out.pdf'
	try:
		f = open(writePath,"x")
	except:
		f = open(writePath,"w")
	f.write(str(t.tolist()))
	f.write("\n")
	f.write(str(pR.tolist()))
	f.write("\n")
	f.write(str(vel.tolist()))
	f.close()
	orhelper.logger.info("File Saved at {}".format(writePath))
	logger = logging.getLogger()
	logger.setLevel(level=logging.ERROR)



	"""import matplotlib.pyplot as plt


	fig, ax = plt.subplots()
	#ax.plot(t,pR,label="Pitch Rate",color='blue')
	#ax2 = ax.twinx()
	ax.plot(t,vel,label="Velocity",color='red')
	ax.legend()
	#ax2.legend(loc='upper right')
	plt.savefig(figPath)
	#plt.show()"""



	logger.setLevel(level=logging.INFO)















