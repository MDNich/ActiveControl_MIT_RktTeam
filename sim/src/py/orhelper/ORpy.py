import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as spopt


import os
import numpy as np
import jpype
from src.py.orhelper.util import *

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

def startOR():
    instance = orhelper.OpenRocketInstance(os.environ['CLASSPATH'], 'OFF')
    instance.__enter__()
    orh = orhelper.Helper(instance)
    or_obj = jpype.JPackage("info").openrocket.core

    return instance, orh, or_obj

def loadRocket(orh, orkName):
    doc = orh.load_doc('dat/ork/' + str(orkName))
    rktObj = doc.getRocket()
    return doc, rktObj

def runOneStep(or_obj, flightConfig, sim, startParamDict,timeStep=0.0025,verbose=False):
    simStatClass = or_obj.simulation.SimulationStatus

    theStartingSimulationStatus = simStatClass(flightConfig, sim.getOptions().toSimulationConditions())

    theStartingSimulationStatus.simulationConditions.setTimeStep(timeStep)

    propDict = startParamDict

    if(verbose):
        logging.warning("== INITIAL CONDITIONS ==")
        for key in propDict.keys():
            logging.info("Key {}:".format(key))
            logging.info(propDict[key])

        logging.warning("== END INITIAL CONDITIONS ==")

    # Set according to propdict.
    theStartingSimulationStatus.setRocketPosition(propDict["position"])
    theStartingSimulationStatus.setRocketWorldPosition(propDict["worldPos"])
    theStartingSimulationStatus.setRocketVelocity(propDict["velocity"])
    theStartingSimulationStatus.setRocketOrientationQuaternion(propDict["orient"])
    theStartingSimulationStatus.setRocketRotationVelocity(propDict["rotVel"])
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
        if(verbose):
            logging.warning("Simulation finished")
    except Exception as e:
        if(verbose):
            logging.error("Java Error: {}".format(str(e)))
            logging.error("Caught it !")

    theEndingSimulationStatus = listenerClass.getFinStat()
    #theEndingSimulationStatus.storeData()
    #datBranch = theEndingSimulationStatus.getFlightDataBranch()

    outDict = {
        "position": theEndingSimulationStatus.getRocketPosition(), # Coordinate object
        "positionPrint": theEndingSimulationStatus.getRocketPosition().pythonOutputStr(), # Coordinate object
        "worldPos": theEndingSimulationStatus.getRocketWorldPosition(), # WorldCoordinate object
        "velocity": theEndingSimulationStatus.getRocketVelocity(), # Coordinate object
        "velocityPrint": theEndingSimulationStatus.getRocketVelocity().pythonOutputStr(), # Coordinate object
        "orient"  : theEndingSimulationStatus.getRocketOrientationQuaternion(), # Quaternion object
        "orientPrint"  : theEndingSimulationStatus.getRocketOrientationQuaternion().printAxisAngle(), # Quaternion object
        "rotVel"  : theEndingSimulationStatus.getRocketRotationVelocity(), # Coordinate object
        "liftoff"  : theEndingSimulationStatus.isLiftoff(), # boolean
        "apogee"   : theEndingSimulationStatus.isApogeeReached(), # boolean
        "motorIgn" : theEndingSimulationStatus.isMotorIgnited(), # boolean
        "lnchRdClr": theEndingSimulationStatus.isLaunchRodCleared(), # boolean
    }

    if(verbose):
        logging.warning("== FINAL CONDITIONS ==")
        for key in outDict.keys():
            logging.info("Key {}:".format(key))
            logging.info(outDict[key])
        logging.warning("== END FINAL CONDITIONS ==")


    # calculate deltas

    deltaDict = calculateDeltDict(or_obj, outDict, propDict,toPrint=verbose)


    return outDict, deltaDict


