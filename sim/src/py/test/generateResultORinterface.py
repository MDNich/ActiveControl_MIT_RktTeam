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

for runTrial in range(1,21):
    print("\n\n\n\n\n\n\n\n")
    print("RUN NUMBER {}".format(runTrial))
    print("\n\n\n\n")
    logging.info("RUN NUMBER {}".format(runTrial))


    # Import classes
    simStatClass = or_obj.simulation.SimulationStatus
    coordClass = or_obj.util.Coordinate
    worldCoordClass = or_obj.util.WorldCoordinate
    quatClass = or_obj.util.Quaternion

    # Load rocket
    doc, rktObj = loadRocket(orh, 'demonstrator_1.ork')


    # load flight conf
    flightConfig = doc.getSelectedConfiguration()
    logging.info("Motor identifier: ")
    logging.info(flightConfig)

    # load sim
    sim = doc.getSimulation(0)
    logging.warning("loaded document + simulation")

    datPath = 'dat/simResults/demonstrator_1_out_{}.csv'.format(runTrial)
    figPath = 'dat/simResults/demonstrator_1_out_{}.pdf'.format(runTrial)


    verboseMode = True
    dataImport = False

    if dataImport:
        logging.error("READING FROM A FILE: {}".format(datPath))
    else:
        logging.error("RUNNING A NEW SIMULATION")
        print("RUNNING A NEW SIMULATION")

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

        runTime = 70 # s
        prefDt = 0.0025 # s/cycle
        likelyDt = 0.0024
        dtList = []
        or_obj.simulation.listeners.MidControlStepLauncher.theTimeStep = prefDt
        #theTimeStep
        nrunsPredict = int(runTime/likelyDt)

        import time
        iniTime = time.time()
        logging.info("Begin simulation with {} timeSteps for total runTime of {} with dt {}.".format(nrunsPredict,runTime,prefDt))
        print("Begin simulation with {} timeSteps for total runTime of {} with dt {}.".format(nrunsPredict,runTime,prefDt))
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
                    print("APOGEE REACHED at iter {} time {}".format(i,times[-1]))
                    apogeeFound = True

            if not motorEnded:
                if(np.abs(np.argmax(vertVelTime)-i) > 5):
                    print("MOTOR ENDED at iter {} time {}".format(i-5,times[-1]))
                    motorEnded = True
                    motorEndLoc = i-5
            i += 1
            if(times[-1] > runTime):
                break

            endTime = time.time()
            #logging.info("Finished in {} s.".format(endTime-iniTime))
            print("Finished in {} s.".format(endTime-iniTime))
            #logging.info("Average dt: {} s".format(np.mean(dtList)))
            print("Average dt: {} s".format(np.mean(dtList)))
            #logging.info("Average time per cycle: {} ms.".format((endTime-iniTime)*1e3/i))
            print("Average time per cycle: {} ms.".format((endTime-iniTime)*1e3/i))

            if(verboseMode):
                print("=== FINAL RESULTS ===")
                for key in dictList[-1].keys():
                    print("Key {}:".format(key))
                    print(dictList[-1][key])
                print("=== END FINAL RESULTS ===")
                print("COMPLETED CYCLE {}.".format(i+1))


        deltaDictFinal = calculateDeltDict(or_obj, dictList[-1], dictList[0],toPrint=True)






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
        print("File Saved at {}".format(datPath))

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
    #plt.show()

    logger.setLevel(level=logging.INFO)
    #jpype.shutdownJVM()
















