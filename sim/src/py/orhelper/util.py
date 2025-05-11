import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as spopt


def parseFromString(a):
    x,y,z,w = tuple([float(j) for j in np.array([i.split(":") for i in a.split(",")])[:,1]])
    return Coordinate(x,y,z,w)


class Coordinate:
    def __init__(self, x, y, z, w=0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def clone(self):
        return Coordinate(self.x, self.y, self.z, self.w)

    def pythonOutputStr(self):
        return "Coordinate({}, {}, {}, {})".format(self.x, self.y, self.z, self.w)

    """
    Dunder methods
    """
    def __str__(self):
        return "Coordinate(x {}, y {}, z {})".format(self.x, self.y, self.z)

    def __repr__(self):
        return self.pythonOutputStr()

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z and self.w == other.w

    def __ne__(self, other):
        return not self.__eq__(other)

    def __add__(self, other):
        return Coordinate(self.x + other.x, self.y + other.y, self.z + other.z, self.w + other.w)

    def __sub__(self, other):
        return Coordinate(self.x - other.x, self.y - other.y, self.z - other.z, self.w - other.w)



class SimulationStatusLocal:
    def __init__(self, pdict):
        self.pdict = pdict

    def getProperty(self, key):
        return self.pdict[key]

    def setProperty(self, key, value):
        self.pdict[key] = value

    def __str__(self):
        return str(self.pdict)





def calculateDeltDict(or_obj, dictNew, dictOld,toPrint=False):
    import logging

    quatClass = or_obj.util.Quaternion

    changeInPosition_java = dictNew["position"].clone().sub(dictOld["position"])
    changeInWorldPosition_java = dictNew["worldPos"].clone().sub(dictOld["worldPos"])
    changeInVelocity_java = dictNew["velocity"].clone().sub(dictOld["velocity"])
    changeInOrientation_java = quatClass.rotation(dictNew["orient"].getAxis().sub(dictOld["orient"].getAxis()), dictNew["orient"].getAngle() - dictOld["orient"].getAngle())
    changeInOrientationAxis_java = dictNew["orient"].getAxis().sub(dictOld["orient"].getAxis())
    changeInOrientationAngle = dictNew["orient"].getAngle() - dictOld["orient"].getAngle()
    changeInRotationVelocity_java = dictNew["rotVel"].clone().sub(dictOld["rotVel"])

    # python objects

    #print(changeInPosition_java.pythonOutputStr())
    if(toPrint):
        changeInPosition = parseFromString(str(changeInPosition_java.pythonOutputStr()))
        changeInVelocity = parseFromString(str(changeInVelocity_java.pythonOutputStr()))
        changeInOrientationAxis = parseFromString(str(changeInOrientationAxis_java.pythonOutputStr()))
        changeInRotationVelocity = parseFromString(str(changeInRotationVelocity_java.pythonOutputStr()))
        changeInOrientationAngleDeg = 180*changeInOrientationAngle/(np.pi)


        #changeInPosition = parseFromString(str(deltaDict["position"].pythonOutputStr()))
        #changeInVelocity = parseFromString(str(deltaDict["velocity"].pythonOutputStr()))
        #changeInOrientationAxis = parseFromString(str(deltaDict["orient"].getAxis().pythonOutputStr()))
        #changeInRotationVelocity = parseFromString(str(deltaDict["rotVel"].pythonOutputStr()))
        #changeInOrientationAngleDeg = 180*deltaDict["orient"].getAngle()/(np.pi)

        print("== CHANGES ==")
        print("changeInPosition: ")
        print(changeInPosition)
        print("changeInVelocity: ")
        print(changeInVelocity)
        print("changeInOrientationAxis: ")
        print(changeInOrientationAxis)
        print("changeInRotationVelocity: ")
        print(changeInRotationVelocity)
        print("changeInOrientationAngleDeg: ")
        print(changeInOrientationAngleDeg)
        print("== END CHANGES ==")

    orChg = changeInOrientation_java
    orChgAxis = orChg.getAxis()

    deltaDict = {
        "position": changeInPosition_java, # Coordinate object
        "positionX": changeInPosition_java.x, # Coordinate object
        "positionY": changeInPosition_java.y, # Coordinate object
        "positionZ": changeInPosition_java.z, # Coordinate object
        "worldPos": changeInWorldPosition_java, # WorldCoordinate object
        "velocity": changeInVelocity_java, # Coordinate object
        "velocityX": changeInVelocity_java.x, # Coordinate object
        "velocityY": changeInVelocity_java.y, # Coordinate object
        "velocityZ": changeInVelocity_java.z, # Coordinate object
        "orient"  : changeInOrientation_java, # Quaternion object
        "orientRotAxis"  : orChgAxis, # Quaternion object
        "orientRotAxisX"  : orChgAxis.x, # Quaternion object
        "orientRotAxisY"  : orChgAxis.y, # Quaternion object
        "orientRotAxisZ"  : orChgAxis.z, # Quaternion object
        "rotVel"  : changeInRotationVelocity_java, # Coordinate object
        "rotVelX"  : changeInRotationVelocity_java.x, # Coordinate object
        "rotVelY"  : changeInRotationVelocity_java.y, # Coordinate object
        "rotVelZ"  : changeInRotationVelocity_java.z, # Coordinate object
        "liftoff"  : not(dictNew["liftoff"] == dictOld["liftoff"]), # boolean
        "apogee"   : not(dictNew["apogee"] == dictOld["apogee"]), # boolean
        "motorIgn"   : not(dictNew["motorIgn"] == dictOld["motorIgn"]), # boolean
        "lnchRdClr"   : not(dictNew["lnchRdClr"] == dictOld["lnchRdClr"]), # boolean
    }
    if(deltaDict["liftoff"]):
        logging.error("Launch status changed to " + str(dictNew["liftoff"]))

    if(deltaDict["apogee"]):
        logging.error("Apogee status changed to " + str(dictNew["apogee"]))

    if(deltaDict["motorIgn"]):
        logging.error("Motor ignition status changed to " + str(dictNew["motorIgn"]))

    if(deltaDict["lnchRdClr"]):
        logging.error("Launch rod clear status changed to " + str(dictNew["lnchRdClr"]))


    return deltaDict
