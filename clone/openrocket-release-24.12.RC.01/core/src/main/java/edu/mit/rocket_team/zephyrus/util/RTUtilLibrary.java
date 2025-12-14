package edu.mit.rocket_team.zephyrus.util;

import info.openrocket.core.util.Coordinate;

public class RTUtilLibrary {

    /**
     * Converts rocket roll & pitch into IMU-measured X, Y, Z angles.
     *
     * @param thisRollAngle  roll angle in radians (rotation about Z)
     * @param thisPitchAngle pitch angle in radians (rotation about X)
     * @return Coordinate(xAngle, yAngle, zAngle) in radians
     */
    public static Coordinate convertToImuAngles(double thisRollAngle, double thisPitchAngle) {

        double cr = Math.cos(thisRollAngle);
        double sr = Math.sin(thisRollAngle);
        double cp = Math.cos(thisPitchAngle);
        double sp = Math.sin(thisPitchAngle);

        // Rotation matrix R = Rz(roll) * Rx(pitch)
        double r11 = cr;
        double r12 = -sr * cp;
        double r13 = sr * sp;

        double r21 = sr;
        double r22 = cr * cp;
        double r23 = -cr * sp;

        double r31 = 0.0;
        double r32 = sp;
        double r33 = cp;

        // Extract IMU Euler angles (X-Y-Z order)
        double xAngle = Math.atan2(r32, r33);   // rotation about X
        double yAngle = Math.asin(-r31);        // rotation about Y (≈ 0)
        double zAngle = Math.atan2(r21, r11);   // rotation about Z

        return new Coordinate(xAngle, yAngle, zAngle);
    }

}
