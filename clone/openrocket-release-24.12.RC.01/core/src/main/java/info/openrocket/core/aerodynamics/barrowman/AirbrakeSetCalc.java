package info.openrocket.core.aerodynamics.barrowman;

import info.openrocket.core.aerodynamics.AerodynamicForces;
import info.openrocket.core.aerodynamics.FlightConditions;
import info.openrocket.core.logging.WarningSet;
import info.openrocket.core.rocketcomponent.AirbrakeSet;
import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.util.MathUtil;
import info.openrocket.core.util.Transformation;

import static info.openrocket.core.rocketcomponent.AirbrakeSet.*;

public class AirbrakeSetCalc extends RocketComponentCalc {

    private AirbrakeSet localRef = null;

    public AirbrakeSetCalc(RocketComponent component) {
        super(component);
        localRef = (AirbrakeSet) component;
    }

    @Override
    public void calculateNonaxialForces(FlightConditions conditions, Transformation transform, AerodynamicForces forces, WarningSet warnings) {
        // no-op
    }

    @Override
    public double calculateFrictionCD(FlightConditions conditions, double componentCf, WarningSet warnings) {
        return 0;
    }

    @Override
    public double calculatePressureCD(FlightConditions conditions, double stagnationCD, double baseCD, WarningSet warnings) {
        double area = localRef.getExposedArea();
        double refArea = conditions.getRefArea();
        double a = conditions.getAOA();
        double cos2 = Math.cos(a)*Math.cos(a);
        double sin2 = Math.sin(a)*Math.sin(a);
        if (area > MathUtil.EPSILON && refArea > MathUtil.EPSILON) {
            return area/refArea * (CD_perp*cos2 + CD_par*sin2)*overrideCD;
        }
        return 0;
    }
}
