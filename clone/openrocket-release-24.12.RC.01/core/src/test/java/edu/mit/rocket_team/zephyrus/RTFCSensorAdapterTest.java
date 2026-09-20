package edu.mit.rocket_team.zephyrus;

import org.junit.jupiter.api.Test;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener;
import info.openrocket.core.util.*;
import edu.mit.rocket_team.zephyrus.instrument.RTGPS;
import edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.Trace;
import static org.junit.jupiter.api.Assertions.*;

class RTFCSensorAdapterTest extends BaseTestCase {
    private SimulationStatus status() {
        var rocket=RTFCVerificationRocket.rocket();
        var options=RTFCVerificationRocket.simulation(rocket).getOptions();
        options.setLaunchLatitude(42.7042123); options.setLaunchLongitude(-77.1919876);
        var status=new SimulationStatus(rocket.getSelectedConfiguration(),options.toSimulationConditions());
        status.setRocketOrientationQuaternion(Quaternion.rotation(Coordinate.ZERO));
        return status;
    }
    @Test void specificForceAndAtmosphereAndGpsUnits() {
        var status=status();
        double g=status.getSimulationConditions().getGravityModel().getGravity(status.getRocketWorldPosition());
        var pad=FlightControllerSimulatorListener.sample(status,Coordinate.ZERO,0);
        assertEquals(g,pad.accel().getAccelX(),1e-6); assertEquals(0,pad.accel().getAccelY()); assertEquals(0,pad.accel().getAccelZ());
        var falling=FlightControllerSimulatorListener.sample(status,new Coordinate(0,0,-g),0);
        assertEquals(0,falling.accel().getAccelX(),1e-6);
        var thrust=FlightControllerSimulatorListener.sample(status,new Coordinate(0,0,30),0);
        assertEquals(30+g,thrust.accel().getAccelX(),1e-5);
        var atmosphere=status.getSimulationConditions().getAtmosphericModel().getConditions(271);
        assertEquals(atmosphere.getPressure()/100,pad.baro().getPressure(),1e-4);
        assertEquals(atmosphere.getTemperature()-273.15,pad.baro().getTemperature(),1e-5);
        RTGPS gps=new RTGPS(new Trace(s->{})); gps.backdoorFudge(pad.gps()); gps.updateAndParse();
        assertEquals(427042123,gps.getLatE7()); assertEquals(-771919876,gps.getLonE7());
        assertEquals(271,gps.getHeight());
    }
    @Test void mountingUsesMatchingAttitudeForAccelerationAndWorldRotation() {
        var status=status();
        var orientation=Quaternion.rotation(new Coordinate(0,Math.PI/2,0));
        status.setRocketOrientationQuaternion(orientation);
        // Body rates X=2,Y=3,Z=1 rad/s, supplied in the world frame like the integrator.
        status.setRocketRotationVelocity(orientation.rotate(new Coordinate(2,3,1)));
        var input=FlightControllerSimulatorListener.sample(status,Coordinate.ZERO,123);
        double g=status.getSimulationConditions().getGravityModel().getGravity(status.getRocketWorldPosition());
        assertEquals(0,input.accel().getAccelX(),1e-6); assertEquals(-g,input.accel().getAccelY(),1e-6);
        assertEquals(Math.toDegrees(1),input.gyro().getGyroX(),1e-5);
        assertEquals(Math.toDegrees(2),input.gyro().getGyroY(),1e-5);
        assertEquals(Math.toDegrees(3),input.gyro().getGyroZ(),1e-5);
        assertEquals(123,input.acquisitionUs());
    }
}
