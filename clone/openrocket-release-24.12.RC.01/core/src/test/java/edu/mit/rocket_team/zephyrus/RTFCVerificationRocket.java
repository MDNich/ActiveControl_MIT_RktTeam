package edu.mit.rocket_team.zephyrus;
import java.nio.file.*;
import java.util.*;
import java.util.function.Consumer;
import info.openrocket.core.util.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.rocketcomponent.position.AxialMethod;
import info.openrocket.core.motor.*;
import info.openrocket.core.document.*;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.listeners.*;
import edu.mit.rocket_team.zephyrus.FC.RTFC;

/** Shared synthetic fixture for JUnit and packaged-JAR verification; no actual launch data. */
public final class RTFCVerificationRocket {
    public static Rocket rocket() {
        Rocket rocket=new Rocket(); rocket.setName("Synthetic FC verification — not a launch prediction");
        FlightConfigurationId id=TestRockets.TEST_FCID_0; rocket.createFlightConfiguration(id); rocket.setSelectedConfiguration(id);
        AxialStage stage=new AxialStage(); rocket.addChild(stage);
        NoseCone nose=new NoseCone(Transition.Shape.OGIVE,0.4,0.06); stage.addChild(nose);
        BodyTube body=new BodyTube(2,0.06,0.002); stage.addChild(body);
        TrapezoidFinSet fins=new TrapezoidFinSet(4,0.3,0.15,0.1,0.18); fins.setThickness(0.005); fins.setAxialMethod(AxialMethod.BOTTOM); body.addChild(fins);
        AirbrakeSet brakes=new AirbrakeSet(0.08,0.04,0.005,0.04,0.1,4); brakes.setName("FC verification airbrakes"); body.addChild(brakes); brakes.setFracExposed(0);
        Parachute chute=new Parachute(); chute.setDiameter(1.2); body.addChild(chute);
        chute.getDeploymentConfigurations().getDefault().setDeployEvent(DeploymentConfiguration.DeployEvent.APOGEE);
        body.setMotorMount(true);
        ThrustCurveMotor motor=new ThrustCurveMotor.Builder().setManufacturer(Manufacturer.getManufacturer("Verification"))
            .setDesignation("FC-test-800").setDescription("Synthetic verification only").setMotorType(Motor.Type.SINGLE).setStandardDelays(new double[]{})
            .setDiameter(0.08).setLength(0.5).setTimePoints(new double[]{0,0.1237,4,4.1}).setThrustPoints(new double[]{0,800,800,0})
            .setCGPoints(new Coordinate[]{new Coordinate(0.25,0,0,2),new Coordinate(0.25,0,0,2),new Coordinate(0.25,0,0,1),new Coordinate(0.25,0,0,1)})
            .setDigest("fc-synthetic-800-v1").build();
        MotorConfiguration config=new MotorConfiguration(body,id); config.setMotor(motor); config.setEjectionDelay(Motor.PLUGGED_DELAY); body.setMotorConfig(config,id);
        stage.setMassOverridden(true); stage.setOverrideMass(10); stage.setSubcomponentsOverriddenMass(true);
        stage.setCGOverridden(true); stage.setOverrideCGX(0.8); stage.setSubcomponentsOverriddenCG(true);
        rocket.enableEvents(); return rocket;
    }
    public static Simulation simulation(Rocket rocket) {
        Simulation s=new Simulation(rocket); s.setFlightConfigurationId(TestRockets.TEST_FCID_0);
        s.getOptions().setRandomSeed(20260920); s.getOptions().setWindSpeedAverage(0); s.getOptions().setWindSpeedDeviation(0); s.getOptions().setLaunchRodAngle(0); s.getOptions().setLaunchRodLength(1);
        s.getOptions().setLaunchAltitude(271); s.getOptions().setMaxSimulationTime(35); return s;
    }
}
