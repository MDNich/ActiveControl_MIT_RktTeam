package edu.mit.rocket_team.zephyrus;

import java.nio.file.*;
import java.util.*;
import java.util.function.Consumer;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import info.openrocket.core.util.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.rocketcomponent.position.AxialMethod;
import info.openrocket.core.motor.*;
import info.openrocket.core.document.*;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.listeners.*;
import edu.mit.rocket_team.zephyrus.FC.RTFC;
import static org.junit.jupiter.api.Assertions.*;

class RTFCSimulationTest extends BaseTestCase {
    @TempDir Path dir;
    private String previous;
    @BeforeEach void outputDirectory() { previous=System.getProperty("openrocket.fc.telemetryDir"); System.setProperty("openrocket.fc.telemetryDir",dir.toString()); }
    @AfterEach void restore() { if(previous==null) System.clearProperty("openrocket.fc.telemetryDir"); else System.setProperty("openrocket.fc.telemetryDir",previous); }
    static Rocket rocket() { return RTFCVerificationRocket.rocket(); }
    static Simulation simulation(Rocket rocket) { return RTFCVerificationRocket.simulation(rocket); }
    private record Result(Simulation simulation,FlightControllerSimulatorListener listener,List<Long> ticks) {}
    private Result run(double step,boolean closed) throws Exception {
        List<Long> ticks=new java.util.ArrayList<>();
        Consumer<String> log=s->{ if(s.contains("action=fc.loop_begin")) ticks.add(Long.parseLong(s.split("boot_us=")[1].split(" ")[0])); };
        FlightControllerSimulatorListener fc=new FlightControllerSimulatorListener(log,step,closed);
        Simulation sim=simulation(rocket()); sim.simulate(fc); return new Result(sim,fc,ticks);
    }
    @Test void closedLoopChangesTrajectoryAndSurvivesLandingStepperCopies() throws Exception {
        Result active=run(0.0025,false),closed=run(0.0025,true);
        assertEquals(edu.mit.rocket_team.zephyrus.util.RTRocketState.APOGEE,active.listener.getFlightComputer().getState());
        assertTrue(active.listener.getFlightComputer().getAirbrakeUpdateCount()>1000);
        assertTrue(active.simulation.getSimulatedData().getMaxAltitude()<closed.simulation.getSimulatedData().getMaxAltitude()-0.01,
            "Active="+active.simulation.getSimulatedData().getMaxAltitude()+" closed="+closed.simulation.getSimulatedData().getMaxAltitude());
        assertEquals(0,active.listener.getFlightComputer().getOutput().exposedFraction());
        assertTrue(Files.size(active.listener.getFlightComputer().telemetry.getCsvPath())>1000);
    }
    @Test void tickScheduleIndependentOfStepAndRepeatedRuns() throws Exception {
        Result a=run(0.0025,false),b=run(0.001,false),repeat=run(0.0025,false);
        assertEquals(a.ticks,b.ticks); assertEquals(a.ticks,repeat.ticks);
        assertEquals(a.simulation.getSimulatedData().getMaxAltitude(),repeat.simulation.getSimulatedData().getMaxAltitude(),1e-8);
        assertEquals(a.simulation.getSimulatedData().getMaxAltitude(),b.simulation.getSimulatedData().getMaxAltitude(),2.0);
    }
    @Test void rk4AlsoRespectsFirmwareDeadlines() throws Exception {
        boolean original=NewControlStepListener.useRK6;
        try {
            NewControlStepListener.useRK6=true; Result rk6=run(0.0025,false);
            NewControlStepListener.useRK6=false; Result rk4=run(0.0025,false);
            assertEquals(rk6.ticks,rk4.ticks);
            assertEquals(rk6.simulation.getSimulatedData().getMaxAltitude(),rk4.simulation.getSimulatedData().getMaxAltitude(),2.0);
        } finally { NewControlStepListener.useRK6=original; }
    }
    @Test void zeroLaunchRodAndOffGridIgnitionMakePositiveProgress() throws Exception {
        Simulation sim=simulation(rocket()); sim.getOptions().setLaunchRodLength(0); sim.getOptions().setMaxSimulationTime(1);
        for(RocketComponent c:sim.getRocket()) if(c instanceof BodyTube body && body.isMotorMount())
            body.getMotorConfig(sim.getFlightConfigurationId()).setIgnitionDelay(0.1237);
        var fc=new FlightControllerSimulatorListener(s->{},0.0025,false); sim.simulate(fc);
        assertTrue(fc.getFlightComputer().getLoopCount()>=200);
        assertEquals(edu.mit.rocket_team.zephyrus.util.RTRocketState.FLIGHT,fc.getFlightComputer().getState());
    }
    @Test void componentMissingIsActionableAndAirbrakeDragIsMonotonic() throws Exception {
        Rocket r=rocket(); AirbrakeSet b=null;
        for(RocketComponent c:r) if(c instanceof AirbrakeSet a) b=a;
        assertNotNull(b);
        info.openrocket.core.aerodynamics.FlightConditions cond=new info.openrocket.core.aerodynamics.FlightConditions(r.getSelectedConfiguration());
        cond.setAOA(0); cond.setMach(0.5);
        var calc=new info.openrocket.core.aerodynamics.barrowman.AirbrakeSetCalc(b);
        b.setFracExposed(0); double zero=calc.calculatePressureCD(cond,0,0,new info.openrocket.core.logging.WarningSet());
        b.setFracExposed(0.5); double half=calc.calculatePressureCD(cond,0,0,new info.openrocket.core.logging.WarningSet());
        b.setFracExposed(1); double full=calc.calculatePressureCD(cond,0,0,new info.openrocket.core.logging.WarningSet());
        assertTrue(full>half && half>zero);
        b.getParent().removeChild(b);
        Exception e=assertThrows(Exception.class,()->simulation(r).simulate(new FlightControllerSimulatorListener(s->{},0.0025,false)));
        assertTrue(e.getMessage().contains("AirbrakeSet"));
    }
}
