package edu.mit.rocket_team.zephyrus;

import edu.mit.rocket_team.zephyrus.FC.RTFC;
import edu.mit.rocket_team.zephyrus.telemetry.*;
import edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.Trace;
import info.openrocket.core.document.*;
import info.openrocket.core.file.*;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.extension.impl.*;
import info.openrocket.core.simulation.listeners.*;
import info.openrocket.core.util.*;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import java.nio.file.*;
import java.util.List;
import static org.junit.jupiter.api.Assertions.*;

class RTImprovementsTest extends BaseTestCase {
    @TempDir Path directory;
    private String previous;
    @BeforeEach void output() { previous=System.getProperty("openrocket.fc.telemetryDir"); System.setProperty("openrocket.fc.telemetryDir",directory.toString()); }
    @AfterEach void restore() { if(previous==null) System.clearProperty("openrocket.fc.telemetryDir"); else System.setProperty("openrocket.fc.telemetryDir",previous); }

    private RTFC packets(TelemetryLinkSettings settings, int endMs) {
        RTFC fc=new RTFC(new Trace(s -> {}),settings); fc.init(); fc.telemetry.open(directory);
        for(int ms=0;ms<=endMs;ms+=10) RTTestInputs.tick(fc,ms,9.8065f,100,3);
        return fc;
    }
    @Test void receiverExtremesDelayAndCancellation() throws Exception {
        RTFC zero=packets(TelemetryLinkSettings.DEFAULT,180); zero.telemetry.close();
        Path dir=zero.telemetry.getCsvPath().getParent();
        assertArrayEquals(Files.readAllBytes(dir.resolve("packets.bin")),Files.readAllBytes(dir.resolve("transmitted-packets.bin")));
        assertEquals(Files.readString(dir.resolve("telemetry.csv")),Files.readString(dir.resolve("transmitted-telemetry.csv")));
        RTFC lost=packets(new TelemetryLinkSettings(1,0,1),180); lost.telemetry.close();
        assertEquals(3,lost.telemetry.getDroppedCount()); assertEquals(0,Files.size(lost.telemetry.getPacketPath()));
        assertEquals(1,Files.readAllLines(lost.telemetry.getCsvPath()).size());
        RTFC delayed=packets(new TelemetryLinkSettings(0,137,1),180);
        assertEquals(3,delayed.telemetry.getPendingCount()); delayed.telemetry.close();
        assertEquals(3,delayed.telemetry.getReceivedCount());
        String first=Files.readAllLines(delayed.telemetry.getCsvPath()).get(1);
        assertTrue(first.startsWith("\"0.197\","), first);
        assertEquals(180000,delayed.trace.bootUs());
        RTFC cancelled=packets(new TelemetryLinkSettings(0,10000,1),180); cancelled.telemetry.finish(false);
        assertEquals(0,cancelled.telemetry.getReceivedCount()); assertEquals(3,cancelled.telemetry.getPendingCount());
        assertTrue(Files.readString(cancelled.telemetry.getCsvPath().getParent().resolve("metadata.txt")).contains("completion=incomplete"));
    }
    @Test void seededReceiverDoesNotAlterTransmittedData() throws Exception {
        RTFC a=packets(new TelemetryLinkSettings(.37,25,82),6000), b=packets(new TelemetryLinkSettings(.37,25,82),6000), c=packets(TelemetryLinkSettings.DEFAULT,6000);
        for(RTFC fc:List.of(a,b,c)) fc.telemetry.close();
        assertArrayEquals(Files.readAllBytes(a.telemetry.getPacketPath()),Files.readAllBytes(b.telemetry.getPacketPath()));
        assertArrayEquals(Files.readAllBytes(c.telemetry.getPacketPath()),Files.readAllBytes(a.telemetry.getCsvPath().getParent().resolve("transmitted-packets.bin")));
        assertTrue(a.telemetry.getReceivedCount()>0 && a.telemetry.getDroppedCount()>0);
        assertEquals(c.getState(),a.getState()); assertEquals(c.getLoopCount(),a.getLoopCount());
    }
    @Test void legacySettingsCloneCancelAndValidation() {
        Simulation simulation=RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());
        JavaCode legacy=new JavaCode(); legacy.setClassName(FlightControllerSimulatorListener.class.getName());
        JavaCode other=new JavaCode(); other.setClassName("example.OtherListener");
        simulation.getSimulationExtensions().add(legacy); simulation.getSimulationExtensions().add(other);
        Simulation snapshot=simulation.clone();
        assertTrue(ZephyrusFlightComputer.read(simulation).isEnabled());
        ZephyrusFlightComputer.apply(simulation,false,new TelemetryLinkSettings(.25,150,44));
        assertEquals(2,simulation.getSimulationExtensions().size());
        assertFalse(ZephyrusFlightComputer.read(simulation).isEnabled());
        Simulation copy=simulation.clone();
        ZephyrusFlightComputer.apply(copy,true,TelemetryLinkSettings.DEFAULT);
        assertEquals(150,ZephyrusFlightComputer.read(simulation).getLinkSettings().delayMs());
        simulation.loadFrom(snapshot);
        assertTrue(simulation.getSimulationExtensions().get(0) instanceof JavaCode);
        assertEquals("example.OtherListener",((JavaCode)simulation.getSimulationExtensions().get(1)).getClassName());
        assertThrows(IllegalArgumentException.class,()->new TelemetryLinkSettings(Double.NaN,0,1));
        assertThrows(IllegalArgumentException.class,()->new TelemetryLinkSettings(0,10001,1));
        Config invalid=new Config(); invalid.put("downlinkDelayMs",1.5);
        assertThrows(IllegalArgumentException.class,()->new ZephyrusFlightComputer().setConfig(invalid));
    }
    @Test void savedExtensionAndAttitudeRoundTripAndSettingsInvalidate() throws Exception {
        var rocket=RTFCVerificationRocket.rocket();
        OpenRocketDocument doc=OpenRocketDocumentFactory.createDocumentFromRocket(rocket);
        Simulation sim=RTFCVerificationRocket.simulation(rocket); doc.addSimulation(sim);
        sim.getOptions().setMaxSimulationTime(.3);
        var listener=new FlightControllerSimulatorListener(s->{},.0025,false);
        sim.simulate(listener);
        FlightDataBranch branch=sim.getSimulatedData().getBranch(0);
        assertNotNull(branch.get(FlightDataType.TYPE_ORIENTATION_QW));
        assertEquals(branch.getLength(),branch.get(FlightDataType.TYPE_VELOCITY_X).size());
        Simulation before=sim.clone();
        ZephyrusFlightComputer.apply(sim,false,new TelemetryLinkSettings(.2,95,7));
        assertEquals(Simulation.Status.OUTDATED,sim.getStatus());
        sim.loadFrom(before); assertEquals(before.getStatus(),sim.getStatus());
        ZephyrusFlightComputer.apply(sim,false,new TelemetryLinkSettings(.2,95,7));
        doc.getDefaultStorageOptions().setSaveSimulationData(true);
        Path path=directory.resolve("attitude.ork"); new GeneralRocketSaver().save(path.toFile(),doc);
        Simulation loaded=new GeneralRocketLoader(path.toFile()).load().getSimulation(0);
        assertFalse(ZephyrusFlightComputer.read(loaded).isEnabled());
        assertEquals(new TelemetryLinkSettings(.2,95,7),ZephyrusFlightComputer.read(loaded).getLinkSettings());
        var restored=loaded.getSimulatedData().getBranch(0);
        for(var type:List.of(FlightDataType.TYPE_ORIENTATION_QW,FlightDataType.TYPE_ORIENTATION_QX,FlightDataType.TYPE_ORIENTATION_QY,FlightDataType.TYPE_ORIENTATION_QZ,FlightDataType.TYPE_VELOCITY_X,FlightDataType.TYPE_VELOCITY_Y)) {
            assertEquals(branch.get(type).size(),restored.get(type).size());
            for(int i=0;i<branch.getLength();i++) assertEquals(branch.getByIndex(type,i),restored.getByIndex(type,i),1e-8);
        }
    }
    @Test void downlinkEffectsLeaveFullFlightAndTicksUnchanged() throws Exception {
        Simulation a=RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket()), b=RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());
        java.util.ArrayList<String> ticksA=new java.util.ArrayList<>(), ticksB=new java.util.ArrayList<>();
        var ca=new FlightControllerSimulatorListener(s->{if(s.contains("action=fc.loop_begin"))ticksA.add(s.substring(s.indexOf("boot_us=")));},.0025,false);
        var cb=new FlightControllerSimulatorListener(s->{if(s.contains("action=fc.loop_begin"))ticksB.add(s.substring(s.indexOf("boot_us=")));},.0025,false,new TelemetryLinkSettings(.3,137,19));
        a.simulate(ca); b.simulate(cb);
        assertEquals(ticksA,ticksB);
        assertEquals(a.getSimulatedData().getMaxAltitude(),b.getSimulatedData().getMaxAltitude(),1e-10);
        assertArrayEquals(Files.readAllBytes(ca.getFlightComputer().telemetry.getPacketPath()),Files.readAllBytes(cb.getFlightComputer().telemetry.getCsvPath().getParent().resolve("transmitted-packets.bin")));
    }
    @Test void duplicateListenersFailBeforeOpeningFiles() {
        var sim=RTFCVerificationRocket.simulation(RTFCVerificationRocket.rocket());
        ZephyrusFlightComputer.apply(sim,true,TelemetryLinkSettings.DEFAULT);
        assertThrows(Exception.class,()->sim.simulate(new FlightControllerSimulatorListener()));
        assertNull(sim.getFlightComputerTelemetryPath());
    }
}
