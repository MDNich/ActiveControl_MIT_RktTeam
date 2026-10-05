package info.openrocket.core.simulation.ensemble;

import info.openrocket.core.document.*;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.exception.*;
import info.openrocket.core.simulation.listeners.*;
import info.openrocket.core.util.*;
import info.openrocket.core.logging.*;
import info.openrocket.core.file.*;
import info.openrocket.core.file.openrocket.OpenRocketSaver;
import info.openrocket.core.motor.ThrustCurveMotor;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import java.nio.file.*;
import java.util.*;
import static org.junit.jupiter.api.Assertions.*;
import static info.openrocket.core.simulation.FlightDataType.*;

class EnsembleTest extends BaseTestCase {
    @TempDir Path temp;
    static EnsembleSettings settings(int n,double motor,double atmosphere) { return new EnsembleSettings(true,n,motor,.05,atmosphere,.01*atmosphere,atmosphere,12345); }
    static FlightData data(double[] times,double[] altitude,double q) {
        var b=new FlightDataBranch("Stage",TYPE_TIME);
        for (int i=0;i<times.length;i++) {
            b.addPoint();b.setValue(TYPE_TIME,times[i]);b.setValue(TYPE_ALTITUDE,altitude[i]);
            b.setValue(TYPE_POSITION_X,2*times[i]);b.setValue(TYPE_POSITION_Y,times[i]);
            b.setValue(TYPE_VELOCITY_TOTAL,10);b.setValue(TYPE_ACCELERATION_TOTAL,2);b.setValue(TYPE_STABILITY,2);
            b.setValue(TYPE_ORIENTATION_QW,q);b.setValue(TYPE_ORIENTATION_QX,0);b.setValue(TYPE_ORIENTATION_QY,0);b.setValue(TYPE_ORIENTATION_QZ,0);
        }
        return new FlightData(b);
    }
    @Test void noiseHasSpecifiedVarianceBetweenKnotsAndIsOrderIndependent() {
        for (double t : new double[]{.5,.5125,.525,.5375}) {
            double mean=0,squares=0;
            for (int i=0;i<30000;i++) { double x=new GaussianThrustNoise(i,7,.05).value(t,19); mean+=x;squares+=x*x; }
            mean/=30000;assertEquals(0,mean,.15);assertEquals(49,squares/30000-mean*mean,1.5);
        }
        var noise=new GaussianThrustNoise(42,7,.05);double at=noise.value(.52,18);
        noise.value(9,18);noise.value(.01,18);assertEquals(at,noise.value(.52,18));
        assertNotEquals(at,noise.value(.54,18));assertNotEquals(at,noise.value(.52,19));
        assertEquals(0,new GaussianThrustNoise(4,0,.05).value(1,2));
    }
    @Test void resamplingUsesPhysicalTimeUnbiasedDeviationAndCommonSupport() {
        var a=new EnsembleAccumulator(settings(2,2,0));
        a.add(data(new double[]{0,.5,1,1.5,2},new double[]{0,5,10,5,0},1));
        a.add(data(new double[]{0,1,1.5},new double[]{0,20,10},-1));
        var result=a.finish();var mean=result.getBranch(0);var sd=result.getEnsembleResult().deviation(0);
        assertEquals(4,mean.getLength());assertEquals(15,mean.getByIndex(TYPE_ALTITUDE,2),1e-12);
        assertEquals(Math.sqrt(50),sd.getByIndex(TYPE_ALTITUDE,2),1e-12);
        assertEquals(1,mean.getLast(TYPE_ORIENTATION_QW),1e-12);
        assertArrayEquals(new double[]{10,20},result.getEnsembleResult().samples(0,EnsembleMetric.MAX_ALTITUDE));
        assertEquals(15,result.getMaxAltitude());assertEquals(1.75,result.getFlightTime());
    }
    @Test void scalarExtremaAreTakenBeforeAveragingAndMissingValuesStayMissing() {
        var a=new EnsembleAccumulator(settings(2,1,0));
        var first=data(new double[]{0,1,2},new double[]{0,10,0},1);
        first.getBranch(0).setValue(TYPE_STABILITY,Double.NaN);
        var second=data(new double[]{0,1,2},new double[]{0,0,20},1);
        a.add(first);a.add(second);var result=a.finish();
        assertEquals(15,result.getMaxAltitude());assertEquals(10,result.getBranch(0).getMaximum(TYPE_ALTITUDE));
        assertTrue(Double.isNaN(result.getBranch(0).getLast(TYPE_STABILITY)));
        assertTrue(Double.isNaN(result.getEnsembleResult().deviation(0).getLast(TYPE_STABILITY)));
    }
    @Test void circularMeansDoNotAverageAcrossWrongSideOfZeroAndEventsUseMeanTime() {
        var a=new EnsembleAccumulator(settings(2,0,0));
        for (double angle:new double[]{Math.toRadians(359),Math.toRadians(1)}) {
            var d=data(new double[]{0,1,2},new double[]{0,1,0},1);
            d.getBranch(0).setValue(TYPE_WIND_DIRECTION,angle);
            d.getBranch(0).addEvent(new FlightEvent(FlightEvent.Type.APOGEE,angle>1?.8:1.2));a.add(d);
        }
        var result=a.finish();assertEquals(0,result.getBranch(0).getLast(TYPE_WIND_DIRECTION),1e-12);
        assertEquals(1,result.getBranch(0).getFirstEvent(FlightEvent.Type.APOGEE).getTime(),1e-12);
        assertEquals(Math.toRadians(Math.sqrt(2)),result.getEnsembleResult().deviation(0).getLast(TYPE_WIND_DIRECTION),1e-5);
    }
    @Test void thrustNoiseIsAdditiveOnlyDuringBurnAndCannotCreateNegativeThrust() {
        var rocket=TestRockets.makeEstesAlphaIII();var sim=new Simulation(rocket);sim.setFlightConfigurationId(TestRockets.TEST_FCID_1);
        var status=new SimulationStatus(sim.getActiveConfiguration(),sim.getOptions().toSimulationConditions());
        var listener=new EnsembleRunListener(settings(2,.3,0),55,0,0);
        assertEquals(0,listener.postSimpleThrustCalculation(status,0));
        var motor=status.getActiveMotors().iterator().next();motor.ignite(.2);status.setSimulationTime(.5);
        double nominal=motor.getThrust(.5); long identity=((long)motor.getID().hashCode()<<32);
        double expected=Math.max(0,nominal+new GaussianThrustNoise(55,.3,.05).value(.3,(long)identity));
        assertEquals(expected,listener.postSimpleThrustCalculation(status,nominal),1e-12);
        motor.burnOut(.6);status.setSimulationTime(.7);assertEquals(0,listener.postSimpleThrustCalculation(status,0));
    }
    static Simulation simulation() {
        var r=TestRockets.makeEstesAlphaIII();var s=new Simulation(r);s.setFlightConfigurationId(TestRockets.TEST_FCID_1);
        s.getOptions().setISAAtmosphere(true);s.getOptions().setLaunchRodLength(1);s.getOptions().setMaxSimulationTime(30);
        s.getOptions().setTimeStep(.02);s.getOptions().getAverageWindModel().setAverage(2);s.getOptions().getAverageWindModel().setStandardDeviation(.2);
        return s;
    }
    @Test void completeFlightsRepeatWithSeedAndZeroNoiseHasZeroSpread() throws Exception {
        var s=simulation();s.getOptions().setEnsembleSettings(settings(3,0,0));s.simulate();
        var zero=s.getSimulatedData().getEnsembleResult();assertEquals(0,zero.deviation(0).getMaximum(TYPE_ALTITUDE),1e-12);
        s.getOptions().setEnsembleSettings(settings(3,.2,0));s.simulate();
        var noisy=s.getSimulatedData().getEnsembleResult();double[] values=noisy.samples(0,EnsembleMetric.MAX_ALTITUDE);
        assertTrue(noisy.deviation(0).getMaximum(TYPE_ALTITUDE)>0);
        s.simulate();assertArrayEquals(values,s.getSimulatedData().getEnsembleResult().samples(0,EnsembleMetric.MAX_ALTITUDE),1e-12);
        s.getOptions().setEnsembleSettings(settings(3,0,.3));s.simulate();assertTrue(s.getSimulatedData().getEnsembleResult().deviation(0).getMaximum(TYPE_ALTITUDE)>0);
    }
    @Test void cancellationAndFailedMemberPreservePriorResult() throws Exception {
        var s=simulation();s.getOptions().setEnsembleSettings(settings(2,0,0));s.simulate();var prior=s.getSimulatedData();
        assertThrows(SimulationCancelledException.class,()->s.simulate(new AbstractSimulationListener(){
            @Override public void postStep(SimulationStatus status) throws SimulationException {throw new SimulationCancelledException();}
        }));assertSame(prior,s.getSimulatedData());assertEquals(0,s.getEnsembleRunNumber());
        assertThrows(SimulationException.class,()->s.simulate(new AbstractSimulationListener(){
            @Override public void postStep(SimulationStatus status) throws SimulationException {throw new SimulationException("Test failure");}
        }));assertSame(prior,s.getSimulatedData());
    }
    @Test void saveReloadPreservesSettingsSamplesAndBands() throws Exception {
        var s=simulation();var settings=settings(2,.1,.1);s.getOptions().setEnsembleSettings(settings);s.simulate();
        var doc=OpenRocketDocumentFactory.createDocumentFromRocket(s.getRocket());doc.addSimulation(s);
        // The in-memory motor database avoids changing any installed user motor files.
        var db=new info.openrocket.core.database.motor.ThrustCurveMotorSetDatabase();
        for (var motor:s.getActiveConfiguration().getAllMotors()) db.addMotor((ThrustCurveMotor)motor.getMotor());
        var context=new DocumentLoadingContext();
        context.setOpenRocketDocument(OpenRocketDocumentFactory.createEmptyRocket());
        context.setAttachmentFactory(new FileSystemAttachmentFactory(temp.toFile()));
        context.setMotorFinder((type,manufacturer,designation,diameter,length,digest,warnings)->db.findMotors(digest,type,manufacturer,designation,diameter,length).stream().findFirst().orElse(null));
        Path file=temp.resolve("ensemble.ork");var storage=new StorageOptions();storage.setSaveSimulationData(true);
        new GeneralRocketSaver().save(file.toFile(),doc,storage);
        var loader=new info.openrocket.core.file.openrocket.importt.OpenRocketLoader();
        try(var zip=new java.util.zip.ZipFile(file.toFile());var in=zip.getInputStream(zip.getEntry("rocket.ork"))) {
            loader.loadFromStream(context,in,file.toString());
        }
        var loaded=context.getOpenRocketDocument().getSimulation(0);
        assertEquals(settings,loaded.getOptions().getEnsembleSettings());assertNotNull(loaded.getSimulatedData().getEnsembleResult(),loader.getWarnings().toString());
        assertArrayEquals(s.getSimulatedData().getEnsembleResult().samples(0,EnsembleMetric.MAX_ALTITUDE),loaded.getSimulatedData().getEnsembleResult().samples(0,EnsembleMetric.MAX_ALTITUDE),1e-10);
        assertEquals(s.getSimulatedData().getEnsembleResult().deviation(0).getLast(TYPE_ALTITUDE),loaded.getSimulatedData().getEnsembleResult().deviation(0).getLast(TYPE_ALTITUDE),1e-4);
        assertEquals(s.getSimulatedData().getMaxAltitude(),loaded.getSimulatedData().getMaxAltitude(),1e-10);
        var originalRuns=s.getSimulatedData().getEnsembleResult().individualRuns();
        var loadedRuns=loaded.getSimulatedData().getEnsembleResult().individualRuns();
        assertNotNull(originalRuns);assertNotNull(loadedRuns);assertEquals(settings.runs(),loadedRuns.size());
        for (int i=0;i<settings.runs();i++) {
            assertEquals(originalRuns.parameters(i),loadedRuns.parameters(i));
            assertFlightEquals(originalRuns.read(i,doc),loadedRuns.read(i,context.getOpenRocketDocument()));
        }
        // Saving again after reopening must retain the raw archive as well.
        Path second=temp.resolve("saved-again.ork");
        try(var out=Files.newOutputStream(second)){new OpenRocketSaver().save(out,context.getOpenRocketDocument(),storage,new WarningSet(),new ErrorSet());}
        String xml=Files.readString(second);
        assertEquals(settings.runs(),xml.split("<ensemblerun ",-1).length-1);
        assertTrue(new OpenRocketSaver().estimateFileSize(doc,storage)>originalRuns.compressedSize());
        // Older aggregate-only files still load; they cannot retroactively recover full runs.
        String legacy=xml.replaceAll("(?s)<ensemblerun .*?</ensemblerun>","");
        context.setOpenRocketDocument(OpenRocketDocumentFactory.createEmptyRocket());
        var legacyLoader=new info.openrocket.core.file.openrocket.importt.OpenRocketLoader();
        legacyLoader.loadFromStream(context,new java.io.ByteArrayInputStream(legacy.getBytes(java.nio.charset.StandardCharsets.UTF_8)),"legacy.ork");
        assertNotNull(context.getOpenRocketDocument().getSimulation(0).getSimulatedData().getEnsembleResult());
        assertNull(context.getOpenRocketDocument().getSimulation(0).getSimulatedData().getEnsembleResult().individualRuns());
        // Explicitly omitting simulation data also omits the full run archive.
        storage.setSaveSimulationData(false);
        var withoutData=new java.io.ByteArrayOutputStream();new OpenRocketSaver().save(withoutData,doc,storage,new WarningSet(),new ErrorSet());
        assertFalse(withoutData.toString(java.nio.charset.StandardCharsets.UTF_8).contains("<ensemblerun "));
    }
    private static void assertFlightEquals(FlightData expected, FlightData actual) {
        assertEquals(expected.getBranchCount(),actual.getBranchCount());
        assertEquals(expected.getMaxAltitude(),actual.getMaxAltitude());
        assertEquals(expected.getFlightTime(),actual.getFlightTime());
        assertEquals(expected.getWarningSet().size(),actual.getWarningSet().size());
        for (int b=0;b<expected.getBranchCount();b++) {
            var e=expected.getBranch(b);var a=actual.getBranch(b);
            assertEquals(e.getName(),a.getName());assertEquals(e.getLength(),a.getLength());
            assertArrayEquals(e.getTypes(),a.getTypes());
            for(var type:e.getTypes()) assertEquals(e.get(type),a.get(type),type.getName());
            assertEquals(e.getEvents().size(),a.getEvents().size());
            for(int j=0;j<e.getEvents().size();j++) {
                var event=e.getEvents().get(j);var stored=a.getEvents().get(j);
                assertEquals(event.getTime(),stored.getTime());assertEquals(event.getType(),stored.getType());
                assertEquals(event.getSource()==null?null:event.getSource().getID(),stored.getSource()==null?null:stored.getSource().getID());
            }
        }
    }
    @Test void archivedFlightsPreserveLateSamplesStagesAndWarningReferences() throws Exception {
        var doc=OpenRocketDocumentFactory.createNewRocket();
        var flight=data(new double[]{0,1,2,3.123456789},new double[]{0,10,5,0},1);
        var booster=flight.getBranch(0).clone();flight.addBranch(booster);
        var warning=Warning.fromString("Per-run warning");flight.getWarningSet().add(warning);
        flight.getBranch(0).addEvent(new FlightEvent(FlightEvent.Type.SIM_WARN,1.123456789,null,warning));
        flight.getBranch(0).addEvent(new FlightEvent(FlightEvent.Type.APOGEE,1.23456789,doc.getRocket().getChild(0)));
        var inputs=new EnsembleRunParameters(1,Long.MIN_VALUE,290.12345678,101315.987654321,-1.23456789,.99999999);
        try(var builder=new EnsembleRunArchive.Builder()) {
            builder.add(inputs,flight);var archive=builder.finish();
            var restored=archive.read(0,doc);assertFlightEquals(flight,restored);
            var storedWarning=restored.getBranch(0).getFirstEvent(FlightEvent.Type.SIM_WARN);
            assertEquals(warning.getID(),((Warning)storedWarning.getData()).getID());
            assertSame(restored.getWarningSet().findById(warning.getID()),storedWarning.getData());
            assertEquals(3.123456789,restored.getBranch(1).getLast(TYPE_TIME));
            assertEquals(inputs,archive.parameters(0));
            assertThrows(IndexOutOfBoundsException.class,()->archive.read(1,doc));
        }
    }
    @Test void fullRunArchiveDefaultsToSavingDataWithoutOverridingExplicitChoice() throws Exception {
        var r=TestRockets.makeEstesAlphaIII();var doc=OpenRocketDocumentFactory.createDocumentFromRocket(r);
        var sim=new Simulation(doc,r);sim.setFlightConfigurationId(TestRockets.TEST_FCID_1);
        sim.getOptions().setISAAtmosphere(true);sim.getOptions().setLaunchRodLength(1);sim.getOptions().setMaxSimulationTime(.1);
        sim.getOptions().setEnsembleSettings(settings(2,0,0));sim.simulate();
        assertTrue(doc.getDefaultStorageOptions().getSaveSimulationData());
        doc.getDefaultStorageOptions().setSaveSimulationData(false);doc.getDefaultStorageOptions().setExplicitlySet(true);
        sim.simulate();assertFalse(doc.getDefaultStorageOptions().getSaveSimulationData());
    }
    @Test void settingsValidateAndParticipateInCopyAndDirtyState() {
        var a=new SimulationOptions();var b=a.clone();assertEquals(a,b);b.setEnsembleSettings(settings(2,1,0));assertNotEquals(a,b);
        a.copyConditionsFrom(b);assertEquals(a,b);assertEquals(b.getEnsembleSettings(),a.clone().getEnsembleSettings());
        assertThrows(IllegalArgumentException.class,()->new EnsembleSettings(true,1,0,.05,0,0,0,1));
        assertThrows(IllegalArgumentException.class,()->new EnsembleSettings(true,2,Double.NaN,.05,0,0,0,1));
        assertThrows(IllegalArgumentException.class,()->new EnsembleSettings(true,2,0,0,0,0,0,1));
        var paths=new edu.mit.rocket_team.zephyrus.telemetry.FlightComputerOutputSettings(temp.resolve("t.csv").toString(),temp.resolve("OR.log").toString());
        assertNotEquals(paths.csvFile(),paths.forRun("test-1").csvFile());assertTrue(paths.forRun("test-1").csvFile().endsWith("t-test-1.csv"));
    }
}
