package info.openrocket.core.simulation.motorbatch;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import static org.junit.jupiter.api.Assertions.*;
import info.openrocket.core.document.*;
import info.openrocket.core.motor.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.rocketcomponent.position.AxialMethod;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.util.BaseTestCase;
import info.openrocket.core.util.TestRockets;
import edu.mit.rocket_team.zephyrus.telemetry.*;
import static info.openrocket.core.simulation.motorbatch.MotorBatchReport.*;

class MotorBatchTest extends BaseTestCase {
    @TempDir Path temp;
    static Simulation baseline() {
        Rocket rocket=TestRockets.makeEstesAlphaIII();
        for(var c:rocket)if(c instanceof InnerTube t && t.isMotorMount()) {
            // The historic numerical fixture has an undersized bore and an overlapping engine block.
            t.setOuterRadius(.0094);t.setThickness(.0003);
            for(var child:t.getChildren())if(child instanceof EngineBlock b)b.setAxialOffset(-b.getLength());
        }
        var doc=OpenRocketDocumentFactory.createDocumentFromRocket(rocket);
        var s=new Simulation(doc,rocket);doc.addSimulation(s);s.setFlightConfigurationId(TestRockets.TEST_FCID_1);
        s.setName("Batch test");s.getOptions().setRandomSeed(17);s.getOptions().setMaxSimulationTime(80);
        s.getOptions().setWindSpeedAverage(0);s.getOptions().setWindSpeedDeviation(0);return s;
    }
    static MotorMount mount(Simulation s){return s.getActiveConfiguration().getAllMotors().iterator().next().getMount();}
    static ThrustCurveMotor motor(Simulation s){return (ThrustCurveMotor)mount(s).getMotorConfig(s.getFlightConfigurationId()).getMotor();}
    static MotorBatchSettings settings(Simulation s,int threads,MotorBatchSettings.DelayPolicy delay) {
        return new MotorBatchSettings(((RocketComponent)mount(s)).getID(),threads,.018,.5,Set.of("Estes"),"","ALL","ALL",false,0,1e9,0,1e9,0,1e6,delay,Map.of());
    }
    static List<MotorCandidateSelector.Candidate> candidates(Simulation s,int n){
        var curves=new ArrayList<ThrustCurveMotor>();
        for(var it=mount(s).getMotorIterator();it.hasNext();)if(it.next().getMotor() instanceof ThrustCurveMotor m)curves.add(m);
        var rows=new ArrayList<MotorCandidateSelector.Candidate>();
        for(int i=0;i<n;i++)rows.add(new MotorCandidateSelector.Candidate("candidate-"+i,curves.get(i%curves.size()),""));return rows;
    }
    @Test void fourWorkersMatchOneAndNeverChangeTheBaseline() throws Exception {
        var s=baseline();var original=motor(s);var s1=settings(s,1,MotorBatchSettings.DelayPolicy.BEST_LISTED);var s4=settings(s,4,MotorBatchSettings.DelayPolicy.BEST_LISTED);
        var candidates=candidates(s,4);
        var serial=new MotorBatchRunner(MotorBatchSnapshot.capture(s,s1),s1,candidates).run(p->{});
        var parallel=new MotorBatchRunner(MotorBatchSnapshot.capture(s,s4),s4,candidates).run(p->{});
        assertEquals(4,parallel.peakThreads(),"Four flights must actually overlap");assertEquals(1,serial.peakThreads());
        assertSame(original,motor(s));assertNull(s.getSimulatedData());
        for(int i=0;i<serial.rows().size();i++) {
            var a=serial.rows().get(i);var b=parallel.rows().get(i);
            assertFalse(a.trials().isEmpty(),a.selected().reasons().toString());assertEquals(a.trials().size(),b.trials().size());
            for(int j=0;j<a.trials().size();j++) {
                assertEquals(a.trials().get(j).status(),b.trials().get(j).status());
                assertEquals(a.trials().get(j).metrics(),b.trials().get(j).metrics());
                assertEquals(a.trials().get(j).reasons(),b.trials().get(j).reasons());
            }
        }
    }
    @Test void cancelledDelaySearchCannotBeReportedAsAnOptimizedPass() throws Exception {
        var s=baseline();var settings=settings(s,4,MotorBatchSettings.DelayPolicy.BEST_LISTED);
        var runner=new MotorBatchRunner(MotorBatchSnapshot.capture(s,settings),settings,candidates(s,12));
        var result=runner.run(p->{if(p.completedTrials()>0)runner.cancel();});
        assertTrue(result.rows().stream().anyMatch(r->r.trials().size()<r.planned()));
        assertTrue(result.rows().stream().filter(r->r.trials().size()<r.planned()).noneMatch(r->r.selected().assessment()==Assessment.MEETS_CRITERIA));
        assertNull(s.getSimulatedData());
    }
    @Test void snapshotsCurvesAndCSVSurviveReopeningIncludingFailedRows() throws Exception {
        var s=baseline();var settings=settings(s,2,MotorBatchSettings.DelayPolicy.KEEP_BASELINE);
        var result=new MotorBatchRunner(MotorBatchSnapshot.capture(s,settings),settings,candidates(s,3)).run(p->{});
        String json=MotorBatchIO.write(result);var restored=MotorBatchIO.read(json);
        assertEquals(result.settings(),restored.settings());assertEquals(result.rows().size(),restored.rows().size());
        assertEquals(result.rows().get(0).selected(),restored.rows().get(0).selected());
        var doc=MotorBatchSnapshot.restore(restored);assertEquals(motor(s).getDigest(),motor(doc.getSimulation(0)).getDigest());
        doc.putMotorBatch(restored);var bytes=new ByteArrayOutputStream();var storage=new StorageOptions();storage.setSaveSimulationData(false);
        new info.openrocket.core.file.openrocket.OpenRocketSaver().save(bytes,doc,storage,new info.openrocket.core.logging.WarningSet(),new info.openrocket.core.logging.ErrorSet());
        var context=new info.openrocket.core.file.DocumentLoadingContext();context.setOpenRocketDocument(OpenRocketDocumentFactory.createEmptyRocket());
        context.setMotorFinder((type,mfr,name,diameter,length,digest,w)->motor(s));
        var loader=new info.openrocket.core.file.openrocket.importt.OpenRocketLoader();loader.loadFromStream(context,new ByteArrayInputStream(bytes.toByteArray()),"batch.ork");
        assertEquals(1,context.getOpenRocketDocument().getMotorBatches().size());
        assertEquals(restored.id(),context.getOpenRocketDocument().getMotorBatches().get(0).id());
        var jsonObject=jakarta.json.Json.createReader(new StringReader(json)).readObject();
        assertThrows(IllegalArgumentException.class,()->MotorBatchIO.read(jakarta.json.Json.createObjectBuilder(jsonObject)
                .add("input",jakarta.json.Json.createObjectBuilder(restored.input()).add("hash","damaged")).build().toString()));
        var damaged=bytes.toString(java.nio.charset.StandardCharsets.UTF_8).replaceFirst("(?s)<motorbatch>.*?</motorbatch>","<motorbatch>{}</motorbatch>");
        var brokenContext=new info.openrocket.core.file.DocumentLoadingContext();brokenContext.setOpenRocketDocument(OpenRocketDocumentFactory.createEmptyRocket());
        brokenContext.setMotorFinder((type,mfr,name,diameter,length,digest,w)->motor(s));
        var brokenLoader=new info.openrocket.core.file.openrocket.importt.OpenRocketLoader();
        brokenLoader.loadFromStream(brokenContext,new ByteArrayInputStream(damaged.getBytes(java.nio.charset.StandardCharsets.UTF_8)),"damaged.ork");
        assertEquals(1,brokenContext.getOpenRocketDocument().getSimulationCount());assertTrue(brokenContext.getOpenRocketDocument().getMotorBatches().isEmpty());
        assertTrue(brokenLoader.getWarnings().toString().contains("Motor batch"));
        var csv=temp.resolve("results.csv");MotorBatchCSV.write(csv,restored,MotorBatchCSV.Scope.ALL_CANDIDATES,Set.of());
        try(var reader=new com.opencsv.CSVReader(Files.newBufferedReader(csv))) {
            var records=reader.readAll();assertEquals(restored.rows().size()+1,records.size());
            for(var row:records)assertEquals(records.get(0).length,row.length);
            assertTrue(Arrays.asList(records.get(0)).contains("nar_tripoli_motor_level"));
        }
    }
    @Test void motorDoesNotPassThroughAnEngineBlockOrBulkheadEvenWithLargerLengthLimit() throws Exception {
        var s=baseline();var mount=(InnerTube)mount(s);var motor=motor(s);
        assertTrue(MotorClearance.conflicts(s.getActiveConfiguration(),mount,motor).isEmpty());
        var block=(EngineBlock)mount.getChild(0);block.setAxialOffset(.001);
        assertTrue(MotorClearance.conflicts(s.getActiveConfiguration(),mount,motor).stream().anyMatch(x->x.contains("Engine Block")));
        var settings=settings(s,4,MotorBatchSettings.DelayPolicy.KEEP_BASELINE);
        var result=new MotorBatchRunner(MotorBatchSnapshot.capture(s,settings),settings,List.of(new MotorCandidateSelector.Candidate("motor",motor,""))).run(p->{});
        assertEquals(Status.SCREENED_OUT,result.rows().get(0).selected().status());assertEquals(0,result.peakThreads());
        assertEquals(Assessment.DOES_NOT_MEET_CRITERIA,result.rows().get(0).selected().assessment());
        block.setAxialOffset(-block.getLength());
        var bulkhead=new Bulkhead();bulkhead.setName("Avionics bulkhead");bulkhead.setLength(.003);bulkhead.setOuterRadius(.01);mount.addChild(bulkhead);bulkhead.setAxialMethod(AxialMethod.TOP);bulkhead.setAxialOffset(.025);
        assertTrue(MotorClearance.conflicts(s.getActiveConfiguration(),mount,motor).stream().anyMatch(x->x.contains("Avionics bulkhead")));
    }
    @Test void hollowCenteringRingClearsMotorButInternalMassAndFinTabDoNot() {
        var s=baseline();var mount=(InnerTube)mount(s);var body=mount.getParent();
        var ring=new CenteringRing();ring.setOuterRadius(.011);ring.setInnerRadius(.0091);ring.setLength(.005);mount.addChild(ring);ring.setAxialMethod(AxialMethod.TOP);ring.setAxialOffset(.03);
        assertTrue(MotorClearance.conflicts(s.getActiveConfiguration(),mount,motor(s)).isEmpty());
        var mass=new MassComponent();mass.setName("Battery");mass.setLength(.01);mass.setRadius(.004);mount.addChild(mass);mass.setAxialOffset(.03);
        assertTrue(MotorClearance.conflicts(s.getActiveConfiguration(),mount,motor(s)).stream().anyMatch(x->x.contains("Battery")));
        mount.removeChild(mass);
        var fin=new TrapezoidFinSet(3,.06,.03,.02,.03);body.addChild(fin);fin.setName("Through-wall tabs");fin.setAxialMethod(AxialMethod.TOP);fin.setAxialOffset(mount.getPosition().x);
        fin.setTabHeight(.009);fin.setTabLength(.04);fin.setThickness(.003);
        assertTrue(MotorClearance.conflicts(s.getActiveConfiguration(),mount,motor(s)).stream().anyMatch(x->x.contains("Through-wall tabs")));
    }
    @Test void intersectionOverrideRunsMotorAndSurvivesSavedResultsAndCSV() throws Exception {
        var s=baseline();var mount=(InnerTube)mount(s);var motor=motor(s);
        s.getOptions().setMaxSimulationTime(300); // Include the full parachute descent for this B4 flight.
        ((EngineBlock)mount.getChild(0)).setAxialOffset(.001);
        var strict=settings(s,2,MotorBatchSettings.DelayPolicy.KEEP_BASELINE);
        assertFalse(strict.ignoreComponentIntersections());
        var settings=strict.withIgnoreComponentIntersections(true);
        var blocked=MotorCandidateSelector.select(List.of(motor),strict,s);
        assertTrue(blocked.get(0).exclusion().contains("MOTOR_INTERSECTS:"));
        var preview=MotorCandidateSelector.select(List.of(motor),settings,s).get(0);
        assertEquals("",preview.exclusion());assertFalse(preview.ignoredIntersections().isEmpty());
        // Even a preview made with the previous setting is re-screened using the captured inputs.
        var result=new MotorBatchRunner(MotorBatchSnapshot.capture(s,settings),settings,blocked).run(p->{});
        var row=result.rows().get(0);
        assertEquals(1,row.trials().size());assertEquals(Status.COMPLETE,row.selected().status());
        assertEquals(Assessment.MEETS_CRITERIA,row.selected().assessment());
        assertEquals(preview.ignoredIntersections(),row.candidate().ignoredIntersections());
        assertNull(s.getSimulatedData());assertSame(motor,motor(s));

        var restored=MotorBatchIO.read(MotorBatchIO.write(result));
        assertTrue(restored.settings().ignoreComponentIntersections());
        assertEquals(row.candidate().ignoredIntersections(),restored.rows().get(0).candidate().ignoredIntersections());
        var doc=MotorBatchSnapshot.restore(restored);doc.putMotorBatch(restored);
        var bytes=new ByteArrayOutputStream();var storage=new StorageOptions();storage.setSaveSimulationData(false);
        new info.openrocket.core.file.openrocket.OpenRocketSaver().save(bytes,doc,storage,new info.openrocket.core.logging.WarningSet(),new info.openrocket.core.logging.ErrorSet());
        var context=new info.openrocket.core.file.DocumentLoadingContext();context.setOpenRocketDocument(OpenRocketDocumentFactory.createEmptyRocket());
        context.setMotorFinder((type,mfr,name,diameter,length,digest,w)->motor);
        new info.openrocket.core.file.openrocket.importt.OpenRocketLoader().loadFromStream(context,new ByteArrayInputStream(bytes.toByteArray()),"override.ork");
        var saved=context.getOpenRocketDocument().getMotorBatches().get(0);
        assertTrue(saved.settings().ignoreComponentIntersections());
        assertEquals(row.candidate().ignoredIntersections(),saved.rows().get(0).candidate().ignoredIntersections());
        for(var scope:List.of(MotorBatchCSV.Scope.ALL_CANDIDATES,MotorBatchCSV.Scope.MEETS_CRITERIA,MotorBatchCSV.Scope.ALL_DELAY_TRIALS)) {
            var csv=new StringWriter();MotorBatchCSV.write(csv,saved,scope,Set.of());
            try(var reader=new com.opencsv.CSVReader(new StringReader(csv.toString()))) {
                var records=reader.readAll();assertEquals(2,records.size());var headers=Arrays.asList(records.get(0));
                assertEquals("true",records.get(1)[headers.indexOf("component_intersection_override")]);
                assertEquals(String.join("; ",preview.ignoredIntersections()),records.get(1)[headers.indexOf("ignored_component_intersections")]);
            }
        }
        var json=jakarta.json.Json.createReader(new StringReader(MotorBatchIO.write(result))).readObject();
        var oldSettings=jakarta.json.Json.createObjectBuilder(json.getJsonObject("settings")).remove("ignoreComponentIntersections");
        var oldRows=jakarta.json.Json.createArrayBuilder();
        json.getJsonArray("rows").getValuesAs(jakarta.json.JsonObject.class).forEach(r->oldRows.add(jakarta.json.Json.createObjectBuilder(r).remove("ignoredIntersections")));
        var legacy=MotorBatchIO.read(jakarta.json.Json.createObjectBuilder(json).add("settings",oldSettings).add("rows",oldRows).build().toString());
        assertFalse(legacy.settings().ignoreComponentIntersections());assertTrue(legacy.rows().get(0).candidate().ignoredIntersections().isEmpty());
        var strictAgain=new MotorBatchRunner(MotorBatchSnapshot.capture(s,strict),strict,List.of(preview));
        assertEquals(0,strictAgain.totalTrials());assertEquals(Status.SCREENED_OUT,strictAgain.report().rows().get(0).selected().status());
    }
    @Test void intersectionOverrideKeepsSizeDelayAndFlightCriteriaChecks() throws Exception {
        var s=baseline();var mount=(InnerTube)mount(s);var motor=motor(s);
        s.getOptions().setMaxSimulationTime(300);
        ((EngineBlock)mount.getChild(0)).setAxialOffset(.001);
        var settings=settings(s,1,MotorBatchSettings.DelayPolicy.KEEP_BASELINE).withIgnoreComponentIntersections(true);
        var tooShort=MotorBatchIO.settings(jakarta.json.Json.createObjectBuilder(MotorBatchIO.settings(settings)).add("length",.01).build());
        var blocked=new MotorBatchRunner(MotorBatchSnapshot.capture(s,tooShort),tooShort,List.of(new MotorCandidateSelector.Candidate("motor",motor,"")));
        assertEquals(0,blocked.totalTrials());assertTrue(blocked.report().rows().get(0).selected().reasons().toString().contains("MOTOR_TOO_LONG"));
        mount.setThickness(.002);
        assertTrue(MotorCandidateSelector.select(List.of(motor),settings,s).get(0).exclusion().contains("DIAMETER_TOO_LARGE"));
        mount.setThickness(.0003);
        var wrongDelay=MotorBatchIO.motor(jakarta.json.Json.createObjectBuilder(MotorBatchIO.motor(motor)).add("delays",jakarta.json.Json.createArrayBuilder().add(99)).build());
        var delayBlocked=new MotorBatchRunner(MotorBatchSnapshot.capture(s,settings),settings,List.of(new MotorCandidateSelector.Candidate("motor",wrongDelay,"")));
        assertEquals(0,delayBlocked.totalTrials());assertEquals(List.of("BASELINE_DELAY_NOT_LISTED"),delayBlocked.report().rows().get(0).selected().reasons());
        var limits=jakarta.json.Json.createObjectBuilder().add("APOGEE",jakarta.json.Json.createArrayBuilder().add(1e9).add(jakarta.json.JsonValue.NULL));
        var demanding=MotorBatchIO.settings(jakarta.json.Json.createObjectBuilder(MotorBatchIO.settings(settings)).add("criteria",limits).build());
        var result=new MotorBatchRunner(MotorBatchSnapshot.capture(s,demanding),demanding,List.of(new MotorCandidateSelector.Candidate("motor",motor,""))).run(p->{});
        assertEquals(Status.COMPLETE,result.rows().get(0).selected().status());
        assertEquals(Assessment.DOES_NOT_MEET_CRITERIA,result.rows().get(0).selected().assessment());
        assertTrue(result.rows().get(0).selected().reasons().toString().contains("APOGEE"));
    }
    @Test void fcFilesAreUniqueAndStateIsRepeatableUnderParallelExecution() throws Exception {
        var rocket=edu.mit.rocket_team.zephyrus.RTFCVerificationRocket.rocket();
        var s=edu.mit.rocket_team.zephyrus.RTFCVerificationRocket.simulation(rocket);s.getOptions().setMaxSimulationTime(.5);
        ZephyrusFlightComputer.apply(s,true,TelemetryLinkSettings.DEFAULT,new FlightComputerOutputSettings(temp.resolve("telem.csv").toString(),temp.resolve("OR.log").toString()));
        var mount=mount(s);var motor=motor(s);
        var settings=new MotorBatchSettings(((RocketComponent)mount).getID(),4,motor.getDiameter(),motor.getLength(),Set.of(),"","ALL","ALL",false,0,1e9,0,1e9,0,1e6,MotorBatchSettings.DelayPolicy.KEEP_BASELINE,Map.of());
        var candidates=new ArrayList<MotorCandidateSelector.Candidate>();for(int i=0;i<4;i++)candidates.add(new MotorCandidateSelector.Candidate("fc-"+i,motor,""));
        var result=new MotorBatchRunner(MotorBatchSnapshot.capture(s,settings),settings,candidates).run(p->{});
        assertEquals(4,result.peakThreads());var paths=new HashSet<String>();
        for(var row:result.rows()){assertEquals(1,row.trials().size(),row.selected().reasons().toString());var trial=row.trials().get(0);assertNotEquals(Status.ERROR,trial.status(),trial.error());assertTrue(paths.add(trial.telemetryPath()));assertTrue(Files.isRegularFile(Path.of(trial.telemetryPath())));assertTrue(Files.isRegularFile(Path.of(trial.logPath())));}
        for(var row:result.rows())assertEquals(result.rows().get(0).trials().get(0).metrics(),row.trials().get(0).metrics());
        assertFalse(Files.exists(temp.resolve("telem.csv")));assertNull(s.getSimulatedData());
    }
    @Test void certificationBoundariesAndDesignationsAreExplicit() {
        assertEquals("G",MotorCertification.impulseClass(160));assertEquals("H",MotorCertification.impulseClass(Math.nextUp(160.0)));
        assertEquals("I",MotorCertification.impulseClass(640));assertEquals("J",MotorCertification.impulseClass(Math.nextUp(640.0)));
        assertEquals("L",MotorCertification.impulseClass(5120));assertEquals("M",MotorCertification.impulseClass(Math.nextUp(5120.0)));
        assertEquals("O",MotorCertification.impulseClass(40960));assertEquals("Above O",MotorCertification.impulseClass(40961));
        var json=MotorBatchIO.motor(motor(baseline()));
        for(var item:Map.of("H128",MotorCertification.Level.L1,"I200",MotorCertification.Level.L1,"J350",MotorCertification.Level.L2,"L1000",MotorCertification.Level.L2,"M1350",MotorCertification.Level.L3,"O8000",MotorCertification.Level.L3).entrySet()) {
            var m=MotorBatchIO.motor(jakarta.json.Json.createObjectBuilder(json).add("designation",item.getKey()).build());
            assertEquals(item.getValue(),MotorCertification.classify(m).level());
            assertEquals("Manufacturer designation",MotorCertification.classify(m).basis());
        }
    }
    @Test void missingOptionalCatalogueMetadataCanBeSaved() {
        var m=motor(baseline());
        var sparse=new ThrustCurveMotor.Builder().setManufacturer(m.getManufacturer()).setDesignation("1/2A3")
                .setCaseInfo(null).setPropellantInfo(null)
                .setMotorType(m.getMotorType()).setDiameter(m.getDiameter()).setLength(m.getLength())
                .setTimePoints(m.getTimePoints()).setThrustPoints(m.getThrustPoints()).setCGPoints(m.getCGPoints())
                .setStandardDelays(m.getStandardDelays()).build();
        var restored=MotorBatchIO.motor(MotorBatchIO.motor(sparse));
        assertArrayEquals(sparse.getTimePoints(),restored.getTimePoints());
        assertArrayEquals(sparse.getThrustPoints(),restored.getThrustPoints());
        assertEquals("1/2A",MotorCertification.classify(restored).impulseClass());
    }
    @Test void longerMotorExtendsForwardIntoBulkheadAndRadialMassCanClear() {
        var s=baseline();var mount=(InnerTube)mount(s);var body=mount.getParent();
        mount.removeChild(mount.getChild(0));
        var bulkhead=new Bulkhead();bulkhead.setName("Forward bulkhead");bulkhead.setLength(.003);bulkhead.setOuterRadius(.012);
        body.addChild(bulkhead);bulkhead.setAxialMethod(AxialMethod.TOP);bulkhead.setAxialOffset(mount.getPosition().x-.015);
        assertTrue(MotorClearance.conflicts(s.getActiveConfiguration(),mount,motor(s)).isEmpty());
        var longer=MotorBatchIO.motor(jakarta.json.Json.createObjectBuilder(MotorBatchIO.motor(motor(s))).add("length",.12).build());
        assertTrue(MotorClearance.conflicts(s.getActiveConfiguration(),mount,longer).stream().anyMatch(x->x.contains("Forward bulkhead")));
        body.removeChild(bulkhead);
        var mass=new MassComponent();mass.setName("Offset battery");mass.setLength(.01);mass.setRadius(.001);
        body.addChild(mass);mass.setAxialMethod(AxialMethod.TOP);mass.setAxialOffset(mount.getPosition().x+.02);mass.setRadialPosition(.011);
        assertTrue(MotorClearance.conflicts(s.getActiveConfiguration(),mount,motor(s)).isEmpty());
        mass.setRadialPosition(.008);
        assertTrue(MotorClearance.conflicts(s.getActiveConfiguration(),mount,motor(s)).stream().anyMatch(x->x.contains("Offset battery")));
    }
    @Test void exteriorRailButtonDoesNotOccupyTheMotorBore() {
        var s=baseline();var mount=(InnerTube)mount(s);var body=mount.getParent();
        var button=new RailButton();body.addChild(button);button.setAxialMethod(AxialMethod.TOP);button.setAxialOffset(mount.getPosition().x+.02);
        for(double angle:new double[]{0,Math.PI/4,Math.PI/2,Math.PI,-Math.PI/2}) {
            button.setAngleOffset(angle);
            assertTrue(MotorClearance.conflicts(s.getActiveConfiguration(),mount,motor(s)).isEmpty(),"Button angle "+angle);
        }
    }
    @Test void ordinarySimulationsMatchBatchForThreeDifferentMotors() throws Exception {
        var s=baseline();var settings=settings(s,4,MotorBatchSettings.DelayPolicy.BEST_LISTED);
        var result=new MotorBatchRunner(MotorBatchSnapshot.capture(s,settings),settings,candidates(s,3)).run(p->{});
        for(var row:result.rows()) {
            var ordinary=baseline();var mount=mount(ordinary);var config=new MotorConfiguration(mount,ordinary.getFlightConfigurationId(),mount.getMotorConfig(ordinary.getFlightConfigurationId()));
            config.setMotor(row.candidate().motor());config.setEjectionDelay(row.selected().delay());
            mount.setMotorConfig(config,ordinary.getFlightConfigurationId());
            ordinary.simulate();
            var expected=MotorBatchMetrics.evaluate(ordinary,settings);
            for(var metric:expected.values().keySet())assertEquals(expected.values().get(metric),row.selected().metrics().get(metric),1e-7,
                    row.candidate().motor().getDesignation()+" "+metric);
        }
        var shortlist=MotorBatchSnapshot.createSimulations(MotorBatchIO.read(MotorBatchIO.write(result)),result.rows());
        assertEquals(3,shortlist.getSimulationCount());
        assertEquals(3,shortlist.getSimulations().stream().map(Simulation::getFlightConfigurationId).distinct().count());
        for(int i=0;i<shortlist.getSimulationCount();i++) {
            var simulation=shortlist.getSimulation(i);assertSame(shortlist.getRocket(),simulation.getRocket());
            assertEquals(result.rows().get(i).candidate().motor().getDigest(),motor(simulation).getDigest());
            simulation.simulate();
            double apogeeDelta=simulation.getSimulatedData().getMaxAltitude()-result.rows().get(i).selected().metrics().get(MotorBatchMetrics.Metric.APOGEE);
            System.out.println("MOTOR_BATCH_VERIFY saved_rerun motor="+motor(simulation).getDesignation()+" apogee_delta_m="+apogeeDelta);
            assertEquals(0,apogeeDelta,1e-3,"Saved OR XML geometry uses decimal rounding");
        }
    }
    @Test void ascentExtremaExcludeDescentAndInterpolateEventBoundaries() {
        var b=new FlightDataBranch("synthetic",FlightDataType.TYPE_TIME,FlightDataType.TYPE_STABILITY);
        double[] stability={1,2,3,-10};
        for(int i=0;i<stability.length;i++){b.addPoint();b.setValue(FlightDataType.TYPE_TIME,i);b.setValue(FlightDataType.TYPE_STABILITY,stability[i]);}
        assertEquals(1.5,MotorBatchMetrics.extreme(b,FlightDataType.TYPE_STABILITY,.5,2,true));
        assertEquals(-10,MotorBatchMetrics.extreme(b,FlightDataType.TYPE_STABILITY,0,3,true));
        assertTrue(Double.isNaN(MotorBatchMetrics.extreme(b,FlightDataType.TYPE_STABILITY,Double.NaN,2,true)));
    }
    @Test void csvPreservesUnicodeQuotedTextMissingValuesAndNumericSigns() throws Exception {
        var s=baseline();var settings=settings(s,1,MotorBatchSettings.DelayPolicy.KEEP_BASELINE);
        var trial=new Trial(1,3,Status.INCOMPLETE,Assessment.DOES_NOT_MEET_CRITERIA,List.of("@review, \"quoted\"\nΔ"),
                Map.of(MotorBatchMetrics.Metric.MIN_STABILITY,-1.5),"","","","",Double.NaN,Double.NaN);
        var row=new Row(candidates(s,1).get(0),List.of(trial),1,trial);
        var report=new MotorBatchReport(UUID.randomUUID().toString(),java.time.Instant.now().toString(),settings,MotorBatchSnapshot.capture(s,settings).input(),List.of(row),1);
        var writer=new StringWriter();var previous=Locale.getDefault();
        try {Locale.setDefault(Locale.GERMANY);MotorBatchCSV.write(writer,report,MotorBatchCSV.Scope.ALL_CANDIDATES,Set.of());}
        finally {Locale.setDefault(previous);}
        try(var reader=new com.opencsv.CSVReader(new StringReader(writer.toString()))) {
            var records=reader.readAll();assertEquals(2,records.size());var headers=Arrays.asList(records.get(0));
            assertEquals("'@review, \"quoted\"\nΔ",records.get(1)[headers.indexOf("reasons")]);
            assertEquals("-1.5",records.get(1)[headers.indexOf("min_ascent_stability_cal")]);
            assertEquals("",records.get(1)[headers.indexOf("apogee_agl_m")]);
        }
    }
}
