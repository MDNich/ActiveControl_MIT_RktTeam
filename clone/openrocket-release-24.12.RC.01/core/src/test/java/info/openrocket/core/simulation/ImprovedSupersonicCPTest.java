package info.openrocket.core.simulation;

import info.openrocket.core.aerodynamics.FlightConditions;
import info.openrocket.core.document.OpenRocketDocumentFactory;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.document.StorageOptions;
import info.openrocket.core.file.DocumentLoadingContext;
import info.openrocket.core.file.openrocket.OpenRocketSaver;
import info.openrocket.core.file.openrocket.importt.OpenRocketLoader;
import info.openrocket.core.logging.ErrorSet;
import info.openrocket.core.logging.WarningSet;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.util.BaseTestCase;
import org.junit.jupiter.api.Test;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;

import static org.junit.jupiter.api.Assertions.*;

class ImprovedSupersonicCPTest extends BaseTestCase {
	@Test
	void modelSelectionIsExclusiveAndSurvivesCopyAndSave() throws Exception {
		var doc = OpenRocketDocumentFactory.createNewRocket();
		var sim = new Simulation(doc, doc.getRocket());
		doc.addSimulation(sim);
		var options = sim.getOptions();
		var copy = options.clone();
		assertFalse(options.isImprovedSupersonicCP());
		assertFalse(options.isFiniteWingFins());
		options.setFiniteWingFins(true);
		var old = sim.duplicateSimulation(doc.getRocket());
		doc.addSimulation(old);
		options.setImprovedSupersonicCP(true);
		assertFalse(options.isFiniteWingFins());
		assertNotEquals(options, copy);
		copy.copyConditionsFrom(options);
		assertEquals(options, copy);
		assertTrue(copy.isImprovedSupersonicCP());
		var standard = sim.duplicateSimulation(doc.getRocket());
		standard.getOptions().setImprovedSupersonicCP(false);
		doc.addSimulation(standard);

		var out = new ByteArrayOutputStream();
		new OpenRocketSaver().save(out, doc, new StorageOptions(), new WarningSet(), new ErrorSet());
		var context = new DocumentLoadingContext();
		var reload = OpenRocketDocumentFactory.createEmptyRocket();
		context.setOpenRocketDocument(reload);
		new OpenRocketLoader().loadFromStream(context, new ByteArrayInputStream(out.toByteArray()), "cp.ork");
		assertTrue(reload.getSimulation(0).getOptions().isImprovedSupersonicCP());
		assertTrue(reload.getSimulation(1).getOptions().isFiniteWingFins());
		assertFalse(reload.getSimulation(1).getOptions().isImprovedSupersonicCP());
		assertFalse(reload.getSimulation(2).getOptions().isImprovedSupersonicCP());
		assertFalse(reload.getSimulation(2).getOptions().isFiniteWingFins());
		options.setFiniteWingFins(true);
		assertFalse(options.isImprovedSupersonicCP());
	}

	@Test
	void exposedBoattailUsesQualifiedLoadsAfterStagingWithSplitTubes() {
		final double inch = .0254;
		var rocket = new Rocket();
		var upper = new AxialStage();
		rocket.addChild(upper);
		var nose = new NoseCone(Transition.Shape.OGIVE, 12 * inch, 2 * inch);
		nose.setShapeParameter(1);
		nose.setAftRadiusAutomatic(false);
		upper.addChild(nose);
		for (int i = 0; i < 4; i++) {
			var tube = new BodyTube(11 * inch, 2 * inch, .0001);
			tube.setOuterRadiusAutomatic(false);
			upper.addChild(tube);
		}
		var tail = new Transition();
		tail.setShapeType(Transition.Shape.CONICAL);
		tail.setLength(4 * inch);
		tail.setForeRadius(2 * inch);
		tail.setAftRadius(1.5 * inch);
		upper.addChild(tail);
		var lower = new AxialStage();
		rocket.addChild(lower);
		lower.addChild(new BodyTube(12 * inch, 1.5 * inch, .0001));
		rocket.enableEvents();

		var cfg = rocket.getSelectedConfiguration();
		var options = new SimulationOptions();
		options.setImprovedSupersonicCP(true);
		var solver = options.toSimulationConditions().getAerodynamicCalculator().newInstance();
		var flight = new FlightConditions(cfg);
		flight.setAOA(Math.toRadians(2));
		flight.setMach(2.8);
		cfg.setAllStages();
		var stack = solver.getCP(cfg, flight, null);
		cfg.setOnlyStage(0);
		var cp = solver.getCP(cfg, flight, null);
		// Archived A-boattail reference, Mach 2.8, 2 degrees, 4-inch diameter.
		assertEquals(7.115199847616 * inch, cp.x, 1e-10);
		assertEquals(2.884959351737, cp.weight, 1e-10);
		cfg.setAllStages();
		assertEquals(stack.x, solver.getCP(cfg, flight, null).x, 1e-10);
		flight.setMach(.8);
		var standard = new SimulationOptions().toSimulationConditions().getAerodynamicCalculator();
		assertEquals(standard.getCP(cfg, flight, null), solver.getCP(cfg, flight, null));
	}
}
