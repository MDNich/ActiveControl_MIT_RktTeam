package info.openrocket.core.aerodynamics.barrowman;

import info.openrocket.core.aerodynamics.FlightConditions;
import info.openrocket.core.rocketcomponent.AxialStage;
import info.openrocket.core.rocketcomponent.BodyTube;
import info.openrocket.core.rocketcomponent.FlightConfiguration;
import info.openrocket.core.rocketcomponent.NoseCone;
import info.openrocket.core.rocketcomponent.ParallelStage;
import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.rocketcomponent.SymmetricComponent;
import info.openrocket.core.rocketcomponent.Transition;
import info.openrocket.core.util.Coordinate;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Geometry and small-angle potential loading for the active main airframe. Lengths are in metres. */
public interface SupersonicBody {
	Coordinate potential(FlightConditions conditions, Coordinate baseline);

	static Map<RocketComponent, SupersonicBody> forConfiguration(FlightConfiguration configuration) {
		List<SymmetricComponent> spine = new ArrayList<>();
		for (AxialStage stage : configuration.getActiveStages()) {
			if (stage instanceof ParallelStage || !(stage.getParent() instanceof Rocket)) continue;
			for (RocketComponent child : stage.getChildren()) {
				if (child instanceof SymmetricComponent part && part.getLength() > 0) spine.add(part);
			}
		}
		spine.sort(Comparator.comparingDouble(SupersonicBody::station));
		Map<RocketComponent, SupersonicBody> models = new HashMap<>();
		for (int i = 0; i < spine.size(); i++) {
			SymmetricComponent part = spine.get(i);
			if (part instanceof NoseCone nose && supports(nose)) {
				double cylinderLength = 0;
				for (int j = i + 1; j < spine.size(); j++) {
					SymmetricComponent tube = spine.get(j);
					if (!(tube instanceof BodyTube) || tube.getParent() != nose.getParent()
							|| Math.abs(tube.getForeRadius() - nose.getAftRadius()) > 1e-6
							|| !touches(spine.get(j - 1), tube)) break;
					cylinderLength += tube.getLength();
				}
				if (cylinderLength > 0) {
					models.put(nose, new NoseCylinder(nose.getLength(), 2 * nose.getAftRadius(),
							Math.min(5, cylinderLength / nose.getLength()), nose.getShapeType() == Transition.Shape.CONICAL));
				}
			} else if (i == spine.size() - 1 && i > 0 && part instanceof Transition taper
					&& !(part instanceof NoseCone) && taper.getShapeType() == Transition.Shape.CONICAL
					&& taper.getForeRadius() > taper.getAftRadius() && taper.getAftRadius() > 0
					&& spine.get(i - 1) instanceof BodyTube tube && touches(tube, taper)
					&& Math.abs(tube.getAftRadius() - taper.getForeRadius()) < 1e-6) {
				models.put(taper, new Boattail(taper.getLength(), 2 * taper.getForeRadius(), 2 * taper.getAftRadius()));
			}
		}
		return models;
	}

	private static boolean supports(NoseCone nose) {
		if (nose.getForeRadius() >= 1e-8 || nose.getAftRadius() <= 0) return false;
		return switch (nose.getShapeType()) {
			case CONICAL -> true;
			case OGIVE -> Math.abs(nose.getShapeParameter() - 1) < 1e-7;
			case HAACK -> Math.abs(nose.getShapeParameter()) < 1e-7;
			default -> false;
		};
	}

	private static double station(SymmetricComponent part) {
		return part.getComponentLocations()[0].x;
	}

	private static boolean touches(SymmetricComponent fore, SymmetricComponent aft) {
		return Math.abs(station(fore) + fore.getLength() - station(aft)) < 1e-6;
	}

	private static Coordinate blend(FlightConditions conditions, Coordinate baseline, double weight,
			double diameter, double slope, double localCP) {
		double referenceArea = Math.PI * diameter * diameter / 4;
		double chart = weight * slope * referenceArea * conditions.getSincAOA() / conditions.getRefArea();
		double retained = (1 - weight) * baseline.weight;
		double total = retained + chart;
		// The force and first moment are additive, including the negative boattail increment.
		return total == 0 ? baseline : new Coordinate((retained * baseline.x + chart * localCP) / total, 0, 0, total);
	}

	record NoseCylinder(double length, double diameter, double cylinderRatio, boolean conical) implements SupersonicBody {
		@Override public Coordinate potential(FlightConditions conditions, Coordinate baseline) {
			double weight = .40 * DatcomCharts.envelope(conditions);
			if (weight == 0) return baseline;
			double u = conditions.getBeta() * diameter / length;
			DatcomTable normal = conical ? DatcomCharts.CONE_FORCE : DatcomCharts.OGIVE_FORCE;
			DatcomTable left = conical ? DatcomCharts.CONE_CP_LEFT : DatcomCharts.OGIVE_CP_LEFT;
			DatcomTable right = conical ? DatcomCharts.CONE_CP_RIGHT : DatcomCharts.OGIVE_CP_RIGHT;
			double fraction = (u <= 1 ? left : right).at(Math.min(u, 1 / u), cylinderRatio);
			return blend(conditions, baseline, weight, diameter, normal.at(u, cylinderRatio),
					fraction * length * (1 + cylinderRatio));
		}
	}

	record Boattail(double length, double foreDiameter, double aftDiameter) implements SupersonicBody {
		@Override public Coordinate potential(FlightConditions conditions, Coordinate baseline) {
			double weight = DatcomCharts.envelope(conditions);
			if (weight == 0) return baseline;
			double beta = conditions.getBeta();
			double ratio = aftDiameter / foreDiameter;
			double corner = beta * (foreDiameter - aftDiameter) / (2 * length);
			double fineness = length / (foreDiameter * beta);
			// Fade through the last 10% of each Mach-dependent chart limit.
			weight *= FiniteWingPressure.smooth((.5 - corner) / .05) * FiniteWingPressure.smooth((2.5 - fineness) / .25);
			if (weight == 0) return baseline;
			double slope = (1 - ratio * ratio) * DatcomCharts.BOATTAIL_FORCE.at(fineness);
			return blend(conditions, baseline, weight, foreDiameter, slope, length * DatcomCharts.BOATTAIL_CP.at(corner, ratio));
		}
	}
}
