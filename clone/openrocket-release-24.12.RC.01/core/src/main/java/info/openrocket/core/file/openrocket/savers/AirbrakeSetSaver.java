package info.openrocket.core.file.openrocket.savers;

import info.openrocket.core.rocketcomponent.AirbrakeSet;

import java.util.ArrayList;
import java.util.List;

public class AirbrakeSetSaver extends ExternalComponentSaver {

	private static final AirbrakeSetSaver instance = new AirbrakeSetSaver();

	public static List<String> getElements(info.openrocket.core.rocketcomponent.RocketComponent c) {
		List<String> list = new ArrayList<>();

		list.add("<airbrakeset>");
		instance.addParams(c, list);
		list.add("</airbrakeset>");

		return list;
	}

	@Override
	protected void addParams(info.openrocket.core.rocketcomponent.RocketComponent c, List<String> elements) {
		super.addParams(c, elements);

        info.openrocket.core.rocketcomponent.AirbrakeSet airbrakes = (info.openrocket.core.rocketcomponent.AirbrakeSet) c;

        elements.add("<length>" + airbrakes.getLength() + "</length>");
        elements.add("<width>" + airbrakes.getWidth() + "</width>");
        elements.add("<thickness>" + airbrakes.getThickness() + "</thickness>");
        elements.add("<fracExposed>" + airbrakes.getFracExposed() + "</fracExposed>");
        elements.add("<numAirbrakes>" + airbrakes.getNumAirbrakes() + "</numAirbrakes>");
        elements.add("<overrideCD>" + airbrakes.getOverrideCD() + "</overrideCD>");
        elements.add("<indivAirbrakeMass>" + airbrakes.getIndivAirbrakeMass() + "</indivAirbrakeMass>");
	}

}
