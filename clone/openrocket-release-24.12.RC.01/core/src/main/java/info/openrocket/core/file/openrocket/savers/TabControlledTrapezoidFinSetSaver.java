package info.openrocket.core.file.openrocket.savers;

import java.util.ArrayList;
import java.util.List;

public class TabControlledTrapezoidFinSetSaver extends FinSetSaver {

	private static final TabControlledTrapezoidFinSetSaver instance = new TabControlledTrapezoidFinSetSaver();

	public static ArrayList<String> getElements(info.openrocket.core.rocketcomponent.RocketComponent c) {
		ArrayList<String> list = new ArrayList<>();

		list.add("<tabctrltrapezoidfinset>");
		instance.addParams(c, list);
		list.add("</tabctrltrapezoidfinset>");

		return list;
	}

	@Override
	protected void addParams(info.openrocket.core.rocketcomponent.RocketComponent c, List<String> elements) {
		super.addParams(c, elements);

		info.openrocket.core.rocketcomponent.TabControlledTrapezoidFinSet fins = (info.openrocket.core.rocketcomponent.TabControlledTrapezoidFinSet) c;
		elements.add("<rootchord>" + fins.getRootChord() + "</rootchord>");
		elements.add("<tipchord>" + fins.getTipChord() + "</tipchord>");
		elements.add("<sweeplength>" + fins.getSweep() + "</sweeplength>");
		elements.add("<height>" + fins.getHeight() + "</height>");

        elements.add("<tabspan>" + fins.getTabSpan() + "</tabspan>");
        elements.add("<tabchord>" + fins.getTabChord() + "</tabchord>");
        elements.add("<taboffset>" + fins.getTabOffset() + "</taboffset>");
        elements.add("<tabangle>" + fins.getTabAngle() + "</tabangle>");

	}

}
