package info.openrocket.core.file.openrocket;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import info.openrocket.core.file.openrocket.savers.RocketComponentSaver;
import info.openrocket.core.file.openrocket.savers.TrapezoidFinSetSaver;
import info.openrocket.core.rocketcomponent.AirbrakeSet;
import info.openrocket.core.rocketcomponent.FinSet;
import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.rocketcomponent.TabControlledTrapezoidFinSet;

/** Writes approximations without changing components, IDs, or their event listeners. */
final class VanillaComponentSaver extends RocketComponentSaver {
    static final double MAX_CANT = Math.toRadians(15);

    static List<String> getElements(RocketComponent component, Supplier<List<String>> ordinary) {
        if (component instanceof AirbrakeSet brakes) {
            return ballast(brakes);
        }
        List<String> elements = component instanceof TabControlledTrapezoidFinSet
                ? TrapezoidFinSetSaver.getElements(component) : ordinary.get();
        if (component instanceof FinSet fins) {
            elements.removeIf(s -> s.startsWith("<leadingedgeangle>"));
            for (int i = 0; i < elements.size(); i++) {
                String s = elements.get(i);
                if (s.startsWith("<crosssection>") && fins.getCrossSection() == FinSet.CrossSection.TRIANGULAR)
                    elements.set(i, "<crosssection>rounded</crosssection>");
                if (s.startsWith("<cant>"))
                    elements.set(i, "<cant>" + Math.toDegrees(Math.max(-MAX_CANT,
                            Math.min(MAX_CANT, fins.getCantAngle()))) + "</cant>");

            }
        }
        return elements;
    }

    private static List<String> ballast(AirbrakeSet brakes) {
        List<String> elements = new ArrayList<>();
        elements.add("<masscomponent>");
        new VanillaComponentSaver().addParams(brakes, elements);
        // Different component lengths change BOTTOM/MIDDLE offsets: use the actual
        // parent-relative position and retain the source component's effective axial CG.
        elements.removeIf(s -> s.startsWith("<axialoffset ") || s.startsWith("<position ")
                || s.startsWith("<overridecg>") || s.startsWith("<preset ")
                || s.startsWith("<overridecd>") || s.startsWith("<overridesubcomponentscd>"));
        elements.add("<axialoffset method=\"top\">" + brakes.getPosition().x + "</axialoffset>");
        elements.add("<position type=\"top\">" + brakes.getPosition().x + "</position>");
        elements.add("<overridecg>" + brakes.getCG().x + "</overridecg>");
        elements.add("<packedlength>" + brakes.getThickness() + "</packedlength>");
        elements.add("<packedradius>" + brakes.getWidth() / 2 + "</packedradius>");
        elements.add("<radialposition>0</radialposition>");
        elements.add("<radialdirection>0</radialdirection>");
        elements.add("<mass>" + brakes.getMass() + "</mass>");
        elements.add("<masscomponenttype>masscomponent</masscomponenttype>");
        elements.add("</masscomponent>");
        return elements;
    }
}
