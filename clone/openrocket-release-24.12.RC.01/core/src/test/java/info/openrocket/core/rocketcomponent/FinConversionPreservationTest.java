package info.openrocket.core.rocketcomponent;

import static org.junit.jupiter.api.Assertions.*;

import info.openrocket.core.appearance.Appearance;
import info.openrocket.core.document.OpenRocketDocumentFactory;
import info.openrocket.core.file.GeneralRocketLoader;
import info.openrocket.core.file.GeneralRocketSaver;
import info.openrocket.core.material.Material;
import info.openrocket.core.rocketcomponent.position.AngleMethod;
import info.openrocket.core.rocketcomponent.position.AxialMethod;
import info.openrocket.core.util.*;
import java.nio.file.Path;
import java.util.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

class FinConversionPreservationTest extends BaseTestCase {
    @TempDir Path directory;

    @ParameterizedTest
    @ValueSource(ints = {1, 4})
    void addingAndRemovingControlTabsPreservesCommonFinProperties(int count) {
        for (AxialMethod axial : List.of(AxialMethod.TOP, AxialMethod.MIDDLE, AxialMethod.BOTTOM, AxialMethod.ABSOLUTE)) {
            var rocket = new Rocket();
            var stage = new AxialStage();
            rocket.addChild(stage);
            var body = new BodyTube(.8, .05, .002);
            stage.addChild(body);
            body.addChild(new MassComponent());
            var fin = new TrapezoidFinSet(count, .18, .06, .04, .09);
            body.addChild(fin);
            rocket.enableEvents();
            fin.setName("Custom fins");
            fin.setComment("Keep my configuration");
            fin.setAxialOffset(axial, axial == AxialMethod.BOTTOM ? -.07 : .12);
            fin.setThickness(.004);
            fin.setCrossSection(FinSet.CrossSection.TRIANGULAR);
            fin.setLeadingEdgeAngle(.25);
            fin.setCantAngle(.07);
            fin.setAngleMethod(AngleMethod.MIRROR_XY);
            fin.setAngleOffset(.8);
            fin.setTabHeight(.015);
            fin.setTabLength(.08);
            fin.setTabOffsetMethod(AxialMethod.MIDDLE);
            fin.setMountingTabOffset(.012);
            fin.setFilletRadius(.008);
            fin.setFilletMaterial(Material.newMaterial(Material.Type.BULK, "Epoxy test", 1400, true));
            fin.setMaterial(Material.newMaterial(Material.Type.BULK, "Fin test", 700, true));
            fin.setFinish(ExternalComponent.Finish.ROUGH);
            fin.setAppearance(new Appearance(new ORColor(255, 0, 0), .4, null));
            fin.setVisible(false);
            fin.setColor(new ORColor(0, 0, 255));
            fin.setLineStyle(LineStyle.DASHDOT);
            fin.getInsideColorComponentHandler().setInsideAppearance(new Appearance(new ORColor(0, 255, 0), .2, null));
            fin.getInsideColorComponentHandler().setSeparateInsideOutside(true);
            var expected = properties(fin);
            var cg = fin.getComponentCG(); // Warm caches before replacing the component.
            double mass = fin.getComponentMass();
            var position = fin.getPosition();
            var controlled = new TabControlledTrapezoidFinSet(fin);
            assertEquals(expected, properties(controlled));
            assertEquals(position, controlled.getPosition());
            assertEquals(cg, controlled.getComponentCG());
            assertEquals(mass, controlled.getComponentMass(), 1e-12);
            assertSame(controlled, body.getChild(1));
            controlled.setTabOffset(.031); // Roll-control location must not move the mounting tab.
            var plain = controlled.removeTabs();
            assertEquals(expected, properties(plain));
            assertEquals(position, plain.getPosition());
            assertEquals(cg, plain.getComponentCG());
            assertEquals(mass, plain.getComponentMass(), 1e-12);
            assertSame(plain, body.getChild(1));
            // Exercise the stored mounting offset after conversion, rather than just its cached position.
            double front = plain.getTabFrontEdge();
            plain.updateTabPosition();
            assertEquals(front, plain.getTabFrontEdge(), 1e-12);
        }
    }

    private List<Object> properties(FinSet f) {
        return Arrays.asList(f.getID(), f.getName(), f.getComment(), f.getFinCount(),
                f.getThickness(), f.getCrossSection(), f.getLeadingEdgeAngle(), f.getCantAngle(),
                f.getAngleMethod(), f.getAngleOffset(), f.getAxialMethod(), f.getAxialOffset(),
                f.getTabHeight(), f.getTabLength(), f.getTabOffsetMethod(), f.getMountingTabOffset(),
                f.getFilletRadius(), f.getFilletMaterial(), f.getMaterial(), f.getFinish(),
                f.getAppearance(), f.isVisible(), f.getColor(), f.getLineStyle(),
                f.getInsideColorComponentHandler().getInsideAppearance(),
                f.getInsideColorComponentHandler().isSeparateInsideOutside());
    }

    @Test
    void conversionPreservesActiveOverrides() {
        var controlled = new TabControlledTrapezoidFinSet();
        controlled.setOverrideMass(.2);
        controlled.setMassOverridden(true);
        controlled.setOverrideCGX(.03);
        controlled.setCGOverridden(true);
        controlled.setOverrideCD(.5);
        controlled.setCDOverridden(true);
        var fin = controlled.removeTabs();
        assertTrue(fin.isMassOverridden());
        assertEquals(.2, fin.getOverrideMass());
        assertTrue(fin.isCGOverridden());
        assertEquals(.03, fin.getOverrideCGX());
        assertTrue(fin.isCDOverridden());
        assertEquals(.5, fin.getOverrideCD());
    }

    @Test
    void mountingAndControlOffsetsSaveAndReloadIndependently() throws Exception {
        var doc = OpenRocketDocumentFactory.createEmptyRocket();
        doc.getRocket().addChild(new AxialStage());
        var body = new BodyTube(.8, .05, .002);
        doc.getRocket().getChild(0).addChild(body);
        var fins = new TabControlledTrapezoidFinSet();
        body.addChild(fins);
        fins.setRootChord(.18);
        fins.setTabHeight(.01);
        fins.setTabLength(.04);
        fins.setTabOffsetMethod(AxialMethod.TOP);
        fins.setMountingTabOffset(.012);
        fins.setTabOffset(.033);
        Path file = directory.resolve("tabs.ork");
        new GeneralRocketSaver().save(file.toFile(), doc, doc.getDefaultStorageOptions());
        var loader = new GeneralRocketLoader(file.toFile());
        var loaded = loader.load();
        assertTrue(loader.getWarnings().isEmpty(), loader.getWarnings().toString());
        var copy = (TabControlledTrapezoidFinSet) loaded.getRocket().findComponent(fins.getID());
        assertEquals(.012, copy.getMountingTabOffset(), 1e-12);
        assertEquals(.033, copy.getTabOffset(), 1e-12);
        assertEquals(.012, copy.removeTabs().getTabOffset(), 1e-12);
    }
}
