package info.openrocket.core.aerodynamics;

import info.openrocket.core.aerodynamics.barrowman.FiniteWingPressure;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class FiniteWingPressureTest {
    @Test void rectangularPanelMatchesAnalyticTipConeForceAndMoment() {
        double beta = Math.sqrt(3);
        var wing = new FiniteWingPressure(1, 1, 1, 0);
        var load = wing.at(2);
        // At each x the tip region has width x/beta and its mean pressure is
        // exactly half the 2-D pressure. Integrate this deficit with 1 and x.
        assertEquals(4 / beta * (1 - 1 / (4 * beta)), load.force(), 0.004);
        assertEquals(4 / beta * (0.5 - 1 / (6 * beta)), load.moment(), 0.004);
    }

    @Test void widePanelApproachesTwoFaceAckeretSlopeAtMidchord() {
        var load = new FiniteWingPressure(1, 1, 1000, 0).at(3);
        assertEquals(4 / Math.sqrt(8), load.force() / 1000, 0.001);
        assertEquals(0.5, load.moment() / load.force(), 0.001);
    }

    @Test void experimentalEnvelopeRespectsIncidenceAndSweptLeadingEdge() {
        var iris = new FiniteWingPressure(.2032, .0762, .0889, .127);
        assertEquals(0, iris.weight(1.5, 0));
        assertEquals(1, iris.weight(2.8, 0));
        assertEquals(0, iris.weight(2.8, Math.toRadians(10)));
        assertEquals(0, iris.weight(5, 0));
        assertEquals(0, new FiniteWingPressure(1, 1, 0.1, 0).weight(3, 0));
        assertTrue(iris.at(2.8).moment() / iris.at(2.8).force() < .2032);
    }
}
