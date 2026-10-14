package info.openrocket.core.simulation.ensemble;

import static info.openrocket.core.simulation.FlightDataType.*;
import static org.junit.jupiter.api.Assertions.*;

import info.openrocket.core.aerodynamics.AerodynamicForces;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.masscalc.RigidBody;
import info.openrocket.core.rocketcomponent.FinSet;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.exception.SimulationException;
import info.openrocket.core.util.*;
import java.util.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

class EnsembleCoefficientTest extends BaseTestCase {
    private EnsembleRunListener listener(double mass, double drag, double forcing, double damping) {
        return new EnsembleRunListener(EnsembleTest.settings(2, 0, 0),
                new EnsembleRunParameters(1, 42, 288.15, 101325, 0, 0, mass, drag, forcing, damping));
    }

    @Test
    void scalesMassInertiaAndDragAndPreservesUnrelatedForces() {
        var listener = listener(1.2, .8, 1.3, .7);
        var body = new RigidBody(new Coordinate(.4, .1, -.2, 10), 2, 3, 4);
        var scaled = listener.postMassCalculation(null, body);
        assertEquals(12, scaled.getMass(), 1e-12);
        assertEquals(body.cm.setWeight(12), scaled.cm);
        assertEquals(2.4, scaled.Ixx, 1e-12);
        assertEquals(3.6, scaled.Iyy, 1e-12);
        assertEquals(4.8, scaled.Izz, 1e-12);
        var f = new AerodynamicForces().zero();
        f.setCD(2);
        f.setCDaxial(-1.5); // Signed axial drag at inverted attitude.
        f.setFrictionCD(.2);
        f.setPressureCD(.3);
        f.setBaseCD(.5);
        f.setOverrideCD(1);
        f.setCrollForce(-.4);
        f.setCrollDamp(-.1);
        f.setCroll(-.25); // Includes an additional +0.05 moment from another extension.
        f.setCN(3);
        f.setCm(4);
        f.setCP(new Coordinate(.7, 0, 0, 2));
        listener.postAerodynamicCalculation(null, f);
        assertEquals(1.6, f.getCD(), 1e-12);
        assertEquals(-1.2, f.getCDaxial(), 1e-12);
        assertEquals(f.getCD(), f.getBaseCD() + f.getFrictionCD() + f.getPressureCD() + f.getOverrideCD(), 1e-12);
        assertEquals(-.52, f.getCrollForce(), 1e-12);
        assertEquals(-.07, f.getCrollDamp(), 1e-12);
        assertEquals(-.4, f.getCroll(), 1e-12);
        assertEquals(3, f.getCN());
        assertEquals(4, f.getCm());
        assertEquals(.7, f.getCP().x);
        assertNull(listener(1, 1, 1, 1).postMassCalculation(null, body));
        assertNull(listener(1, 1, 1, 1).postAerodynamicCalculation(null, f));
    }

    @Test
    void fullFlightScalesLoadedAndBurnedOutMassAndRecoveryDrag() throws Exception {
        var baseline = EnsembleTest.simulation();
        baseline.simulate(listener(1, 1, 1, 1));
        var heavier = EnsembleTest.simulation();
        heavier.simulate(listener(1.2, 1, 1, 1));
        var a = baseline.getSimulatedData().getBranch(0);
        var b = heavier.getSimulatedData().getBranch(0);
        assertEquals(a.get(TYPE_MASS).get(0) * 1.2, b.get(TYPE_MASS).get(0), 1e-12);
        assertEquals(a.getLast(TYPE_MASS) * 1.2, b.getLast(TYPE_MASS), 1e-12);
        assertEquals(a.get(TYPE_CG_LOCATION).get(0), b.get(TYPE_CG_LOCATION).get(0), 1e-12);
        assertEquals(a.get(TYPE_ROTATIONAL_INERTIA).get(0) * 1.2,
                b.get(TYPE_ROTATIONAL_INERTIA).get(0), 1e-12);
        assertTrue(heavier.getSimulatedData().getMaxAltitude() < baseline.getSimulatedData().getMaxAltitude());
        var draggy = EnsembleTest.simulation();
        draggy.simulate(listener(1, 1.3, 1, 1));
        var d = draggy.getSimulatedData().getBranch(0);
        assertTrue(draggy.getSimulatedData().getMaxAltitude() < baseline.getSimulatedData().getMaxAltitude());
        assertEquals(recoveryCd(a) * 1.3, recoveryCd(d), 1e-10);
    }

    private double recoveryCd(FlightDataBranch branch) {
        double deployment = branch.getEvents().stream()
                .filter(e -> e.getType() == FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT)
                .findFirst().orElseThrow().getTime();
        for (int i = 0; i < branch.getLength(); i++)
            if (branch.get(TYPE_TIME).get(i) > deployment + .1 && Double.isFinite(branch.get(TYPE_DRAG_COEFF).get(i)))
                return branch.get(TYPE_DRAG_COEFF).get(i);
        throw new AssertionError("Missing recovery drag");
    }

    @ParameterizedTest
    @ValueSource(ints = {0, 1, 2, 3})
    void eachSourceProducesRepeatableSpreadAndStoresDrawnMultipliers(int source) throws Exception {
        var s = EnsembleTest.simulation();
        for (var c : s.getRocket()) if (c instanceof FinSet f) f.setCantAngle(.005);
        var sigma = new double[4];
        sigma[source] = .1;
        s.getOptions().setEnsembleSettings(new EnsembleSettings(true, 3, 0, .05, 0, 0, 0, 12345,
                sigma[0], sigma[1], sigma[2], sigma[3]));
        s.simulate();
        var result = s.getSimulatedData().getEnsembleResult();
        var type = source < 2 ? TYPE_ALTITUDE : TYPE_ROLL_RATE;
        assertTrue(result.deviation(0).getMaximum(type) > 1e-6);
        var draws = result.individualRuns();
        var a = draws.parameters(0);
        var b = draws.parameters(1);
        switch (source) {
            case 0 -> assertNotEquals(a.massMultiplier(), b.massMultiplier());
            case 1 -> assertNotEquals(a.dragMultiplier(), b.dragMultiplier());
            case 2 -> assertNotEquals(a.rollForceMultiplier(), b.rollForceMultiplier());
            case 3 -> assertNotEquals(a.rollDampingMultiplier(), b.rollDampingMultiplier());
        }
        s.simulate();
        var repeat = s.getSimulatedData().getEnsembleResult();
        assertEquals(a, repeat.individualRuns().parameters(0));
        assertArrayEquals(result.samples(0, EnsembleMetric.MAX_ALTITUDE),
                repeat.samples(0, EnsembleMetric.MAX_ALTITUDE), 1e-12);
        assertEquals(result.deviation(0).getMaximum(type), repeat.deviation(0).getMaximum(type), 1e-12);
    }

    @Test
    void legacySettingsAndRunParametersDefaultToNominalAndNewFieldsValidate() {
        var settings = EnsembleTest.settings(2, 0, 0);
        var a = new HashMap<>(settings.attributes());
        for (var key : List.of("masssigma", "dragsigma", "rollforcesigma", "rolldampingsigma")) a.remove(key);
        assertEquals(settings, EnsembleSettings.fromAttributes(a));
        var parameters = new EnsembleRunParameters(1, 12, 288, 101325, 0, 0);
        var p = new HashMap<>(parameters.attributes());
        for (var key : List.of("massmultiplier", "dragmultiplier", "rollforcemultiplier", "rolldampingmultiplier")) p.remove(key);
        assertEquals(parameters, EnsembleRunParameters.fromAttributes(p));
        for (var key : List.of("masssigma", "dragsigma", "rollforcesigma", "rolldampingsigma")) {
            for (String invalid : List.of("-0.1", "NaN", "Infinity", "1.01")) {
                var invalidSettings = new HashMap<>(a);
                invalidSettings.put(key, invalid);
                assertThrows(IllegalArgumentException.class, () -> EnsembleSettings.fromAttributes(invalidSettings));
            }
        }
        assertThrows(IllegalArgumentException.class, () -> new EnsembleRunParameters(1, 12, 288, 101325, 0, 0, 0, 1, 1, 1));
    }

    @Test
    void impossibleDrawStopsWithoutReplacingPreviousResults() throws Exception {
        var s = EnsembleTest.simulation();
        s.simulate();
        var previous = s.getSimulatedData();
        // Find a deterministic first draw whose 100% mass uncertainty gives a negative mass.
        int seed = 0;
        while (new Random(((long) seed) ^ 0x6a09e667f3bcc909L).nextGaussian() > -1) seed++;
        s.getOptions().setEnsembleSettings(new EnsembleSettings(true, 2, 0, .05, 0, 0, 0, seed, 1, 0, 0, 0));
        var error = assertThrows(SimulationException.class, s::simulate);
        assertTrue(error.getMessage().contains("nonpositive mass or coefficient"));
        assertSame(previous, s.getSimulatedData());
        assertEquals(0, s.getEnsembleRunNumber());
    }
}
