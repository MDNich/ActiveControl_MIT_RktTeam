import com.google.inject.Guice;
import info.openrocket.core.aerodynamics.BarrowmanCalculator;
import info.openrocket.core.aerodynamics.FlightConditions;
import info.openrocket.core.plugin.PluginModule;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.rocketcomponent.position.AxialMethod;
import info.openrocket.core.startup.Application;
import info.openrocket.swing.startup.GuiModule;
import java.nio.file.*;
import java.util.Locale;

/** Offline, geometry-based comparison with two published wind-tunnel datasets. */
public final class FinModelBenchmark {
    static final double INCH = 0.0254;

    static Rocket makeModel(boolean arcas) {
        Rocket rocket = new Rocket();
        AxialStage stage = new AxialStage();
        rocket.addChild(stage);
        // Dimensions in inches from the drawings. Convert at the OR boundary.
        double noseLength = arcas ? 10.6 : 25.9;
        double radius = arcas ? 1.125 : 3.0;
        double bodyEnd = arcas ? 39.14 : 61.7;
        NoseCone nose = new NoseCone(arcas ? Transition.Shape.OGIVE : Transition.Shape.POWER,
                noseLength * INCH, radius * INCH);
        if (!arcas) nose.setShapeParameter(.75);
        stage.addChild(nose);
        BodyTube tube = new BodyTube((bodyEnd - noseLength) * INCH, radius * INCH, .001);
        stage.addChild(tube);
        if (arcas) {
            Transition tail = new Transition();
            stage.addChild(tail);
            tail.setShapeType(Transition.Shape.CONICAL);
            tail.setLength((40.95 - bodyEnd) * INCH);
            tail.setForeRadius(radius * INCH);
            tail.setAftRadius(.735 * INCH);
        }
        double root = arcas ? 3.38 : 9.15;
        double tip = arcas ? 2.165 : 6.54;
        double height = arcas ? 2.103 : 4.95;
        double sweep = arcas ? height * Math.tan(Math.toRadians(30)) : height;
        double leadingStation = arcas ? 37.30 : bodyEnd - root;
        TrapezoidFinSet fins = new TrapezoidFinSet(4, root * INCH, tip * INCH, sweep * INCH, height * INCH);
        tube.addChild(fins);
        fins.setAxialMethod(AxialMethod.TOP);
        fins.setAxialOffset((leadingStation - noseLength) * INCH);
        fins.setCantAngle(Math.toRadians(arcas ? 0 : .8));
        fins.setThickness((arcas ? .15 : 2 * root * Math.tan(Math.toRadians(5))) * INCH);
        rocket.enableEvents();
        return rocket;
    }

    public static void main(String[] args) throws Exception {
        System.setProperty("openrocket.bypass.presets", "true");
        System.setProperty("openrocket.bypass.motors", "true");
        var gui = new GuiModule();
        Application.setInjector(Guice.createInjector(gui, new PluginModule()));
        gui.startLoader();
        var measurements = Files.readAllLines(Path.of(args[0]));
        try (var out = new java.io.PrintWriter(Files.newBufferedWriter(Path.of(args[1])))) {
            out.println("geometry,model,mach,measured_cna,predicted_cna,measured_cp_d,predicted_cp_d,measured_cm_alpha,predicted_cm_alpha");
            for (String name : new String[]{"ARCAS", "Yuska-B"}) {
                boolean arcas = name.equals("ARCAS");
                var configuration = makeModel(arcas).getSelectedConfiguration();
                double referenceStation = arcas ? 25.95 / 2.25 : 61.7 / 6.0;
                double[][] squaredErrors = new double[2][3];
                int supersonicPoints = 0;
                for (boolean finite : new boolean[]{false, true}) {
                    var solver = new BarrowmanCalculator(finite);
                    var conditions = new FlightConditions(configuration);
                    for (String row : measurements.subList(1, measurements.size())) {
                        String[] fields = row.split(",");
                        if (!fields[0].equals(name)) continue;
                        double mach = Double.parseDouble(fields[1]);
                        double measuredSlope = Double.parseDouble(fields[2]);
                        double measuredCP = Double.parseDouble(fields[3]);
                        conditions.setMach(mach);
                        var cp = solver.getCP(configuration, conditions, null);
                        double predictedCP = cp.x / conditions.getRefLength();
                        double measuredMoment = measuredSlope * (referenceStation - measuredCP);
                        double predictedMoment = cp.weight * (referenceStation - predictedCP);
                        if (mach >= 1.5) {
                            if (!finite) supersonicPoints++;
                            double[] errors = {cp.weight - measuredSlope, predictedCP - measuredCP,
                                    predictedMoment - measuredMoment};
                            for (int i = 0; i < errors.length; i++) squaredErrors[finite ? 1 : 0][i] += errors[i] * errors[i];
                        }
                        out.printf(Locale.ROOT, "%s,%s,%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f%n",
                                name, finite ? "candidate" : "legacy", mach, measuredSlope, cp.weight,
                                measuredCP, predictedCP, measuredMoment, predictedMoment);
                    }
                    // A dense sweep catches singularities or discontinuities a few tunnel points miss.
                    double previousCP = Double.NaN, previousSlope = Double.NaN;
                    for (int k = 750; k <= 5001; k++) {
                        conditions.setMach(k / 1000.0);
                        var cp = solver.getCP(configuration, conditions, null);
                        double position = cp.x / conditions.getRefLength();
                        if (!Double.isFinite(position + cp.weight) || Math.abs(position - previousCP) > .1
                                || Math.abs(cp.weight - previousSlope) > .15)
                            throw new AssertionError("Abrupt or invalid load: " + name + " M=" + conditions.getMach());
                        previousCP = position;
                        previousSlope = cp.weight;
                    }
                }
                if (supersonicPoints != 5) throw new AssertionError("Incomplete benchmark for " + name);
                for (int i = 0; i < 3; i++) {
                    if (!(squaredErrors[1][i] < squaredErrors[0][i]))
                        throw new AssertionError("Candidate failed supersonic error comparison for " + name + " metric " + i);
                }
            }
            if (out.checkError()) throw new java.io.IOException("Could not write comparison");
        }
        System.exit(0);
    }
}
