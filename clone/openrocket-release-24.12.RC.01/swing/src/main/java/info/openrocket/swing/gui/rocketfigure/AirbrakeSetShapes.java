package info.openrocket.swing.gui.rocketfigure;

import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.Transformation;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

import static info.openrocket.swing.gui.rocketfigure.FinSetShapes.generatePath;

public class AirbrakeSetShapes extends RocketComponentShapes {

    @Override
    public Class<AirbrakeSet> getShapeClass() {
        return AirbrakeSet.class;
    }

    @Override
    public RocketComponentShapes[] getShapesSide(final RocketComponent component, final Transformation transformation) {
        AirbrakeSet airbrakeset = (AirbrakeSet) component;

        double[][] ninetyDegRotMatrix3D = new double[][]{   {1, 0, 0},
                                                            {0, 0, 1},
                                                            {0, -1, 0}};
        Transformation ninetyDegRot = new Transformation(ninetyDegRotMatrix3D);
        ArrayList<RocketComponentShapes> shapeList = new ArrayList<>();

        for(int i = 0; i < 4; i++) {
            ArrayList<Coordinate> coords1 = airbrakeset.getComponentPoints_top();
            ArrayList<Coordinate> coords2 = airbrakeset.getComponentPoints_side_1();
            ArrayList<Coordinate> coords3 = airbrakeset.getComponentPoints_side_2();
            ArrayList<Coordinate> coords4 = airbrakeset.getComponentPoints_side_3();
            ArrayList<Coordinate> coords5 = airbrakeset.getComponentPoints_side_4();
            ArrayList<Coordinate> coords6 = airbrakeset.getComponentPoints_bottom();
            ArrayList<Coordinate> allCoords = airbrakeset.getComponentPoints();

            Coordinate[] newCoords1 = transformation.transform(coords1.toArray(new Coordinate[0]));
            Coordinate[] newCoords2 = transformation.transform(coords2.toArray(new Coordinate[0]));
            Coordinate[] newCoords3 = transformation.transform(coords3.toArray(new Coordinate[0]));
            Coordinate[] newCoords4 = transformation.transform(coords4.toArray(new Coordinate[0]));
            Coordinate[] newCoords5 = transformation.transform(coords5.toArray(new Coordinate[0]));
            Coordinate[] newCoords6 = transformation.transform(coords6.toArray(new Coordinate[0]));
            Coordinate[] patchLine = transformation.transform(new Coordinate[]{allCoords.get(4), allCoords.get(0)});


            for(int j = 0; j < i; j++) {
                newCoords1 = ninetyDegRot.transform(newCoords1);
                newCoords2 = ninetyDegRot.transform(newCoords2);
                newCoords3 = ninetyDegRot.transform(newCoords3);
                newCoords4 = ninetyDegRot.transform(newCoords4);
                newCoords5 = ninetyDegRot.transform(newCoords5);
                newCoords6 = ninetyDegRot.transform(newCoords6);
                patchLine = ninetyDegRot.transform(patchLine);
            }





            // Generate shapes

            shapeList.add(new RocketComponentShapes(generatePath(newCoords1), airbrakeset));
            shapeList.add(new RocketComponentShapes(generatePath(newCoords2), airbrakeset));
            shapeList.add(new RocketComponentShapes(generatePath(newCoords3), airbrakeset));
            shapeList.add(new RocketComponentShapes(generatePath(newCoords4), airbrakeset));
            shapeList.add(new RocketComponentShapes(generatePath(newCoords5), airbrakeset));
            shapeList.add(new RocketComponentShapes(generatePath(newCoords6), airbrakeset));
            shapeList.add(new RocketComponentShapes(generatePath(patchLine), airbrakeset));
        }






        return shapeList.toArray(new RocketComponentShapes[0]);
    }

    @Override
    public RocketComponentShapes[] getShapesBack(final RocketComponent component, final Transformation transformation) {
        AirbrakeSet airbrakeset = (AirbrakeSet) component;
        return new RocketComponentShapes[0];
    }


}
