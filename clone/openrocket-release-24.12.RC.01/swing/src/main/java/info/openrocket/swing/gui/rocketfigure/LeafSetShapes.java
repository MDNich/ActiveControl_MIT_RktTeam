package info.openrocket.swing.gui.rocketfigure;

import info.openrocket.core.rocketcomponent.AirbrakeSet;
import info.openrocket.core.rocketcomponent.FinSet;
import info.openrocket.core.rocketcomponent.LeafSet;
import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.startup.Application;
import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.Transformation;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;


public class LeafSetShapes extends RocketComponentShapes {
    @Override
    public Class<LeafSet> getShapeClass() {
        return LeafSet.class;
    }

    @Override
    public RocketComponentShapes[] getShapesSide(final RocketComponent component, final Transformation transformation) {
        LeafSet leafset = (LeafSet) component;
        return new RocketComponentShapes[0];
    }

    @Override
    public RocketComponentShapes[] getShapesBack(final RocketComponent component, final Transformation transformation) {
        LeafSet leafset = (LeafSet) component;
        return new RocketComponentShapes[0];
    }
	
	
}
