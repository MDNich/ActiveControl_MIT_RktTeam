package info.openrocket.core.rocketcomponent;

import java.util.ArrayList;
import java.util.List;

import info.openrocket.core.l10n.Translator;
import info.openrocket.core.startup.Application;
import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.MathUtil;

/**
 * A set of trapezoidal fins with tabs used for roll control
 *
 * @author Marc D NICHITIU <nichitiu@mit.edu>
 */

public class TabControlledTrapezoidFinSet extends TrapezoidFinSet{

    // Units are in meters.

    private double tabLength;
    private double tabDepth;
    private double tabAngle; // Angle from plane of the fin in radians

    // simple constructor.
    public TabControlledTrapezoidFinSet() {
        super(4, 0.05, 0.05, 0.025, 0.03);
        this.tabLength = 0.02;
        this.tabDepth = 0.02;
        this.tabAngle = 0;
    }

    // full constructor
    public TabControlledTrapezoidFinSet(int fins, double rootChord, double tipChord, double sweep,
                                       double height, double tabLength, double tabDepth, double tabAngle) {
        super(fins, rootChord, tipChord, sweep, height);
        this.tabLength = tabLength;
        this.tabDepth = tabDepth;
        this.tabAngle = tabAngle;
    }

    public void setTabShape(double tabLength, double tabDepth, double tabAngle) {
        for (RocketComponent listener : configListeners) {
            if (listener instanceof TabControlledTrapezoidFinSet) {
                ((TabControlledTrapezoidFinSet) listener).setTabShape(tabLength, tabDepth, tabAngle);
            }
        }
        this.tabLength = tabLength;
        this.tabDepth = tabDepth;
        this.tabAngle = tabAngle;
        fireComponentChangeEvent(ComponentChangeEvent.BOTH_CHANGE);
    }


    // Set get.
    public double getTabLength() {
        return tabLength;
    }
    public double getTabDepth() {
        return tabDepth;
    }
    public double getTabAngle() {
        return tabAngle;
    }
    public void setTabAngle(double tabAngle) {
        this.tabAngle = tabAngle;
        fireComponentChangeEvent(ComponentChangeEvent.BOTH_CHANGE);
    }
    public void setTabLength(double tabLength) {
        this.tabLength = tabLength;
        fireComponentChangeEvent(ComponentChangeEvent.BOTH_CHANGE);
    }
    public void setTabDepth(double tabDepth) {
        this.tabDepth = tabDepth;
        fireComponentChangeEvent(ComponentChangeEvent.BOTH_CHANGE);
    }






}
