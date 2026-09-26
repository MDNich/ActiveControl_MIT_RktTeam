package info.openrocket.core.rocketcomponent;

import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.BoundingBox;
import info.openrocket.core.util.MathUtil;
import info.openrocket.core.util.Transformation;

import java.util.ArrayList;
import java.util.List;

/**
 * A set of trapezoidal fins with tabs used for roll control.
 *
 * @author Marc D NICHITIU <nichitiu@mit.edu>
 */

public class TabControlledTrapezoidFinSet extends TrapezoidFinSet {

    // Units are in meters.
    public static final double MAX_TAB_ANGLE = Math.PI / 2;

    private double tabSpan;
    private double tabChord;
    private double tabOffset; // Offset from the root of the fin to the root of the tab
    private double tabAngle; // Angle from plane of the fin in radians
    private double CNALPHA; // Angle from plane of the fin in radians

    // simple constructor.
    public TabControlledTrapezoidFinSet() {
        super(4, 0.05, 0.05, 0.025, 0.03);
        this.tabSpan = 0.02;
        this.tabChord = 0.02;
        this.tabOffset = this.getHeight()/2; // Default to halfway up the fin
        this.tabAngle = 0;

    }

    public TabControlledTrapezoidFinSet(TrapezoidFinSet trapezoidFinSet) {
        super(trapezoidFinSet.getFinCount(), trapezoidFinSet.getRootChord(), trapezoidFinSet.getTipChord(),
                trapezoidFinSet.getSweep(), trapezoidFinSet.getHeight());

        boolean freezeRocket = true;
        final FinSet finset = trapezoidFinSet;
        final RocketComponent root = trapezoidFinSet.getRoot();
        List<RocketComponent> toInvalidate = new ArrayList<>();

        try {
            if (freezeRocket && root instanceof Rocket) {
                ((Rocket) root).freeze();
            }

            // Get fin set position and remove fin set
            final RocketComponent parent = finset.getParent();
            final int position;
            if (parent != null) {
                position = parent.getChildPosition(finset);
                parent.removeChild(position);
            } else {
                position = -1;
            }

            toInvalidate = copyFinSetProperties(this, finset);
            updateConvertedName(finset, this);

            // Add replacement fin set to parent
            if (parent != null) {
                parent.addChild(this, position);
            }

            // Convert config listeners
            for (RocketComponent listener : new ArrayList<>(finset.configListeners)) {
                if (listener instanceof FinSet) {
                    finset.removeConfigListener(listener);
                    this.addConfigListener(listener);
                }
            }

        } finally {
            if (freezeRocket && root instanceof Rocket) {
                ((Rocket) root).thaw();
            }
            // Invalidate components after events have been fired
            for (RocketComponent c : toInvalidate) {
                c.invalidate();
            }
        }



        this.tabSpan = 0.02;
        this.tabChord = 0.02;
        this.tabOffset = this.getHeight()/2; // Default to halfway up the fin
        this.tabAngle = 0;
    }

    // full constructor
    public TabControlledTrapezoidFinSet(int fins, double rootChord, double tipChord, double sweep,
                                        double height, double tabSpan, double tabChord, double tabOffset, double tabAngle) {
        super(fins, rootChord, tipChord, sweep, height);
        this.tabSpan = tabSpan;
        this.tabChord = tabChord;
        this.tabAngle = clampTabAngle(tabAngle);
        this.tabOffset = tabOffset;
    }

    public TrapezoidFinSet removeTabs() {
        TabControlledTrapezoidFinSet oldset = this;
        TrapezoidFinSet newset = new TrapezoidFinSet(oldset.getFinCount(), oldset.getRootChord(), oldset.getTipChord(),
                oldset.getSweep(), oldset.getHeight());

        boolean freezeRocket = true;
        final FinSet finset = oldset;
        final RocketComponent root = oldset.getRoot();
        List<RocketComponent> toInvalidate = new ArrayList<>();

        try {
            if (freezeRocket && root instanceof Rocket) {
                ((Rocket) root).freeze();
            }

            // Get fin set position and remove fin set
            final RocketComponent parent = finset.getParent();
            final int position;
            if (parent != null) {
                position = parent.getChildPosition(finset);
                parent.removeChild(position);
            } else {
                position = -1;
            }

            toInvalidate = copyFinSetProperties(newset, finset);
            updateConvertedName(finset, newset);

            // Add replacement fin set to parent
            if (parent != null) {
                parent.addChild(newset, position);
            }

            // Convert config listeners
            for (RocketComponent listener : new ArrayList<>(finset.configListeners)) {
                if (listener instanceof FinSet) {
                    finset.removeConfigListener(listener);
                    newset.addConfigListener(listener);
                }
            }

        } finally {
            if (freezeRocket && root instanceof Rocket) {
                ((Rocket) root).thaw();
            }
            // Invalidate components after events have been fired
            for (RocketComponent c : toInvalidate) {
                c.invalidate();
            }
        }
        return newset;
    }

    private static List<RocketComponent> copyFinSetProperties(FinSet target, FinSet source) {
        List<RocketComponent> toInvalidate = target.copyFrom(source);
        target.setAppearance(source.getAppearance());
        target.setVisible(source.isVisible());
        if (source.isCDOverridden()) {
            target.setOverrideCD(source.getOverrideCD());
        }
        target.setCDOverridden(source.isCDOverridden());
        return toInvalidate;
    }

    private static void updateConvertedName(FinSet source, FinSet target) {
        String sourceComponentTypeName = source.getComponentName();
        String name = target.getName();
        if (name.startsWith(sourceComponentTypeName)) {
            target.setName(target.getComponentName() + name.substring(sourceComponentTypeName.length()));
        }
    }

    public void setTabShape(double tabLength, double tabDepth, double tabOffset, double tabAngle) {
        for (RocketComponent listener : configListeners) {
            if (listener instanceof TabControlledTrapezoidFinSet) {
                ((TabControlledTrapezoidFinSet) listener).setTabShape(tabLength, tabDepth, tabOffset, tabAngle);
            }
        }
        this.tabSpan = tabLength;
        this.tabChord = tabDepth;
        this.tabAngle = clampTabAngle(tabAngle);
        this.tabOffset = tabOffset;
        fireComponentChangeEvent(ComponentChangeEvent.BOTH_CHANGE);
    }


    // Set get.
    public double getTabSpan() {
        return tabSpan;
    }
    public double getTabChord() {
        return tabChord;
    }
    public double getTabAngle() {
        return tabAngle;
    }
    public double getTabOffset() {
        return tabOffset;
    }
    public void setTabAngle(double tabAngle) {
        double clamped = clampTabAngle(tabAngle);
        for (RocketComponent listener : configListeners) {
            if (listener instanceof TabControlledTrapezoidFinSet controlled) {
                controlled.setTabAngle(clamped);
            }
        }
        if (MathUtil.equals(this.tabAngle, clamped)) {
            return;
        }
        this.tabAngle = clamped;
        fireComponentChangeEvent(ComponentChangeEvent.BOTH_CHANGE);
    }

    private static double clampTabAngle(double angle) {
        return Double.isNaN(angle) ? 0 : MathUtil.clamp(angle, -MAX_TAB_ANGLE, MAX_TAB_ANGLE);
    }
    public void setTabSpan(double tabSpan) {
        this.tabSpan = tabSpan;
        fireComponentChangeEvent(ComponentChangeEvent.BOTH_CHANGE);
    }
    public void setTabChord(double tabChord) {
        this.tabChord = tabChord;
        fireComponentChangeEvent(ComponentChangeEvent.BOTH_CHANGE);
    }
    public void setTabOffset(double tabOffset) {
        this.tabOffset = tabOffset;
        fireComponentChangeEvent(ComponentChangeEvent.BOTH_CHANGE);
    }


    public double getCNALPHA() {
        return this.CNALPHA;
    }
    public void setCNALPHA(double newCna) {
        this.CNALPHA = newCna;
        fireComponentChangeEvent(ComponentChangeEvent.BOTH_CHANGE);
    }


    @Override
    public String getComponentName() {
        //// Trapezoidal fin set
        return "Tab Controlled Trapezoidal Fin Set";
    }

    /** Tab corners at zero deflection, ordered trailing root, hinge root, hinge tip, trailing tip. */
    public Coordinate[] getRollCtrlTabNeutralPoints() {
        Coordinate[] fin = getFinPoints();
        Coordinate direction = getTrailingEdgeDirection();
        Coordinate inward = new Coordinate(-direction.y, direction.x, 0).multiply(tabChord);
        Coordinate trailingRoot = fin[fin.length - 1].add(direction.multiply(tabOffset));
        Coordinate trailingTip = trailingRoot.add(direction.multiply(tabSpan));
        return new Coordinate[]{trailingRoot, trailingRoot.add(inward), trailingTip.add(inward), trailingTip};
    }

    private Coordinate getTrailingEdgeDirection() {
        Coordinate[] fin = getFinPoints();
        Coordinate edge = fin[fin.length - 2].sub(fin[fin.length - 1]);
        double length = Math.hypot(edge.x, edge.y);
        return length > MathUtil.EPSILON ? edge.multiply(1 / length) : new Coordinate(0, 1, 0);
    }

    /**
     * Rotate about the upstream hinge, parallel to the trailing edge. Positive angles
     * move the trailing edge toward -Z, consistent with the fin cant convention.
     */
    public Transformation getRollCtrlTabRotation() {
        if (tabAngle == 0) {
            return Transformation.IDENTITY;
        }
        Coordinate hinge = getRollCtrlTabNeutralPoints()[1];
        Coordinate direction = getTrailingEdgeDirection();
        double frameAngle = Math.atan2(-direction.x, direction.y);
        return new Transformation(hinge)
                .applyTransformation(Transformation.rotate_z(frameAngle))
                .applyTransformation(Transformation.rotate_y(tabAngle))
                .applyTransformation(Transformation.rotate_z(-frameAngle))
                .applyTransformation(new Transformation(hinge.multiply(-1)));
    }

    public Coordinate[] getRollCtrlTabPoints() {
        return getRollCtrlTabRotation().transform(getRollCtrlTabNeutralPoints());
    }

    @Override
    public BoundingBox getInstanceBoundingBox() {
        BoundingBox bounds = super.getInstanceBoundingBox();
        Coordinate halfThickness = getRollCtrlTabRotation().linearTransform(new Coordinate(0, 0, getThickness() / 2));
        for (Coordinate point : getRollCtrlTabPoints()) {
            bounds.update(point.add(halfThickness));
            bounds.update(point.sub(halfThickness));
        }
        return bounds;
    }
}
