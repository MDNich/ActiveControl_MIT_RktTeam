package info.openrocket.core.rocketcomponent;

import info.openrocket.core.util.Coordinate;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.*;

/**
 * A set of trapezoidal fins with tabs used for roll control.
 *
 * @author Marc D NICHITIU <nichitiu@mit.edu>
 */

public class TabControlledTrapezoidFinSet extends TrapezoidFinSet {

    // Units are in meters.

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
        FreeformFinSet freeform;
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



            // Set name
            final String componentTypeName = finset.getComponentName();
            final String name = this.getName();

            this.setAxialOffset(finset.getAxialOffset());
            this.setThickness(finset.getThickness());
            this.setMaterial(finset.getMaterial());
            this.setFilletMaterial(finset.getFilletMaterial());
            this.setAppearance(finset.getAppearance());

            if (name.startsWith(componentTypeName)) {
                this.setName(this.getComponentName() +
                        name.substring(componentTypeName.length()));
            }
            this.setName(finset.getName() + " (Tab Ctrlled)");

            this.setAppearance(finset.getAppearance());

            // Add freeform fin set to parent
            if (parent != null) {
                parent.addChild(this, position);
            }

            // Convert config listeners
            for (RocketComponent listener : finset.configListeners) {
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
        this.tabAngle = tabAngle;
        this.tabOffset = tabOffset;
    }

    public void setTabShape(double tabLength, double tabDepth, double tabOffset, double tabAngle) {
        for (RocketComponent listener : configListeners) {
            if (listener instanceof TabControlledTrapezoidFinSet) {
                ((TabControlledTrapezoidFinSet) listener).setTabShape(tabLength, tabDepth, tabOffset, tabAngle);
            }
        }
        this.tabSpan = tabLength;
        this.tabChord = tabDepth;
        this.tabAngle = tabAngle;
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
        this.tabAngle = tabAngle;
        fireComponentChangeEvent(ComponentChangeEvent.BOTH_CHANGE);
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

    public Coordinate[] getRollCtrlTabPoints() {
        List<Coordinate> points = new ArrayList<>(4);

        // Root of the tab is offset from the root of the fin by tabOffset

        // horiz travel of trailing edge is rootChord - tipChord - sweep
        // angle of trailing edge is thus atan((rootChord - tipChord - sweep)/height)

        //double trailingEdgeAng = Math.PI/4;///atan((this.getRootChord() - this.getTipChord() - this.getSweep())/this.getSpan());

        double horizTravel = Math.abs(this.getRootChord() - this.getTipChord() - this.getSweep());
        double trailingEdgeAng =  atan(this.getHeight()/horizTravel);
        System.out.println(trailingEdgeAng);

        if (this.getTipChord() + this.getSweep() > this.getRootChord()) {
            // then the trailing edge is slanting the other way.
            trailingEdgeAng = Math.PI-trailingEdgeAng;
        }



        double cosA = cos(trailingEdgeAng);

        points.add(new Coordinate(this.getRootChord()-cos(trailingEdgeAng)*tabOffset, sin(trailingEdgeAng)*tabOffset));
        points.add(new Coordinate(this.getRootChord()-cos(trailingEdgeAng)*tabOffset - sin(trailingEdgeAng)*tabChord, sin(trailingEdgeAng)*tabOffset-cos(trailingEdgeAng)*tabChord));
        points.add(new Coordinate(this.getRootChord()-cos(trailingEdgeAng)*(tabOffset+tabSpan) - sin(trailingEdgeAng)*tabChord, sin(trailingEdgeAng)*(tabOffset + tabSpan)-cos(trailingEdgeAng)*tabChord));
        points.add(new Coordinate(this.getRootChord()-cos(trailingEdgeAng)*(tabOffset+tabSpan), sin(trailingEdgeAng)*(tabOffset + tabSpan)));

        return points.toArray(new Coordinate[0]);
    }










}
