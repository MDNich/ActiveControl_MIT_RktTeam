package info.openrocket.core.rocketcomponent;

import info.openrocket.core.rocketcomponent.position.AxialMethod;
import info.openrocket.core.rocketcomponent.position.AxialPositionable;
import info.openrocket.core.util.ArrayList;
import info.openrocket.core.util.BoundingBox;
import info.openrocket.core.util.Coordinate;
import org.slf4j.LoggerFactory;

import java.util.Collection;
import java.util.List;

/**
 * Represents a set of airbrakes attached to a rocket. Airbrakes are used
 * to modify the aerodynamic properties of the rocket by increasing drag,
 * which is useful for controlling descent or adjusting flight performance.
 * This class is a specialized extension of the {@code LeafSet} base class.
 *
 * Features:
 * - Provides parameters for the physical dimensions (length, width, thickness)
 *   of the airbrakes.
 * - Includes properties such as mass, number of airbrakes, and their center of gravity (CG) offset.
 * - Supports aerodynamics concepts via drag coefficients for perpendicular
 *   and parallel orientation.
 * - Exposes methods for calculating physical properties (e.g. fraction of airbrakes exposed).
 *
 * Class behavior:
 * - Implements several abstract methods from {@code LeafSet}, though in this
 *   implementation these methods primarily return placeholders.
 * - Provides setters for dimension-related attributes with basic validation
 *   to ensure physical constraints and meaningful data.
 */

public class AirbrakeSet extends LeafSet implements AxialPositionable, BoxBounded {

    @Override
    public String getComponentName() {
        return "Airbrake Set (4)";
    }

    private static final org.slf4j.Logger log = LoggerFactory.getLogger(AirbrakeSet.class);
    private double length = 0; // y direction, radial
    private double width = 0; // z direction, tangential
    private double thickness = 0; // x direction, axial

    private double CG_offset = 0; // also called $d$, distance from CG to axis of rocket
    private double indivAirbrakeMass = 0; // mass, kg
    private int numAirbrakes = 4; // number of airbrakes

    public static double CD_perp = 1.28;
    public static double CD_par = 0.01;

    public static double overrideCD = 1;

    private double localOverrideCD = 1;


    public double fudgefactor = 1;


    /**
     * Default constructor that creates an AirbrakeSet with preset dimensions and properties.
     */
    public AirbrakeSet() {
        this(0.025, 0.01, 0.005, 0.025/2.0, 0.023, 4);
    }

    /**
     * Constructs an AirbrakeSet with specified dimensions and properties.
     *
     * @param length            The length (y direction, radial) of each airbrake
     * @param width             The width (z direction, tangential) of each airbrake
     * @param thickness         The thickness (x direction, axial) of each airbrake
     * @param CG_offset         The distance from CG to axis of rocket
     * @param indivAirbrakeMass The mass of each individual airbrake
     * @param numAirbrakes      The number of airbrakes in the set
     */
    public AirbrakeSet(double length, double width, double thickness, double CG_offset, double indivAirbrakeMass, int numAirbrakes) {
        super(AxialMethod.TOP);
        this.setLength(length);
        this.setWidth(width);
        this.setThickness(thickness);
        this.setCG_offset(CG_offset);
        this.setIndivAirbrakeMass(indivAirbrakeMass);
        this.setNumAirbrakes(numAirbrakes);

        fireComponentChangeEvent(ComponentChangeEvent.AEROMASS_CHANGE);
    }

    /**
     * Calculates and returns the volume of a single airbrake component.
     *
     * @return The volume of one airbrake component
     */
    @Override
    public double getComponentVolume() {
        return length * width * thickness;
    }

    /**
     * Returns the center of gravity coordinate for the airbrake component.
     *
     * @return The CG coordinate relative to the component's position
     */
    @Override
    public Coordinate getComponentCG() {
        Coordinate root = this.getPosition();
        return new Coordinate(root.x+thickness/2.0, 0,0);
    }

    public Coordinate getSingleAirbrakeCG() {
        Coordinate root = this.getPosition();
        return new Coordinate(root.x+thickness/2.0, root.z + CG_offset, root.y);
    }

    /**
     * Calculates the longitudinal unit inertia of the airbrake.
     * Returns the average of Iyy and Izz moments of inertia.
     *
     * @return The longitudinal unit inertia
     */
    @Override
    public double getLongitudinalUnitInertia() {
        // return the average of Iyy and Izz
        double h = thickness;
        double l = length;
        double w = width;
        double d = CG_offset;
        return 0.5*((h*h+w*w)/12.0 + (l*l+h*h)/12.0 + d*d);
    }

    /**
     * Calculates the rotational unit inertia (Ixx) of the airbrake.
     *
     * @return The rotational unit inertia
     */
    @Override
    public double getRotationalUnitInertia() {
        // return Ixx
        double l = length;
        double w = width;
        double d = CG_offset;
        return d*d + (l*l + w*w)/12.0;
    }

    /**
     * Determines if this component can have child components.
     * Always returns false for airbrakes.
     *
     * @return false
     */
    @Override
    public boolean allowsChildren() {
        return false;
    }

    /**
     * Checks if the given component type is compatible with this component.
     * Always returns false for airbrakes.
     *
     * @param type The component type to check
     * @return false
     */
    @Override
    public boolean isCompatible(Class<? extends RocketComponent> type) {
        return false;
    }





    /**
     * Adds bounding coordinates to the given set.  The body tube will fit within the
     * convex hull of the points.
     *
     * Currently, the points are simply a rectangular box around the body tube.
     */
    @Override
    public Collection<Coordinate> getComponentBounds() {
        ArrayList<Coordinate> bounds = new ArrayList<>(2);
        // top view: rocket at left, y horizontal, z up
        Coordinate CG = this.getSingleAirbrakeCG();

        Coordinate upperTopRightPoint    = new Coordinate(CG.x + thickness/2.0, CG.y + length/2.0, CG.z + width/2.0);
        Coordinate upperTopLeftPoint     = new Coordinate(CG.x + thickness/2.0, CG.y - length/2.0, CG.z + width/2.0);
        Coordinate upperBottomLeftPoint  = new Coordinate(CG.x + thickness/2.0, CG.y - length/2.0, CG.z - width/2.0);
        Coordinate upperBottomRightPoint = new Coordinate(CG.x + thickness/2.0, CG.y + length/2.0, CG.z - width/2.0);

        Coordinate lowerTopRightPoint    = new Coordinate(CG.x - thickness/2.0, CG.y + length/2.0, CG.z + width/2.0);
        Coordinate lowerTopLeftPoint     = new Coordinate(CG.x - thickness/2.0, CG.y - length/2.0, CG.z + width/2.0);
        Coordinate lowerBottomLeftPoint  = new Coordinate(CG.x - thickness/2.0, CG.y - length/2.0, CG.z - width/2.0);
        Coordinate lowerBottomRightPoint = new Coordinate(CG.x - thickness/2.0, CG.y + length/2.0, CG.z - width/2.0);
    
        
        bounds.add(upperTopLeftPoint);
        bounds.add(lowerBottomRightPoint);

        return bounds;
    }

    public ArrayList<Coordinate> getComponentPoints_top() {
        ArrayList<Coordinate> bounds = new ArrayList<>(4);

        Coordinate CG = this.getSingleAirbrakeCG();

        Coordinate upperTopRightPoint    = new Coordinate(CG.x + thickness/2.0, CG.y + length/2.0, CG.z + width/2.0);
        Coordinate upperTopLeftPoint     = new Coordinate(CG.x + thickness/2.0, CG.y - length/2.0, CG.z + width/2.0);
        Coordinate upperBottomLeftPoint  = new Coordinate(CG.x + thickness/2.0, CG.y - length/2.0, CG.z - width/2.0);
        Coordinate upperBottomRightPoint = new Coordinate(CG.x + thickness/2.0, CG.y + length/2.0, CG.z - width/2.0);

        return new ArrayList<>(List.of(
                new Coordinate[]{upperTopRightPoint, upperTopLeftPoint,
                        upperBottomLeftPoint, upperBottomRightPoint}));
    }
    public ArrayList<Coordinate> getComponentPoints_bottom() {
        ArrayList<Coordinate> bounds = new ArrayList<>(4);

        Coordinate CG = this.getSingleAirbrakeCG();


        Coordinate bottomTopRightPoint    = new Coordinate(CG.x - thickness/2.0, CG.y + length/2.0, CG.z + width/2.0);
        Coordinate bottomTopLeftPoint     = new Coordinate(CG.x - thickness/2.0, CG.y - length/2.0, CG.z + width/2.0);
        Coordinate bottomBottomLeftPoint  = new Coordinate(CG.x - thickness/2.0, CG.y - length/2.0, CG.z - width/2.0);
        Coordinate bottomBottomRightPoint = new Coordinate(CG.x - thickness/2.0, CG.y + length/2.0, CG.z - width/2.0);

        return new ArrayList<>(List.of(
                new Coordinate[]{bottomTopRightPoint, bottomTopLeftPoint,
                        bottomBottomLeftPoint, bottomBottomRightPoint}));
    }
    public ArrayList<Coordinate> getComponentPoints_side_1() {
        ArrayList<Coordinate> bounds = new ArrayList<>(4);

        Coordinate CG = this.getSingleAirbrakeCG();

        Coordinate upperTopRightPoint    = new Coordinate(CG.x + thickness/2.0, CG.y + length/2.0, CG.z + width/2.0);
        Coordinate upperTopLeftPoint     = new Coordinate(CG.x + thickness/2.0, CG.y - length/2.0, CG.z + width/2.0);
        Coordinate upperBottomLeftPoint  = new Coordinate(CG.x + thickness/2.0, CG.y - length/2.0, CG.z - width/2.0);
        Coordinate upperBottomRightPoint = new Coordinate(CG.x + thickness/2.0, CG.y + length/2.0, CG.z - width/2.0);

        Coordinate lowerTopRightPoint    = new Coordinate(CG.x - thickness/2.0, CG.y + length/2.0, CG.z + width/2.0);
        Coordinate lowerTopLeftPoint     = new Coordinate(CG.x - thickness/2.0, CG.y - length/2.0, CG.z + width/2.0);
        Coordinate lowerBottomLeftPoint  = new Coordinate(CG.x - thickness/2.0, CG.y - length/2.0, CG.z - width/2.0);
        Coordinate lowerBottomRightPoint = new Coordinate(CG.x - thickness/2.0, CG.y + length/2.0, CG.z - width/2.0);

        return new ArrayList<>(List.of(
                new Coordinate[]{upperTopRightPoint, upperTopLeftPoint,
                        lowerTopLeftPoint, lowerTopRightPoint}));
    }
    public ArrayList<Coordinate> getComponentPoints_side_2() {
        ArrayList<Coordinate> bounds = new ArrayList<>(4);

        Coordinate CG = this.getSingleAirbrakeCG();

        Coordinate upperTopRightPoint    = new Coordinate(CG.x + thickness/2.0, CG.y + length/2.0, CG.z + width/2.0);
        Coordinate upperTopLeftPoint     = new Coordinate(CG.x + thickness/2.0, CG.y - length/2.0, CG.z + width/2.0);
        Coordinate upperBottomLeftPoint  = new Coordinate(CG.x + thickness/2.0, CG.y - length/2.0, CG.z - width/2.0);
        Coordinate upperBottomRightPoint = new Coordinate(CG.x + thickness/2.0, CG.y + length/2.0, CG.z - width/2.0);

        Coordinate lowerTopRightPoint    = new Coordinate(CG.x - thickness/2.0, CG.y + length/2.0, CG.z + width/2.0);
        Coordinate lowerTopLeftPoint     = new Coordinate(CG.x - thickness/2.0, CG.y - length/2.0, CG.z + width/2.0);
        Coordinate lowerBottomLeftPoint  = new Coordinate(CG.x - thickness/2.0, CG.y - length/2.0, CG.z - width/2.0);
        Coordinate lowerBottomRightPoint = new Coordinate(CG.x - thickness/2.0, CG.y + length/2.0, CG.z - width/2.0);

        return new ArrayList<>(List.of(
                new Coordinate[]{upperBottomLeftPoint, upperBottomRightPoint,
                        lowerBottomRightPoint, lowerBottomLeftPoint}));
    }
    public ArrayList<Coordinate> getComponentPoints_side_3() {
        ArrayList<Coordinate> bounds = new ArrayList<>(4);

        Coordinate CG = this.getSingleAirbrakeCG();

        Coordinate upperTopRightPoint    = new Coordinate(CG.x + thickness/2.0, CG.y + length/2.0, CG.z + width/2.0);
        Coordinate upperTopLeftPoint     = new Coordinate(CG.x + thickness/2.0, CG.y - length/2.0, CG.z + width/2.0);
        Coordinate upperBottomLeftPoint  = new Coordinate(CG.x + thickness/2.0, CG.y - length/2.0, CG.z - width/2.0);
        Coordinate upperBottomRightPoint = new Coordinate(CG.x + thickness/2.0, CG.y + length/2.0, CG.z - width/2.0);

        Coordinate lowerTopRightPoint    = new Coordinate(CG.x - thickness/2.0, CG.y + length/2.0, CG.z + width/2.0);
        Coordinate lowerTopLeftPoint     = new Coordinate(CG.x - thickness/2.0, CG.y - length/2.0, CG.z + width/2.0);
        Coordinate lowerBottomLeftPoint  = new Coordinate(CG.x - thickness/2.0, CG.y - length/2.0, CG.z - width/2.0);
        Coordinate lowerBottomRightPoint = new Coordinate(CG.x - thickness/2.0, CG.y + length/2.0, CG.z - width/2.0);

        return new ArrayList<>(List.of(
                new Coordinate[]{upperTopLeftPoint, upperBottomLeftPoint,
                        lowerBottomLeftPoint, lowerTopLeftPoint}));
    }
    public ArrayList<Coordinate> getComponentPoints_side_4() {
        ArrayList<Coordinate> bounds = new ArrayList<>(4);

        Coordinate CG = this.getSingleAirbrakeCG();

        Coordinate upperTopRightPoint    = new Coordinate(CG.x + thickness/2.0, CG.y + length/2.0, CG.z + width/2.0);
        Coordinate upperTopLeftPoint     = new Coordinate(CG.x + thickness/2.0, CG.y - length/2.0, CG.z + width/2.0);
        Coordinate upperBottomLeftPoint  = new Coordinate(CG.x + thickness/2.0, CG.y - length/2.0, CG.z - width/2.0);
        Coordinate upperBottomRightPoint = new Coordinate(CG.x + thickness/2.0, CG.y + length/2.0, CG.z - width/2.0);

        Coordinate lowerTopRightPoint    = new Coordinate(CG.x - thickness/2.0, CG.y + length/2.0, CG.z + width/2.0);
        Coordinate lowerTopLeftPoint     = new Coordinate(CG.x - thickness/2.0, CG.y - length/2.0, CG.z + width/2.0);
        Coordinate lowerBottomLeftPoint  = new Coordinate(CG.x - thickness/2.0, CG.y - length/2.0, CG.z - width/2.0);
        Coordinate lowerBottomRightPoint = new Coordinate(CG.x - thickness/2.0, CG.y + length/2.0, CG.z - width/2.0);

        return new ArrayList<>(List.of(
                new Coordinate[]{upperTopRightPoint, upperBottomRightPoint,
                        lowerBottomRightPoint, lowerTopRightPoint}));
    }


    public ArrayList<Coordinate> getComponentPoints() {
        ArrayList<Coordinate> bounds = new ArrayList<>(2);
        // top view: rocket at left, y horizontal, z up
        Coordinate CG = this.getSingleAirbrakeCG();
        Coordinate upperTopRightPoint    = new Coordinate(CG.x + thickness/2.0, CG.y + length/2.0, CG.z + width/2.0);
        Coordinate upperTopLeftPoint     = new Coordinate(CG.x + thickness/2.0, CG.y - length/2.0, CG.z + width/2.0);
        Coordinate upperBottomLeftPoint  = new Coordinate(CG.x + thickness/2.0, CG.y - length/2.0, CG.z - width/2.0);
        Coordinate upperBottomRightPoint = new Coordinate(CG.x + thickness/2.0, CG.y + length/2.0, CG.z - width/2.0);

        Coordinate lowerTopRightPoint    = new Coordinate(CG.x - thickness/2.0, CG.y + length/2.0, CG.z + width/2.0);
        Coordinate lowerTopLeftPoint     = new Coordinate(CG.x - thickness/2.0, CG.y - length/2.0, CG.z + width/2.0);
        Coordinate lowerBottomLeftPoint  = new Coordinate(CG.x - thickness/2.0, CG.y - length/2.0, CG.z - width/2.0);
        Coordinate lowerBottomRightPoint = new Coordinate(CG.x - thickness/2.0, CG.y + length/2.0, CG.z - width/2.0);

        return new ArrayList<>(List.of(
                new Coordinate[]{upperTopRightPoint, upperTopLeftPoint,
                        upperBottomLeftPoint, upperBottomRightPoint,
                        lowerTopRightPoint,lowerTopLeftPoint,
                        lowerBottomLeftPoint,lowerBottomRightPoint}));
    }

    @Override
    public double getLength() {
        return length;
    }

    public void setLength(double length) {
        if (length < 0) {
            System.out.println("AirbrakeSet: length must be positive");
            return;
        }
        if (this.parent != null) {
            double radiusOfBodyTube = ((BodyTube) this.parent).getOuterRadius();
            if (length > radiusOfBodyTube) {
                System.out.println("AirbrakeSet: length is too long for current CG offset and body tube radius");
                System.out.println("AirbrakeSet: patching");
                double prevFe = this.getFracExposed();
                length = radiusOfBodyTube;
                CG_offset = length/2.0;
                this.setFracExposed(prevFe);
            }
        }
        this.length = length;
        fireComponentChangeEvent(ComponentChangeEvent.AEROMASS_CHANGE);
    }

    public double getWidth() {
        return width;
    }

    public void setWidth(double width) {
        if (width < 0) {
            System.out.println("AirbrakeSet: width must be positive");
            return;
        }
        this.width = width;
        fireComponentChangeEvent(ComponentChangeEvent.AEROMASS_CHANGE);
    }

    @Override
    public double getComponentMass() {
        return indivAirbrakeMass*numAirbrakes;
    }

    public double getThickness() {
        return thickness;
    }

    public void setThickness(double thickness) {
        if (thickness < 0) {
            System.out.println("AirbrakeSet: thickness must be positive");
            return;
        }
        this.thickness = thickness;
        fireComponentChangeEvent(ComponentChangeEvent.AEROMASS_CHANGE);
    }

    public double getCG_offset() {
        return CG_offset;
    }

    public void setCG_offset(double CG_offset) {
        if (CG_offset < length/2.0) {
            System.out.println("AirbrakeSet: CG offset must be greater than half the length");
            return;
        }
        this.CG_offset = CG_offset;
        fireComponentChangeEvent(ComponentChangeEvent.AEROMASS_CHANGE);
    }


    /**
     * Calculates the fraction of the airbrake that is exposed beyond the body tube.
     *
     * @return The fraction of the airbrake length that is exposed
     */
    public double getFracExposed() {
        double radiusOfBodyTube = ((BodyTube) this.parent).getOuterRadius();
        double exposedLength = CG_offset + length/2.0 - radiusOfBodyTube;
        return exposedLength/length;
    }


    // lol
    public void setFracExposed_fudged_for_simulation(double fracExposed) {
        this.setFracExposed(fudgefactor*fracExposed);
    }

    // lol
    public double getFracExposed_fudged_for_simulation() {
        return this.getFracExposed()*fudgefactor;
    }


    public void setFracExposed(double fracExposed) {
        if (fracExposed < 0 || fracExposed > 1) {
            System.out.println("AirbrakeSet: fraction exposed must be between 0 and 1, received " + fracExposed);
            if (fracExposed -1 > 0) {
                this.setFracExposed(1);
            } else {
                this.setFracExposed(0);
            }
            return;
        }
        double radiusOfBodyTube = ((BodyTube) this.parent).getOuterRadius();
        double exposedLength = fracExposed * length;
        this.CG_offset = exposedLength - length/2.0 + radiusOfBodyTube;
        fireComponentChangeEvent(ComponentChangeEvent.AEROMASS_CHANGE);
    }


    public double getIndivAirbrakeMass() {
        return indivAirbrakeMass;
    }

    public void setIndivAirbrakeMass(double mass) {
        if (mass < 0) {
            System.out.println("AirbrakeSet: mass must be positive");
            return;
        }
        this.indivAirbrakeMass = mass;
        this.setOverrideMass(mass*numAirbrakes);
        this.setMassOverridden(true);
        fireComponentChangeEvent(ComponentChangeEvent.AEROMASS_CHANGE);
    }

    public int getNumAirbrakes() {
        return numAirbrakes;
    }

    public void setNumAirbrakes(int numAirbrakes) {
        if (numAirbrakes != 4) {
            System.out.println("AirbrakeSet: number of airbrakes must be 4");
            this.numAirbrakes = 4;
            return;
        }
        this.numAirbrakes = numAirbrakes;
        this.setOverrideMass(indivAirbrakeMass *numAirbrakes);
        this.setMassOverridden(true);
        fireComponentChangeEvent(ComponentChangeEvent.AEROMASS_CHANGE);
    }

    /**
     * Calculates the exposed area of the airbrake.
     *
     * @return The exposed area of the airbrake
     */
    public double getExposedArea() {
        return length * width * this.getFracExposed();
    }

    /**
     * Calculates the density of the airbrake material based on mass and volume.
     *
     * @return The density of the airbrake material
     */
    public double getDensity() {
        return indivAirbrakeMass/length/width/thickness;
    }

    /**
     * Creates and returns a clone of this AirbrakeSet with identical properties.
     *
     * @return A new AirbrakeSet instance with the same properties
     */
    public AirbrakeSet clone() {
        return new AirbrakeSet(length, width, thickness, CG_offset, indivAirbrakeMass, numAirbrakes);
    }

    public double getLocalOverrideCD() {
        return localOverrideCD;
    }

    public void setLocalOverrideCD(double localOverrideCD) {
        this.localOverrideCD = localOverrideCD;
        overrideCD = localOverrideCD;
        fireComponentChangeEvent(ComponentChangeEvent.AEROMASS_CHANGE);
    }

    @Override
    public BoundingBox getInstanceBoundingBox() {
        Coordinate CG = this.getSingleAirbrakeCG();

        Coordinate upperTopRightPoint    = new Coordinate(CG.x + thickness/2.0, CG.y + length/2.0, CG.z + width/2.0);
        Coordinate upperTopLeftPoint     = new Coordinate(CG.x + thickness/2.0, CG.y - length/2.0, CG.z + width/2.0);
        Coordinate upperBottomLeftPoint  = new Coordinate(CG.x + thickness/2.0, CG.y - length/2.0, CG.z - width/2.0);
        Coordinate upperBottomRightPoint = new Coordinate(CG.x + thickness/2.0, CG.y + length/2.0, CG.z - width/2.0);

        Coordinate lowerTopRightPoint    = new Coordinate(CG.x - thickness/2.0, CG.y + length/2.0, CG.z + width/2.0);
        Coordinate lowerTopLeftPoint     = new Coordinate(CG.x - thickness/2.0, CG.y - length/2.0, CG.z + width/2.0);
        Coordinate lowerBottomLeftPoint  = new Coordinate(CG.x - thickness/2.0, CG.y - length/2.0, CG.z - width/2.0);
        Coordinate lowerBottomRightPoint = new Coordinate(CG.x - thickness/2.0, CG.y + length/2.0, CG.z - width/2.0);

        return new BoundingBox(upperTopLeftPoint,lowerBottomRightPoint);
    }
}
