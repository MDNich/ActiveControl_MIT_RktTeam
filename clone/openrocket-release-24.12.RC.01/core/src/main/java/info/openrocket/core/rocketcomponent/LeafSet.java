package info.openrocket.core.rocketcomponent;

import info.openrocket.core.rocketcomponent.position.AxialMethod;
import info.openrocket.core.rocketcomponent.position.AxialPositionable;
import info.openrocket.core.util.Coordinate;

import java.util.Collection;
import java.util.List;



/**
 * The LeafSet class represents a specific type of external rocket component
 * with aerodynamic and mass-related properties. It inherits from the
 * ExternalComponent class, ensuring it possesses physical appearance
 * and aerodynamic simulation effects.
 *
 * This abstract class provides foundational properties and behavior
 * specific to a "LeafSet" component in the larger context of rocket
 * modeling applications.
 *
 * Key Features:
 * - Inherits aerodynamic and mass-related calculations from ExternalComponent.
 * - Overrides the component name to return "LeafSet".
 *
 * Users of this class should extend it and provide implementations for other
 * abstract methods defined in ExternalComponent where required.
 */
public abstract class LeafSet extends ExternalComponent implements AxialPositionable, BoxBounded {
    /**
     * Constructor that sets the relative position of the component.
     *
     * @param relativePosition
     */
    public LeafSet(AxialMethod relativePosition) {
        super(relativePosition);
    }

}
