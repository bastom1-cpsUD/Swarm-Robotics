package org.communicationModels;


import org.robots.Robot;
import org.graphs.util.OrientedPoint;
import org.graphs.util.RigidBodyTransformation;

/**
 * One neighbouring robot as seen by an observer, expressed in the observer's own
 * local frame.
 *
 * <p>The stored pose is a full pose, not just a location: {@link #getLocalOrientation()}
 * is the neighbour's heading relative to the observer's. That was not true before
 * {@code RigidBodyTransformation.apply()} was fixed to compose the input pose's own
 * orientation -- every Observation used to report the observing transform's rotation
 * instead, so the value was identical for every neighbour and carried no information.
 */
public class Observation {
    private int id;
    private OrientedPoint position;

    public Observation(Robot neighbor, RigidBodyTransformation globalToLocal) {
        this.id = neighbor.getRobotId();
        this.position = globalToLocal.apply(neighbor.getPosition());
    }

    public int getId() {
        return id;
    }

    /**
     * The neighbour's pose in the observer's local frame.
     * @return the local pose, including the neighbour's relative heading
     */
    public OrientedPoint getLocalPosition() {
        return position;
    }

    /**
     * The neighbour's heading in the observer's local frame, in radians, normalized to
     * (-pi, pi]. Zero means the neighbour faces the same way the observer does.
     * @return the neighbour's relative heading
     */
    public double getLocalOrientation() {
        return position.getOrientation();
    }
}
