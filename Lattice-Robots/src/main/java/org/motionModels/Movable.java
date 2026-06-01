package org.motionModels;

import org.graphs.OrientedPoint;

/**
 * An interface defining the required method for a moveable robot
 */
public interface Movable {
    
    /**
     * Moves the robot according to its motion model for a given time duration
     * @param dt the duration of the robot movement
     */
    public void move(double dt);

    /**
     * Moves the robot according to its motion model towards a specific target position for a given amount of time
     * @param target the target position of the robot
     * @param dt the duration of the robot movement
     * @return true if the robot has reached its target position, otherwise false
     */
    public boolean moveTo(OrientedPoint target, double dt);

    /**
     * Initiates the robot's movement by triggering the starting mechanism for the motion model of the robot
     */
    public void startMoving();

    /**
     * Provides the distance traveled by the robot according to its motion model
     * @return the distance traveled by the robot
     */
    public double getDistanceTraveled();
}
