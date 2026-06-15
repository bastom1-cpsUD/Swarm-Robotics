package org.motionModels;

import org.graphs.OrientedPoint;

public abstract class MotionModel {
    /**
     * The distance traveled for the robot within the motion model
     */
    protected double distTraveled;

    /**
     * Moves the robot towards a provided target destination for a given time step.
     * @param currentPose the current position of the robot
     * @param target the target destination of the robot
     * @param dt the duration of the robot movement
     * @return true if the robot reached its target destination, false otherwise
     */
    public abstract boolean moveTo(OrientedPoint currentPose, OrientedPoint target, double dt);
    /** 
     *Moves the robot according to the motion model for a given time step.
     *  @Param currentPose the current position of the robot
     *  @Param dt the duration of the robot movement
     *  @return true if the robot has completed its movement, false otherwise
    */
    public abstract boolean move(OrientedPoint currentPose, double dt);

    public double getDistanceTraveled() {
        return distTraveled;
    } 
}
