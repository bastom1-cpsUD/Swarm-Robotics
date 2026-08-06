package org.motionModels; 

import org.graphs.util.OrientedPoint;
/**
 * An interface defining the additonal methods required for the motion model of a lattice robot.
 */
public interface LatticeMotionModel {
    /**
     * Calculate the intermediate pose for a robot moving from its current position to a target position, staying within communication range of its parent at the given time step.
     * @param currentPose the current position of the robot
     * @param parentPose the current position of the parent 
     * @param target the target destination assigned by the parent robot
     * @param timeStep the duration of the time step for which the robot will be moving towards the target
     * @return the intermediate position for the robot to move to towards the target while maintaining communication with its parent, returning the target if it is within communication range
     */
    OrientedPoint getIntermediatePose(OrientedPoint currentPose, OrientedPoint parentPose, OrientedPoint target, double timeStep);

    /**
     * Moves the robot towards a provided target destination for a given time step, while maintaining communication with its parent robot.
     * @param currentPose the current position of the robot
     * @param newPose the target destination of the robot
     * @param dt the duration of the robot movement
     * @return true if the robot reached its target destination, false otherwise
     */
    boolean moveTo(OrientedPoint currentPose, OrientedPoint newPose, double dt);

    public double getMaxSpeed();
    
}