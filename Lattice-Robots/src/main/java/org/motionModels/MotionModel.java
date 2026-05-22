package org.motionModels;

import org.graphs.OrientedPoint;

public abstract class MotionModel {

    protected double distTraveled;

    public abstract boolean moveTo(OrientedPoint currentPose, OrientedPoint target, double dt);

    public abstract boolean move(OrientedPoint currentPose, double dt);

    public abstract void startMoving();

    public double getDistanceTraveled() {
        return distTraveled;
    } 
}
