package org.motionModels;

import org.graphs.OrientedPoint;

public interface Movable {
    
    public void move(double dt);

    public boolean moveTo(OrientedPoint target, double dt);

    public void startMoving();

    public double getDistanceTraveled();
}
