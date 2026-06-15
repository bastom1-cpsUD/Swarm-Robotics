package org.communicationModels;

import org.robots.LatticeRobot;
import org.graphs.OrientedPoint;
import org.graphs.RigidBodyTransformation;

public class Observation {
    private int id;
    private OrientedPoint position;

    public Observation(LatticeRobot neighbor, RigidBodyTransformation globalToLocal) {
        this.id = neighbor.getRobotId();
        this.position = globalToLocal.apply(neighbor.getPosition());
    }

    public int getId() {
        return id;
    }

    public OrientedPoint getLocalPosition() {
        return position;
    }
}