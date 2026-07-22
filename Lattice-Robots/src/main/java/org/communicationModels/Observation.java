package org.communicationModels;


import org.robots.Robot;
import org.graphs.util.OrientedPoint;
import org.graphs.util.RigidBodyTransformation;

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

    public OrientedPoint getLocalPosition() {
        return position;
    }
}