package org.graphs;

public class Vertex {
    private final int ID;
    private final OrientedPoint POSE_IN_GRAPH;

    public Vertex(int id, OrientedPoint pose) {
        this.ID = id;
        this.POSE_IN_GRAPH= pose;
    }

    public OrientedPoint getPose() {
        return POSE_IN_GRAPH;
    }

    public int getId() {
        return ID;
    }
}
