package org.graphs;

public class LatticeEdge {
    private OrientedPoint from;
    private OrientedPoint to;
    private RigidBodyTransformation transformation;
    public LatticeEdge(OrientedPoint from, OrientedPoint to) {
        this.from = from;
        this.to = to;
        this.transformation = new RigidBodyTransformation(from, to);
    }

    public OrientedPoint getFrom() {
        return from;
    }

    public OrientedPoint getTo() {
        return to;
    }

    public RigidBodyTransformation getEdgeTransformation() {
        return transformation;
    }

    public RigidBodyTransformation getInverseEdgeTransformation() {
        return transformation.inverse();
    }
}
