package org.graphs;

public class LatticeEdge {
    private Vertex from;
    private Vertex to;
    private OrientedPoint toPos;
    private RigidBodyTransformation transformation;
    public LatticeEdge(Vertex from, Vertex to, OrientedPoint pose) {
        this.from = from;
        this.to = to;
        this.toPos = pose;
        this.transformation = new RigidBodyTransformation(from.getPose(), toPos);
    }

    public Vertex getFrom() {
        return from;
    }

    public Vertex getTo() {
        return to;
    }

    public RigidBodyTransformation getEdgeTransformation() {
        return transformation;
    }

    public RigidBodyTransformation getInverseEdgeTransformation() {
        return transformation.inverse();
    }
}
