package org.graphs;

public class LatticeEdge {
    private Vertex from;
    private Vertex to;
    private OrientedPoint toPos;
    private RigidBodyTransformation transformation;
    private boolean isNull;
    public LatticeEdge(Vertex from, Vertex to, OrientedPoint pose) {
        this.from = from;
        this.to = to;
        this.toPos = pose;
        this.transformation = new RigidBodyTransformation(from.getPose(), toPos);
        this.isNull = false;
    }

    public LatticeEdge() {
        this.from = null;
        this.to = null;
        this.toPos = null;
        this.transformation = null;
        this.isNull = true;
    }

    public Vertex getFrom() {
        return from;
    }

    public Vertex getTo() {
        return to;
    }

    public OrientedPoint getToPos() {
        return toPos;
    }

    public RigidBodyTransformation getEdgeTransformation() {
        return transformation;
    }

    public RigidBodyTransformation getInverseEdgeTransformation() {
        return transformation.inverse();
    }

    public boolean isNull() {
        return isNull;
    }
}
