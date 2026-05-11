package org.graphs;

public class LatticeEdge {
    private OrientedPoint from;
    private OrientedPoint to;
    private Transformation transformation;
    public LatticeEdge(OrientedPoint from, OrientedPoint to) {
        this.from = from;
        this.to = to;
        this.transformation = new Transformation(from, to);
    }

    public OrientedPoint getFrom() {
        return from;
    }

    public OrientedPoint getTo() {
        return to;
    }

    public Transformation getEdgeTransformation() {
        return transformation;
    }

    public Transformation getInverseEdgeTransformation() {
        return transformation.inverse();
    }
}
