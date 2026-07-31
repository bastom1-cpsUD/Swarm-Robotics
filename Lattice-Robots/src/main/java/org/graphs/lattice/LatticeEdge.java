package org.graphs.lattice;

import org.graphs.util.*;
/**
 * A class representing a directed edge in a lattice graph, connecting two vertices and labeled with a rigid body transformation.
 */
public class LatticeEdge {
    /**
     * The source vertex of the edge and the starting point for the transformation.
     */
    private Vertex from;
    /**
     * The target vertex of the edge and the endpoint of the transformation
     */
    private Vertex to;
    /**
     * The position and orientation of the target vertex relative to the source vertex, which is used to compute the edge's rigid body transformation.
     */
    private OrientedPoint toPos;
    /**
     * The rigid body transformation associated with this lattice edge.
     */
    private RigidBodyTransformation transformation;
    /**
     * A flag indicating whether this edge is a null edge.
     */
    private boolean isNull;

    private int Id;

    /**
     * Constructs a lattice edge from a source vertex to a target vertex with a specified pose, and computes the corresponding rigid body transformation.
     * @param from the source vertex of the edge
     * @param to the target vertex of the edge
     * @param pose the pose of the target vertex relative to the source vertex, used to compute the edge's rigid body transformation
     */
    public LatticeEdge(int Id, Vertex from, Vertex to, OrientedPoint pose) {
        this.Id = Id;
        this.from = from;
        this.to = to;
        this.toPos = pose;
        // Single-arg, not (from.getPose(), toPos): `pose` is already the target's pose
        // relative to `from`, so the two-arg constructor would subtract `from`'s pose a
        // second time. That was visibly wrong for HexagonLattice, whose v1 sits at
        // (25, 43.3) -- e5 ("Right", offset (50, 0)) was becoming (25, -43.3).
        this.transformation = new RigidBodyTransformation(toPos);
        this.isNull = false;
    }

    /**
     * The general constructor for a null edge, which can be used as a placeholder to represent the absence of a valid edge in certain contexts.
     */
    public LatticeEdge() {
        this.from = null;
        this.to = null;
        this.toPos = null;
        this.transformation = null;
        this.isNull = true;
    }

    public int getId() {
        return Id;
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

    @Override
    public String toString() {
        return "LatticeEdge[ID: " + Id + " From: " + this.from + " To: " + this.to + " Pose: " + this.toPos + " Null: " + isNull + "]";
    }

    @Override
    public boolean equals(Object o) {
        if(this == o) {
            return true;
        } else if(!(o instanceof LatticeEdge)) {
            return false;
        }

        LatticeEdge other = (LatticeEdge) o;

        return this.getId() == other.getId() 
            && this.getFrom().equals(other.getFrom())
            && this.getTo().equals(other.getTo())
            && this.getToPos().equals(other.getToPos())
            && this.isNull() == other.isNull();
    }
}
