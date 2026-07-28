package org.graphs.lattice;

import org.graphs.util.OrientedPoint;

public class Vertex {
    /**
     * The unique identifier for this vertex, which can be used to distinguish it from other vertices within the graph.
     */
    private final int ID;
    /**
     * The relative position and orientation of this vertex within the lattice graph.
     */
    private final OrientedPoint POSE_IN_GRAPH;

    /**
     * Constructs a vertex with a specified ID and pose within the lattice graph.
     * @param id the unique identifier for this vertex
     * @param pose the relative position and orientation of this vertex within the lattice graph
     */
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

    @Override
    public boolean equals(Object o) {
        if(this == o) {
            return true;
        }

        if(!(o instanceof Vertex)) {
            return false;
        }

         Vertex other = (Vertex) o;
        return this.ID == other.getId() && this.POSE_IN_GRAPH.equals(other.getPose());
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(this.ID, this.POSE_IN_GRAPH);
    }

    @Override
    public String toString() {
        return "Vertex[ID: " + this.ID + " " + this.POSE_IN_GRAPH + "]";
    }
}
