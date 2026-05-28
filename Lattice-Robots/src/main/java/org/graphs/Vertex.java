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

    @Override
    public boolean equals(Object o) {
        if(this == o) {
            return true;
        }

        if(!(o instanceof Vertex)) {
            return false;
        }

         Vertex other = (Vertex) o;
        return this.ID == other.getId() && this.POSE_IN_GRAPH == other.getPose();
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
