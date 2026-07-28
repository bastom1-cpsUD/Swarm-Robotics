package org.graphs.voltage;

import org.graphs.util.OrientedPoint;

/**
 * One position type in a repeating lattice pattern. Every physical vertex in
 * the infinite lattice is a copy of some Role; see DCEL-Implementation-Plan.md sec 3.
 */
public final class Role {
    private final int id;
    private final OrientedPoint pose;

    public Role(int id, OrientedPoint pose) {
        this.id = id;
        this.pose = pose;
    }

    public int getId() {
        return id;
    }

    public OrientedPoint getPose() {
        return pose;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (!(o instanceof Role)) {
            return false;
        }

        Role other = (Role) o;
        return this.id == other.id && this.pose.equals(other.pose);
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(id, pose);
    }

    @Override
    public String toString() {
        return "Role[id: " + id + " " + pose + "]";
    }
}
