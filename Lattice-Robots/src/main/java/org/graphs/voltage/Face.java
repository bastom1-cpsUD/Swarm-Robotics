package org.graphs.voltage;

import java.util.ArrayList;
import java.util.List;

import org.graphs.util.RigidBodyTransformation;

/**
 * A closed orbit of getNext() -- the loop traced by starting at any boundary
 * half-edge and following getNext() until returning to the start. Holonomy is
 * captured at construction time, once, rather than recomputed per lookup.
 * See DCEL-Implementation-Plan.md sec 2.3 / 4.
 */
public final class Face {
    private final int id;
    private final HalfEdge representative;
    private final int cycleLength;
    private final RigidBodyTransformation holonomy;

    Face(int id, HalfEdge representative, int cycleLength, RigidBodyTransformation holonomy) {
        this.id = id;
        this.representative = representative;
        this.cycleLength = cycleLength;
        this.holonomy = holonomy;
    }

    public int getId() {
        return id;
    }

    public int getCycleLength() {
        return cycleLength;
    }

    public RigidBodyTransformation getHolonomy() {
        return holonomy;
    }

    public List<HalfEdge> getBoundary() {
        List<HalfEdge> boundary = new ArrayList<>(cycleLength);
        HalfEdge h = representative;
        for (int i = 0; i < cycleLength; i++) {
            boundary.add(h);
            h = h.getNext();
        }
        return boundary;
    }

    @Override
    public String toString() {
        return "Face[id: " + id + " cycleLength: " + cycleLength + "]";
    }
}
