package org.graphs.voltage;

import org.graphs.RigidBodyTransformation;

/**
 * One directed half of a connection between two Role copies, labeled with the
 * rigid-body transform ("voltage") it represents. twin/next/prev/face are
 * resolved once by VoltageGraphBuilder.build(), never inferred at lookup time.
 * See DCEL-Implementation-Plan.md sec 2-3.
 */
public final class HalfEdge {
    private final int id;
    private final Role origin;
    private final RigidBodyTransformation voltage;

    private HalfEdge twin;
    private HalfEdge next;
    private HalfEdge prev;
    private Face face;

    HalfEdge(int id, Role origin, RigidBodyTransformation voltage) {
        this.id = id;
        this.origin = origin;
        this.voltage = voltage;
    }

    public int getId() {
        return id;
    }

    public Role getOrigin() {
        return origin;
    }

    public Role getTarget() {
        return twin.origin;
    }

    public RigidBodyTransformation getVoltage() {
        return voltage;
    }

    public HalfEdge getTwin() {
        return twin;
    }

    public HalfEdge getNext() {
        return next;
    }

    public HalfEdge getPrev() {
        return prev;
    }

    public Face getFace() {
        return face;
    }

    void setTwin(HalfEdge twin) {
        this.twin = twin;
    }

    void setNext(HalfEdge next) {
        this.next = next;
    }

    void setPrev(HalfEdge prev) {
        this.prev = prev;
    }

    void setFace(Face face) {
        this.face = face;
    }

    @Override
    public String toString() {
        return "HalfEdge[id: " + id + " origin: " + origin.getId() + "]";
    }
}
