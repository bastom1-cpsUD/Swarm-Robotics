package org.graphs.voltage;

import java.util.List;
import java.util.Map;

import org.graphs.util.RigidBodyTransformation;

/**
 * The finite quotient graph standing in for an infinite, periodic lattice: a
 * handful of Roles and HalfEdges, each edge labeled with the rigid-body
 * transform ("voltage") it represents. See DCEL-Implementation-Plan.md sec 2-3.
 * Built only by VoltageGraphBuilder, which resolves getTwin()/getNext()/getFace()
 * once instead of leaving them to be inferred at lookup time.
 */
public final class VoltageGraph {
    public static final double DEFAULT_EPSILON = 1e-6;

    private final List<Role> roles;
    private final Map<Integer, HalfEdge> halfEdgesById;
    private final Map<Role, List<HalfEdge>> outgoing;
    private final List<Face> faces;
    private final Role primaryRole;
    private final int maxCycleLength;

    VoltageGraph(List<Role> roles, Map<Integer, HalfEdge> halfEdgesById,
                 Map<Role, List<HalfEdge>> outgoing, List<Face> faces, Role primaryRole) {
        this.roles = roles;
        this.halfEdgesById = halfEdgesById;
        this.outgoing = outgoing;
        this.faces = faces;
        this.primaryRole = primaryRole;

        // Cached rather than derived per call: faces are fixed at build time, and Phase
        // 5's hop cap consults this on every relay.
        int longest = 0;
        for (Face face : faces) {
            longest = Math.max(longest, face.getCycleLength());
        }
        this.maxCycleLength = longest;
    }

    public List<Role> getRoles() {
        return roles;
    }

    public Role getPrimaryRole() {
        return primaryRole;
    }

    public List<HalfEdge> getOutgoingHalfEdges(Role role) {
        return outgoing.getOrDefault(role, List.of());
    }

    public HalfEdge getHalfEdgeById(int id) {
        return halfEdgesById.get(id);
    }

    public List<Face> getFaces() {
        return faces;
    }

    public HalfEdge getTwin(HalfEdge h) {
        return h.getTwin();
    }

    public HalfEdge getNext(HalfEdge h) {
        return h.getNext();
    }

    /**
     * Checks that a walk's accumulated voltage returns to identity and its
     * length matches a known face's cycle length. Length alone is not enough --
     * a walk can revisit half-edges (e.g. octagon-square, where a role has
     * fewer outgoing edges than the faces it borders are long) without closing.
     * See DCEL-Implementation-Plan.md sec 2.3 / primer sec 6-7.
     */
    public boolean validateCycle(List<HalfEdge> walk) {
        return validateCycle(walk, DEFAULT_EPSILON);
    }

    public boolean validateCycle(List<HalfEdge> walk, double epsilon) {
        if (walk.isEmpty()) {
            return false;
        }

        RigidBodyTransformation holonomy = RigidBodyTransformation.identity();
        for (HalfEdge h : walk) {
            holonomy = holonomy.compose(h.getVoltage());
        }
        return holonomy.isApproximatelyIdentity(epsilon) && isFaceCycleLength(walk.size());
    }

    /**
     * Checks a walk against separate translation and rotation tolerances, rather than
     * the single conflated scalar {@link #validateCycle(List, double)} uses. See
     * {@link RigidBodyTransformation#isApproximatelyIdentity(double, double)} for why
     * one scalar cannot serve both.
     *
     * <p>Composition is left-to-right in walk order, matching the order a face
     * certificate accumulates its hops as it travels. Reversing the walk generally does
     * NOT yield the identity -- these transforms do not commute -- so a certificate that
     * accumulates in the wrong order will fail here, which is the intended behaviour.
     *
     * @param walk the half-edges traversed, in order
     * @param positionEpsilon translation tolerance in lattice units
     * @param angleEpsilon rotation tolerance in radians
     * @return true if the walk closes and its length matches some face's cycle length
     */
    public boolean validateCycle(List<HalfEdge> walk, double positionEpsilon, double angleEpsilon) {
        if (walk.isEmpty()) {
            return false;
        }

        RigidBodyTransformation holonomy = RigidBodyTransformation.identity();
        for (HalfEdge h : walk) {
            holonomy = holonomy.compose(h.getVoltage());
        }

        return holonomy.isApproximatelyIdentity(positionEpsilon, angleEpsilon)
                && isFaceCycleLength(walk.size());
    }

    /**
     * Whether any face in this graph is bounded by exactly this many half-edges. The
     * length half of the closure test: a walk whose accumulated voltage returns to
     * identity but whose length matches no face has wrapped some face more than once,
     * or traced something that is not a face at all.
     *
     * @param length a candidate walk length
     * @return true if some face has this cycle length
     */
    public boolean isFaceCycleLength(int length) {
        for (Face face : faces) {
            if (face.getCycleLength() == length) {
                return true;
            }
        }
        return false;
    }

    /**
     * The longest face boundary in this graph -- 12 on the dodecagonal lattices, 4 on
     * the square. Bounds how far a face certificate may legitimately travel: a walk that
     * has taken more hops than this cannot still be tracing a single face, so relayers
     * use it to cut off a wandering certificate rather than forwarding it forever.
     *
     * @return the maximum cycle length over all faces
     */
    public int maxCycleLength() {
        return maxCycleLength;
    }
}
