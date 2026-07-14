package org.graphs.voltage;

import org.graphs.RigidBodyTransformation;

import java.util.List;
import java.util.Map;

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

    VoltageGraph(List<Role> roles, Map<Integer, HalfEdge> halfEdgesById,
                 Map<Role, List<HalfEdge>> outgoing, List<Face> faces, Role primaryRole) {
        this.roles = roles;
        this.halfEdgesById = halfEdgesById;
        this.outgoing = outgoing;
        this.faces = faces;
        this.primaryRole = primaryRole;
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

        RigidBodyTransformation holonomy = new RigidBodyTransformation();
        for (HalfEdge h : walk) {
            holonomy = holonomy.compose(h.getVoltage());
        }

        if (!holonomy.isApproximatelyIdentity(epsilon)) {
            return false;
        }

        for (Face face : faces) {
            if (face.getCycleLength() == walk.size()) {
                return true;
            }
        }
        return false;
    }
}
