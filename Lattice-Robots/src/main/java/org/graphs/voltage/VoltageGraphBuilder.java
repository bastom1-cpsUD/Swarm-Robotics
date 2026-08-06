package org.graphs.voltage;

import org.graphs.util.OrientedPoint;
import org.graphs.util.RigidBodyTransformation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Declares a lattice's roles and half-edges, then resolves twin/next/face
 * pointers once, at construction time. See DCEL-Implementation-Plan.md sec 3.
 *
 * addHalfEdgePair fixes which half-edges are twins; it does not by itself fix
 * rotation order, since a role that is both endpoints of one of its own edges
 * (e.g. a single-role square lattice) has its half-edge and that half-edge's
 * twin pointing in opposite directions -- never adjacent in true rotation
 * order. Such roles need setRotationOrder called explicitly; a role that only
 * ever appears at one end of its edges (e.g. HexagonLattice's two roles) can
 * rely on the default (insertion order), which already matches.
 */
public final class VoltageGraphBuilder {
    private final Map<Integer, Role> rolesById = new LinkedHashMap<>();
    private final Map<Role, List<HalfEdge>> insertionOrder = new HashMap<>();
    private final Map<Role, List<HalfEdge>> explicitOrder = new HashMap<>();
    private final List<HalfEdge> allHalfEdges = new ArrayList<>();
    private Role primaryRole;
    private int nextHalfEdgeId = 0;

    public Role addRole(int id, OrientedPoint pose) {
        Role role = new Role(id, pose);
        rolesById.put(id, role);
        insertionOrder.put(role, new ArrayList<>());
        return role;
    }

    public VoltageGraphBuilder setPrimaryRole(Role role) {
        this.primaryRole = role;
        return this;
    }

    /**
     * Registers one physical connection as a pair of half-edges: h, running
     * from `from` to `to`, and its twin, running back. toPoseRelativeToFrom is
     * `to`'s pose measured relative to `from` (org.graphs.LatticeEdge's
     * convention). Returns h; the twin is h.getTwin().
     */
    public HalfEdge addHalfEdgePair(Role from, Role to, OrientedPoint toPoseRelativeToFrom) {
        // Single-arg, not (from.getPose(), toPoseRelativeToFrom): the argument is
        // already expressed in `from`'s frame, so the two-arg constructor would
        // subtract `from`'s pose a second time. Identical for every lattice declared
        // so far -- every role that originates an edge sits at the origin -- but
        // wrong the moment one does not.
        RigidBodyTransformation voltage = new RigidBodyTransformation(toPoseRelativeToFrom);

        HalfEdge h = new HalfEdge(nextHalfEdgeId++, from, voltage);
        HalfEdge twin = new HalfEdge(nextHalfEdgeId++, to, voltage.inverse());
        h.setTwin(twin);
        twin.setTwin(h);

        insertionOrder.get(from).add(h);
        insertionOrder.get(to).add(twin);
        allHalfEdges.add(h);
        allHalfEdges.add(twin);

        return h;
    }

    /**
     * Overrides the default (insertion-order) rotation order for `role`. Must
     * list every half-edge originating at `role`, exactly once.
     */
    public VoltageGraphBuilder setRotationOrder(Role role, HalfEdge... outgoingInOrder) {
        explicitOrder.put(role, List.of(outgoingInOrder));
        return this;
    }

    public VoltageGraph build() {
        if (primaryRole == null) {
            throw new IllegalStateException("VoltageGraphBuilder.build(): no primary role set");
        }

        Map<Role, List<HalfEdge>> rotationOrder = new LinkedHashMap<>();
        for (Role role : rolesById.values()) {
            List<HalfEdge> order = explicitOrder.getOrDefault(role, insertionOrder.get(role));
            validateRotationOrder(role, order);
            rotationOrder.put(role, order);
        }

        resolveNextAndPrev(rotationOrder);
        List<Face> faces = discoverFaces();

        Map<Integer, HalfEdge> halfEdgesById = new HashMap<>();
        for (HalfEdge h : allHalfEdges) {
            halfEdgesById.put(h.getId(), h);
        }

        return new VoltageGraph(new ArrayList<>(rolesById.values()), halfEdgesById,
                rotationOrder, faces, primaryRole);
    }

    private void validateRotationOrder(Role role, List<HalfEdge> order) {
        List<HalfEdge> expected = insertionOrder.get(role);
        if (order.size() != expected.size() || !order.containsAll(expected)) {
            throw new IllegalStateException(
                "VoltageGraphBuilder.build(): rotation order for role " + role.getId()
                + " must contain exactly the " + expected.size()
                + " half-edge(s) originating there, each once");
        }
    }

    private void resolveNextAndPrev(Map<Role, List<HalfEdge>> rotationOrder) {
        for (HalfEdge h : allHalfEdges) {
            HalfEdge twin = h.getTwin();
            List<HalfEdge> targetOrder = rotationOrder.get(twin.getOrigin());
            int i = targetOrder.indexOf(twin);
            int n = targetOrder.size();
            // next(h) = the edge immediately clockwise from twin(h): the previous
            // entry in twin(h)'s CCW rotation order. Edmonds' rule -- see
            // DCEL-Implementation-Plan.md sec 2.2.
            HalfEdge next = targetOrder.get(Math.floorMod(i - 1, n));
            h.setNext(next);
        }
        for (HalfEdge h : allHalfEdges) {
            h.getNext().setPrev(h);
        }
    }

    // A safety cap on how many times a label orbit may repeat while its
    // holonomy is checked -- generous relative to the rotation orders (up to
    // 6-fold) any real wallpaper group symmetry can contribute.
    private static final int MAX_LAPS = 24;

    private List<Face> discoverFaces() {
        List<Face> faces = new ArrayList<>();
        Set<HalfEdge> visited = new HashSet<>();
        int faceId = 0;

        for (HalfEdge start : allHalfEdges) {
            if (visited.contains(start)) {
                continue;
            }

            // The label orbit -- following next() back to the starting
            // half-edge object. A role can have fewer outgoing edges than the
            // faces it borders are long (e.g. octagon-square's single role,
            // where the square face repeats one label four times), so the
            // label orbit closing is necessary but not sufficient: the
            // physical face closes only once the ACCUMULATED voltage over
            // some number of laps of this orbit reaches identity. See
            // DCEL-Implementation-Plan.md sec 2.3 / primer sec 7.
            List<HalfEdge> labelOrbit = new ArrayList<>();
            HalfEdge h = start;
            do {
                labelOrbit.add(h);
                h = h.getNext();
            } while (h != start);

            RigidBodyTransformation holonomy = RigidBodyTransformation.identity();
            int totalSteps = 0;
            int laps = 0;
            do {
                for (HalfEdge e : labelOrbit) {
                    holonomy = holonomy.compose(e.getVoltage());
                    totalSteps++;
                }
                laps++;
            } while (!holonomy.isApproximatelyIdentity(VoltageGraph.DEFAULT_EPSILON) && laps < MAX_LAPS);

            if (!holonomy.isApproximatelyIdentity(VoltageGraph.DEFAULT_EPSILON)) {
                throw new IllegalStateException(
                    "VoltageGraphBuilder.build(): face starting at half-edge "
                    + start.getId() + " does not close within " + MAX_LAPS
                    + " lap(s) of its label orbit. Check the declared poses and rotation order.");
            }

            visited.addAll(labelOrbit);
            Face face = new Face(faceId++, start, totalSteps, holonomy);
            for (HalfEdge e : labelOrbit) {
                e.setFace(face);
            }
            faces.add(face);
        }

        return faces;
    }
}
