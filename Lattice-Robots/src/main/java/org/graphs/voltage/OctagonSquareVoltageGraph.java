package org.graphs.voltage;

import org.graphs.util.OrientedPoint;

/**
 * The octagon-square (4.8.8) lattice as a VoltageGraph: four roles, one per
 * rotation state (0/90/180/270 degrees) a copy of the tiling's single
 * vertex-transitive vertex can appear in, connected entirely by PURE
 * TRANSLATIONS -- no edge carries any rotation, and no semi-edge is needed.
 *
 * A single-role, rotation-carrying-edge construction is mathematically valid
 * (the tiling is vertex-transitive under its full symmetry group, rotations
 * included) but depends on every consumer correctly composing rotations
 * along a chain of assignments to recover a role's true orientation --
 * exactly the class of bug CyclebuilderComms.getAssignedGlobalPosition() had
 * (see git history). Four roles side-steps that dependency entirely, the
 * same way HexagonVoltageGraph's two roles avoid ever needing a rotation:
 * every edge here is a plain translate, so a robot's orientation never has
 * to accumulate anything for the formation to be geometrically correct.
 *
 * Derivation: label role k's local frame as rotated dtheta = 90k degrees
 * from a reference. Every role has the same three canonical local directions
 * (45, 135, 270 degrees, per the turning-angle argument in the primer/
 * DCEL-Implementation-Plan.md sec 5, 7-8): departing at world angle
 * (canonical + 90k) leads to role (k+1) mod 4 via the 45-degree edge, role
 * (k-1) mod 4 via the 135-degree edge, and role (k+2) mod 4 via the
 * 270-degree edge. Independently verified (script, not by hand) that every
 * resulting directed edge's twin is the target role's edge with the exact
 * negated translation, and that the discovered faces are a 4-cycle (square)
 * and an 8-cycle (octagon), each with identity holonomy.
 */
public final class OctagonSquareVoltageGraph {
    private static final double EDGE_LENGTH = 50.0;

    private OctagonSquareVoltageGraph() {
    }

    public static VoltageGraph build() {
        VoltageGraphBuilder builder = new VoltageGraphBuilder();

        Role r0 = builder.addRole(0, new OrientedPoint(0, 0, 0));
        Role r1 = builder.addRole(1, new OrientedPoint(0, 0, 0));
        Role r2 = builder.addRole(2, new OrientedPoint(0, 0, 0));
        Role r3 = builder.addRole(3, new OrientedPoint(0, 0, 0));
        builder.setPrimaryRole(r0);

        HalfEdge r0A = builder.addHalfEdgePair(r0, r1, translateAt(45));   // twin: r1C
        HalfEdge r0C = builder.addHalfEdgePair(r0, r3, translateAt(135));  // twin: r3A
        HalfEdge r0B = builder.addHalfEdgePair(r0, r2, translateAt(270)); // twin: r2B
        HalfEdge r1A = builder.addHalfEdgePair(r1, r2, translateAt(135)); // twin: r2C
        HalfEdge r1B = builder.addHalfEdgePair(r1, r3, translateAt(0));   // twin: r3B
        HalfEdge r2A = builder.addHalfEdgePair(r2, r3, translateAt(225)); // twin: r3C

        HalfEdge r1C = r0A.getTwin();
        HalfEdge r3A = r0C.getTwin();
        HalfEdge r2B = r0B.getTwin();
        HalfEdge r2C = r1A.getTwin();
        HalfEdge r3B = r1B.getTwin();
        HalfEdge r3C = r2A.getTwin();

        // Rotation order at each role, sorted by real world departure angle.
        builder.setRotationOrder(r0, r0A, r0C, r0B);
        builder.setRotationOrder(r1, r1B, r1A, r1C);
        builder.setRotationOrder(r2, r2B, r2A, r2C);
        builder.setRotationOrder(r3, r3C, r3B, r3A);

        return builder.build();
    }

    private static OrientedPoint translateAt(double degrees) {
        double radians = Math.toRadians(degrees);
        return new OrientedPoint(EDGE_LENGTH * Math.cos(radians), EDGE_LENGTH * Math.sin(radians), 0);
    }
}
