package org.graphs.voltage;

import org.graphs.OrientedPoint;

/**
 * The octagon-square (4.8.8) lattice as a VoltageGraph: one vertex-transitive
 * role, degree 3. Two of its edges (A and C) are twins of each other; the
 * third (B, the octagon-octagon edge) is its own twin, a semi-edge fixed by
 * the tiling's own 180-degree symmetry about that edge's midpoint. Poses and
 * rotation order below are not guesses -- they were derived from the tiling's
 * real coordinates (octagons of side 1 on a grid of spacing 1+sqrt(2), a
 * small square in each diagonal gap) and independently confirmed against a
 * direct geometric trace of one real square and one real octagon before
 * being encoded here. See DCEL-Implementation-Plan.md sec 5 and the primer's
 * sec 7-8 for the turning-angle argument this construction satisfies:
 * the square face closes after 4 repetitions of edge A's voltage (4 x 90
 * degrees), and the octagon face closes after 4 repetitions of edges C then
 * B (4 x (90 + 180) accumulating to a net 360-degree turn over 8 steps).
 */
public final class OctagonSquareVoltageGraph {
    private static final double EDGE_LENGTH = 50.0;

    private OctagonSquareVoltageGraph() {
    }

    public static VoltageGraph build() {
        VoltageGraphBuilder builder = new VoltageGraphBuilder();

        Role role = builder.addRole(0, new OrientedPoint(0, 0, 0));
        builder.setPrimaryRole(role);

        HalfEdge a = builder.addHalfEdgePair(role, role, edgeAt(45, 90));
        HalfEdge c = a.getTwin();
        HalfEdge b = builder.addSelfTwinHalfEdge(role, edgeAt(270, 180));

        builder.setRotationOrder(role, a, c, b);

        return builder.build();
    }

    private static OrientedPoint edgeAt(double degrees, double rotationDegrees) {
        double radians = Math.toRadians(degrees);
        return new OrientedPoint(
            EDGE_LENGTH * Math.cos(radians),
            EDGE_LENGTH * Math.sin(radians),
            Math.toRadians(rotationDegrees));
    }
}
