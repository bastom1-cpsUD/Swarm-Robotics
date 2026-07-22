package org.graphs.voltage;

import org.graphs.util.OrientedPoint;

/**
 * The square lattice as a VoltageGraph: one role, four edges (RIGHT/UP/LEFT/DOWN),
 * same EDGE_LENGTH as org.graphs.SquareLattice. The role is both endpoints of
 * every edge, so RIGHT and its twin LEFT are never adjacent in true rotation
 * order -- the rotation order is set explicitly rather than left to default
 * insertion order. See DCEL-Implementation-Plan.md sec 5 / primer sec 3-4.
 */
public final class SquareVoltageGraph {
    private static final double EDGE_LENGTH = 70.0;

    private SquareVoltageGraph() {
    }

    public static VoltageGraph build() {
        VoltageGraphBuilder builder = new VoltageGraphBuilder();

        Role role = builder.addRole(0, new OrientedPoint(0, 0, 0));
        builder.setPrimaryRole(role);

        HalfEdge right = builder.addHalfEdgePair(role, role, new OrientedPoint(EDGE_LENGTH, 0, 0));
        HalfEdge up = builder.addHalfEdgePair(role, role, new OrientedPoint(0, EDGE_LENGTH, 0));
        HalfEdge left = right.getTwin();
        HalfEdge down = up.getTwin();

        builder.setRotationOrder(role, right, up, left, down);

        return builder.build();
    }
}
