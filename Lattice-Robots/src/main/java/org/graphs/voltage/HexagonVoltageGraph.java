package org.graphs.voltage;

import org.graphs.util.OrientedPoint;

/**
 * The hexagon lattice as a VoltageGraph: two roles, three edges each, all
 * running v0-to-v1 (their twins, v1-to-v0, are derived automatically). Same
 * EDGE_LENGTH and poses as org.graphs.HexagonLattice. Neither role is an
 * endpoint of its own edges, so insertion order already matches true
 * rotation order -- no explicit setRotationOrder call is needed here, unlike
 * SquareVoltageGraph. See DCEL-Implementation-Plan.md sec 5 / primer sec 3, 7.
 */
public final class HexagonVoltageGraph {
    private static final double EDGE_LENGTH = 50.0;
    private static final double COS_60 = Math.cos(Math.toRadians(60));
    private static final double SIN_60 = Math.sin(Math.toRadians(60));

    private HexagonVoltageGraph() {
    }

    public static VoltageGraph build() {
        VoltageGraphBuilder builder = new VoltageGraphBuilder();

        Role v0 = builder.addRole(1, new OrientedPoint(0, 0, 0));
        Role v1 = builder.addRole(2, new OrientedPoint(EDGE_LENGTH * COS_60, EDGE_LENGTH * SIN_60, 0));
        builder.setPrimaryRole(v0);

        builder.addHalfEdgePair(v0, v1, new OrientedPoint(-EDGE_LENGTH, 0, 0));                              // LEFT
        builder.addHalfEdgePair(v0, v1, new OrientedPoint(EDGE_LENGTH * COS_60, -EDGE_LENGTH * SIN_60, 0));  // DOWN-RIGHT
        builder.addHalfEdgePair(v0, v1, new OrientedPoint(EDGE_LENGTH * COS_60, EDGE_LENGTH * SIN_60, 0));   // UP-RIGHT

        return builder.build();
    }
}
