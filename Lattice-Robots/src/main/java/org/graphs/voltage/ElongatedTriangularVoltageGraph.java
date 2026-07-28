package org.graphs.voltage;

import org.graphs.util.OrientedPoint;

/**
 * The elongated triangular tiling (3.3.3.4.4) as a VoltageGraph: 2 roles, one per
 * translation-orbit vertex, connected entirely by PURE TRANSLATIONS -- no edge
 * carries any rotation, so a robot's heading never has to accumulate anything
 * for the formation to be geometrically correct (the same reason
 * OctagonSquareVoltageGraph and DodecagonTriangleVoltageGraph use multiple
 * roles instead of rotation-carrying edges).
 *
 * One translational unit cell has 2 vertices and the faces 2 triangles and 1 square.
 * Coordinates are derived from the tiling's exact lattice and verified so that
 * every face's holonomy closes to the identity.
 */
public final class ElongatedTriangularVoltageGraph {
    private static final double EDGE_LENGTH = 50.0;

    private ElongatedTriangularVoltageGraph() {
    }

    public static VoltageGraph build() {
        VoltageGraphBuilder builder = new VoltageGraphBuilder();

        Role r0 = builder.addRole(0, new OrientedPoint(0, 0, 0));
        Role r1 = builder.addRole(1, new OrientedPoint(0, 0, 0));
        builder.setPrimaryRole(r0);

        HalfEdge e0 = builder.addHalfEdgePair(r0, r1, translateAt(300));  // r0 -(300 deg)-> r1
        HalfEdge e1 = builder.addHalfEdgePair(r0, r1, translateAt(240));  // r0 -(240 deg)-> r1
        HalfEdge e2 = builder.addHalfEdgePair(r0, r1, translateAt(90));  // r0 -(90 deg)-> r1
        HalfEdge e3 = builder.addHalfEdgePair(r0, r0, translateAt(0));  // r0 -(0 deg)-> r0
        HalfEdge e4 = builder.addHalfEdgePair(r1, r1, translateAt(0));  // r1 -(0 deg)-> r1

        builder.setRotationOrder(r0, e0, e1, e3.getTwin(), e2, e3);
        builder.setRotationOrder(r1, e2.getTwin(), e4.getTwin(), e0.getTwin(), e1.getTwin(), e4);

        return builder.build();
    }

    private static OrientedPoint translateAt(double degrees) {
        double radians = Math.toRadians(degrees);
        return new OrientedPoint(EDGE_LENGTH * Math.cos(radians), EDGE_LENGTH * Math.sin(radians), 0);
    }
}
