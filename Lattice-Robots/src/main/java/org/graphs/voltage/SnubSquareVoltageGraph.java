package org.graphs.voltage;

import org.graphs.util.OrientedPoint;

/**
 * The snub square tiling (3.3.4.3.4) as a VoltageGraph: 4 roles, one per
 * translation-orbit vertex, connected entirely by PURE TRANSLATIONS -- no edge
 * carries any rotation, so a robot's heading never has to accumulate anything
 * for the formation to be geometrically correct (the same reason
 * OctagonSquareVoltageGraph and DodecagonTriangleVoltageGraph use multiple
 * roles instead of rotation-carrying edges).
 *
 * One translational unit cell has 4 vertices and the faces 4 triangles and 2 squares.
 * Coordinates are derived from the tiling's exact lattice and verified so that
 * every face's holonomy closes to the identity.
 */
public final class SnubSquareVoltageGraph {
    private static final double EDGE_LENGTH = 70.0;

    private SnubSquareVoltageGraph() {
    }

    public static VoltageGraph build() {
        VoltageGraphBuilder builder = new VoltageGraphBuilder();

        Role r0 = builder.addRole(0, new OrientedPoint(0, 0, 0));
        Role r1 = builder.addRole(1, new OrientedPoint(0, 0, 0));
        Role r2 = builder.addRole(2, new OrientedPoint(0, 0, 0));
        Role r3 = builder.addRole(3, new OrientedPoint(0, 0, 0));
        builder.setPrimaryRole(r0);

        HalfEdge e0 = builder.addHalfEdgePair(r0, r3, translateAt(270));  // r0 -(270 deg)-> r3
        HalfEdge e1 = builder.addHalfEdgePair(r0, r1, translateAt(210));  // r0 -(210 deg)-> r1
        HalfEdge e2 = builder.addHalfEdgePair(r0, r3, translateAt(120));  // r0 -(120 deg)-> r3
        HalfEdge e3 = builder.addHalfEdgePair(r0, r2, translateAt(60));  // r0 -(60 deg)-> r2
        HalfEdge e4 = builder.addHalfEdgePair(r0, r1, translateAt(0));  // r0 -(0 deg)-> r1
        HalfEdge e5 = builder.addHalfEdgePair(r1, r3, translateAt(330));  // r1 -(330 deg)-> r3
        HalfEdge e6 = builder.addHalfEdgePair(r1, r2, translateAt(270));  // r1 -(270 deg)-> r2
        HalfEdge e7 = builder.addHalfEdgePair(r1, r2, translateAt(120));  // r1 -(120 deg)-> r2
        HalfEdge e8 = builder.addHalfEdgePair(r2, r3, translateAt(180));  // r2 -(180 deg)-> r3
        HalfEdge e9 = builder.addHalfEdgePair(r2, r3, translateAt(30));  // r2 -(30 deg)-> r3

        builder.setRotationOrder(r0, e0, e1, e2, e3, e4);
        builder.setRotationOrder(r1, e5, e6, e4.getTwin(), e7, e1.getTwin());
        builder.setRotationOrder(r2, e7.getTwin(), e3.getTwin(), e8, e6.getTwin(), e9);
        builder.setRotationOrder(r3, e2.getTwin(), e9.getTwin(), e5.getTwin(), e0.getTwin(), e8.getTwin());

        return builder.build();
    }

    private static OrientedPoint translateAt(double degrees) {
        double radians = Math.toRadians(degrees);
        return new OrientedPoint(EDGE_LENGTH * Math.cos(radians), EDGE_LENGTH * Math.sin(radians), 0);
    }
}
