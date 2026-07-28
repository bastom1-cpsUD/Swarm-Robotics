package org.graphs.voltage;

import org.graphs.util.OrientedPoint;

/**
 * The rhombitrihexagonal tiling (3.4.6.4) as a VoltageGraph: 6 roles, one per
 * translation-orbit vertex, connected entirely by PURE TRANSLATIONS -- no edge
 * carries any rotation, so a robot's heading never has to accumulate anything
 * for the formation to be geometrically correct (the same reason
 * OctagonSquareVoltageGraph and DodecagonTriangleVoltageGraph use multiple
 * roles instead of rotation-carrying edges).
 *
 * One translational unit cell has 6 vertices and the faces 2 triangles, 3 squares and 1 hexagon.
 * Coordinates are derived from the tiling's exact lattice and verified so that
 * every face's holonomy closes to the identity.
 */
public final class HexagonSquareTriangleVoltageGraph {
    private static final double EDGE_LENGTH = 50.0;

    private HexagonSquareTriangleVoltageGraph() {
    }

    public static VoltageGraph build() {
        VoltageGraphBuilder builder = new VoltageGraphBuilder();

        Role r0 = builder.addRole(0, new OrientedPoint(0, 0, 0));
        Role r1 = builder.addRole(1, new OrientedPoint(0, 0, 0));
        Role r2 = builder.addRole(2, new OrientedPoint(0, 0, 0));
        Role r3 = builder.addRole(3, new OrientedPoint(0, 0, 0));
        Role r4 = builder.addRole(4, new OrientedPoint(0, 0, 0));
        Role r5 = builder.addRole(5, new OrientedPoint(0, 0, 0));
        builder.setPrimaryRole(r0);

        HalfEdge e0 = builder.addHalfEdgePair(r0, r2, translateAt(330));  // r0 -(330 deg)-> r2
        HalfEdge e1 = builder.addHalfEdgePair(r0, r5, translateAt(240));  // r0 -(240 deg)-> r5
        HalfEdge e2 = builder.addHalfEdgePair(r0, r1, translateAt(120));  // r0 -(120 deg)-> r1
        HalfEdge e3 = builder.addHalfEdgePair(r0, r4, translateAt(30));  // r0 -(30 deg)-> r4
        HalfEdge e4 = builder.addHalfEdgePair(r1, r2, translateAt(180));  // r1 -(180 deg)-> r2
        HalfEdge e5 = builder.addHalfEdgePair(r1, r5, translateAt(90));  // r1 -(90 deg)-> r5
        HalfEdge e6 = builder.addHalfEdgePair(r1, r3, translateAt(30));  // r1 -(30 deg)-> r3
        HalfEdge e7 = builder.addHalfEdgePair(r2, r3, translateAt(240));  // r2 -(240 deg)-> r3
        HalfEdge e8 = builder.addHalfEdgePair(r2, r4, translateAt(90));  // r2 -(90 deg)-> r4
        HalfEdge e9 = builder.addHalfEdgePair(r3, r4, translateAt(300));  // r3 -(300 deg)-> r4
        HalfEdge e10 = builder.addHalfEdgePair(r3, r5, translateAt(150));  // r3 -(150 deg)-> r5
        HalfEdge e11 = builder.addHalfEdgePair(r4, r5, translateAt(360));  // r4 -(360 deg)-> r5

        builder.setRotationOrder(r0, e0, e1, e2, e3);
        builder.setRotationOrder(r1, e2.getTwin(), e4, e5, e6);
        builder.setRotationOrder(r2, e4.getTwin(), e7, e0.getTwin(), e8);
        builder.setRotationOrder(r3, e9, e6.getTwin(), e10, e7.getTwin());
        builder.setRotationOrder(r4, e11, e8.getTwin(), e3.getTwin(), e9.getTwin());
        builder.setRotationOrder(r5, e10.getTwin(), e5.getTwin(), e11.getTwin(), e1.getTwin());

        return builder.build();
    }

    private static OrientedPoint translateAt(double degrees) {
        double radians = Math.toRadians(degrees);
        return new OrientedPoint(EDGE_LENGTH * Math.cos(radians), EDGE_LENGTH * Math.sin(radians), 0);
    }
}
