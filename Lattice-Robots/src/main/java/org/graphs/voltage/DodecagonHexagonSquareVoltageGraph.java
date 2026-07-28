package org.graphs.voltage;

import org.graphs.util.OrientedPoint;

/**
 * The truncated trihexagonal tiling (4.6.12) as a VoltageGraph: 12 roles, one per
 * translation-orbit vertex, connected entirely by PURE TRANSLATIONS -- no edge
 * carries any rotation, so a robot's heading never has to accumulate anything
 * for the formation to be geometrically correct (the same reason
 * OctagonSquareVoltageGraph and DodecagonTriangleVoltageGraph use multiple
 * roles instead of rotation-carrying edges).
 *
 * One translational unit cell has 12 vertices and the faces 3 squares, 2 hexagons and 1 dodecagon.
 * Coordinates are derived from the tiling's exact lattice and verified so that
 * every face's holonomy closes to the identity.
 */
public final class DodecagonHexagonSquareVoltageGraph {
    private static final double EDGE_LENGTH = 50.0;

    private DodecagonHexagonSquareVoltageGraph() {
    }

    public static VoltageGraph build() {
        VoltageGraphBuilder builder = new VoltageGraphBuilder();

        Role r0 = builder.addRole(0, new OrientedPoint(0, 0, 0));
        Role r1 = builder.addRole(1, new OrientedPoint(0, 0, 0));
        Role r2 = builder.addRole(2, new OrientedPoint(0, 0, 0));
        Role r3 = builder.addRole(3, new OrientedPoint(0, 0, 0));
        Role r4 = builder.addRole(4, new OrientedPoint(0, 0, 0));
        Role r5 = builder.addRole(5, new OrientedPoint(0, 0, 0));
        Role r6 = builder.addRole(6, new OrientedPoint(0, 0, 0));
        Role r7 = builder.addRole(7, new OrientedPoint(0, 0, 0));
        Role r8 = builder.addRole(8, new OrientedPoint(0, 0, 0));
        Role r9 = builder.addRole(9, new OrientedPoint(0, 0, 0));
        Role r10 = builder.addRole(10, new OrientedPoint(0, 0, 0));
        Role r11 = builder.addRole(11, new OrientedPoint(0, 0, 0));
        builder.setPrimaryRole(r0);

        HalfEdge e0 = builder.addHalfEdgePair(r0, r11, translateAt(270));  // r0 -(270 deg)-> r11
        HalfEdge e1 = builder.addHalfEdgePair(r0, r1, translateAt(120));  // r0 -(120 deg)-> r1
        HalfEdge e2 = builder.addHalfEdgePair(r0, r7, translateAt(30));  // r0 -(30 deg)-> r7
        HalfEdge e3 = builder.addHalfEdgePair(r1, r2, translateAt(150));  // r1 -(150 deg)-> r2
        HalfEdge e4 = builder.addHalfEdgePair(r1, r6, translateAt(30));  // r1 -(30 deg)-> r6
        HalfEdge e5 = builder.addHalfEdgePair(r2, r3, translateAt(180));  // r2 -(180 deg)-> r3
        HalfEdge e6 = builder.addHalfEdgePair(r2, r9, translateAt(90));  // r2 -(90 deg)-> r9
        HalfEdge e7 = builder.addHalfEdgePair(r3, r4, translateAt(210));  // r3 -(210 deg)-> r4
        HalfEdge e8 = builder.addHalfEdgePair(r3, r8, translateAt(90));  // r3 -(90 deg)-> r8
        HalfEdge e9 = builder.addHalfEdgePair(r4, r5, translateAt(240));  // r4 -(240 deg)-> r5
        HalfEdge e10 = builder.addHalfEdgePair(r4, r11, translateAt(150));  // r4 -(150 deg)-> r11
        HalfEdge e11 = builder.addHalfEdgePair(r5, r6, translateAt(270));  // r5 -(270 deg)-> r6
        HalfEdge e12 = builder.addHalfEdgePair(r5, r10, translateAt(150));  // r5 -(150 deg)-> r10
        HalfEdge e13 = builder.addHalfEdgePair(r6, r7, translateAt(300));  // r6 -(300 deg)-> r7
        HalfEdge e14 = builder.addHalfEdgePair(r7, r8, translateAt(330));  // r7 -(330 deg)-> r8
        HalfEdge e15 = builder.addHalfEdgePair(r8, r9, translateAt(360));  // r8 -(360 deg)-> r9
        HalfEdge e16 = builder.addHalfEdgePair(r9, r10, translateAt(30));  // r9 -(30 deg)-> r10
        HalfEdge e17 = builder.addHalfEdgePair(r10, r11, translateAt(60));  // r10 -(60 deg)-> r11

        builder.setRotationOrder(r0, e0, e1, e2);
        builder.setRotationOrder(r1, e1.getTwin(), e3, e4);
        builder.setRotationOrder(r2, e3.getTwin(), e5, e6);
        builder.setRotationOrder(r3, e7, e8, e5.getTwin());
        builder.setRotationOrder(r4, e9, e10, e7.getTwin());
        builder.setRotationOrder(r5, e11, e12, e9.getTwin());
        builder.setRotationOrder(r6, e13, e4.getTwin(), e11.getTwin());
        builder.setRotationOrder(r7, e14, e2.getTwin(), e13.getTwin());
        builder.setRotationOrder(r8, e15, e8.getTwin(), e14.getTwin());
        builder.setRotationOrder(r9, e6.getTwin(), e15.getTwin(), e16);
        builder.setRotationOrder(r10, e12.getTwin(), e16.getTwin(), e17);
        builder.setRotationOrder(r11, e10.getTwin(), e17.getTwin(), e0.getTwin());

        return builder.build();
    }

    private static OrientedPoint translateAt(double degrees) {
        double radians = Math.toRadians(degrees);
        return new OrientedPoint(EDGE_LENGTH * Math.cos(radians), EDGE_LENGTH * Math.sin(radians), 0);
    }
}
