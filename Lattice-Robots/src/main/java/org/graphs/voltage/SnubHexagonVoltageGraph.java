package org.graphs.voltage;

import org.graphs.util.OrientedPoint;

/**
 * The snub hexagonal tiling (3.3.3.3.6) as a VoltageGraph: 6 roles, one per
 * translation-orbit vertex, connected entirely by PURE TRANSLATIONS -- no edge
 * carries any rotation, so a robot's heading never has to accumulate anything
 * for the formation to be geometrically correct (the same reason
 * OctagonSquareVoltageGraph and DodecagonTriangleVoltageGraph use multiple
 * roles instead of rotation-carrying edges).
 *
 * One translational unit cell has 6 vertices and the faces 8 triangles and 1 hexagon.
 * Coordinates are derived from the tiling's exact lattice and verified so that
 * every face's holonomy closes to the identity.
 */
public final class SnubHexagonVoltageGraph {
    private static final double EDGE_LENGTH = 50.0;

    private SnubHexagonVoltageGraph() {
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

        HalfEdge e0 = builder.addHalfEdgePair(r0, r5, translateAt(240));  // r0 -(240 deg)-> r5
        HalfEdge e1 = builder.addHalfEdgePair(r0, r4, translateAt(180));  // r0 -(180 deg)-> r4
        HalfEdge e2 = builder.addHalfEdgePair(r0, r3, translateAt(120));  // r0 -(120 deg)-> r3
        HalfEdge e3 = builder.addHalfEdgePair(r0, r2, translateAt(60));  // r0 -(60 deg)-> r2
        HalfEdge e4 = builder.addHalfEdgePair(r0, r1, translateAt(0));  // r0 -(0 deg)-> r1
        HalfEdge e5 = builder.addHalfEdgePair(r1, r3, translateAt(300));  // r1 -(300 deg)-> r3
        HalfEdge e6 = builder.addHalfEdgePair(r1, r2, translateAt(120));  // r1 -(120 deg)-> r2
        HalfEdge e7 = builder.addHalfEdgePair(r1, r4, translateAt(60));  // r1 -(60 deg)-> r4
        HalfEdge e8 = builder.addHalfEdgePair(r1, r5, translateAt(0));  // r1 -(0 deg)-> r5
        HalfEdge e9 = builder.addHalfEdgePair(r2, r3, translateAt(180));  // r2 -(180 deg)-> r3
        HalfEdge e10 = builder.addHalfEdgePair(r2, r5, translateAt(120));  // r2 -(120 deg)-> r5
        HalfEdge e11 = builder.addHalfEdgePair(r2, r4, translateAt(0));  // r2 -(0 deg)-> r4
        HalfEdge e12 = builder.addHalfEdgePair(r3, r4, translateAt(240));  // r3 -(240 deg)-> r4
        HalfEdge e13 = builder.addHalfEdgePair(r3, r5, translateAt(60));  // r3 -(60 deg)-> r5
        HalfEdge e14 = builder.addHalfEdgePair(r4, r5, translateAt(300));  // r4 -(300 deg)-> r5

        builder.setRotationOrder(r0, e0, e1, e2, e3, e4);
        builder.setRotationOrder(r1, e5, e4.getTwin(), e6, e7, e8);
        builder.setRotationOrder(r2, e6.getTwin(), e3.getTwin(), e9, e10, e11);
        builder.setRotationOrder(r3, e2.getTwin(), e12, e5.getTwin(), e13, e9.getTwin());
        builder.setRotationOrder(r4, e14, e7.getTwin(), e11.getTwin(), e12.getTwin(), e1.getTwin());
        builder.setRotationOrder(r5, e10.getTwin(), e13.getTwin(), e8.getTwin(), e14.getTwin(), e0.getTwin());

        return builder.build();
    }

    private static OrientedPoint translateAt(double degrees) {
        double radians = Math.toRadians(degrees);
        return new OrientedPoint(EDGE_LENGTH * Math.cos(radians), EDGE_LENGTH * Math.sin(radians), 0);
    }
}
