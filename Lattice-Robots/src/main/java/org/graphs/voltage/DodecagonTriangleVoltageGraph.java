package org.graphs.voltage;

import org.graphs.util.OrientedPoint;

public class DodecagonTriangleVoltageGraph {
    private static final double EDGE_LENGTH = 50.0;

    private DodecagonTriangleVoltageGraph() {
    }

    public static VoltageGraph build() {
        VoltageGraphBuilder builder = new VoltageGraphBuilder();

        Role r0 = builder.addRole(0,new OrientedPoint(0,0,0));
        Role r1 = builder.addRole(1, new OrientedPoint(0,0,0));
        Role r2 = builder.addRole(2, new OrientedPoint(0,0,0));
        Role r3 = builder.addRole(3, new OrientedPoint(0,0,0));
        Role r4 = builder.addRole(4, new OrientedPoint(0,0,0));
        Role r5 = builder.addRole(5, new OrientedPoint(0,0,0));
        builder.setPrimaryRole(r0);

        HalfEdge r0A = builder.addHalfEdgePair(r0, r1, translateAt(270));
        HalfEdge r0B = builder.addHalfEdgePair(r0, r4, translateAt(120));
        HalfEdge r0C = builder.addHalfEdgePair(r0, r5, translateAt(60));

        HalfEdge r1A = builder.addHalfEdgePair(r1, r3, translateAt(300));
        HalfEdge r1B = builder.addHalfEdgePair(r1, r2, translateAt(240));
        HalfEdge r1C = r0A.getTwin();

        HalfEdge r2A = builder.addHalfEdgePair(r2, r3, translateAt(0));
        HalfEdge r2B = builder.addHalfEdgePair(r2, r5, translateAt(210));
        HalfEdge r2C = r1B.getTwin();

        HalfEdge r3A = builder.addHalfEdgePair(r3, r4, translateAt(330));
        HalfEdge r3B = r2A.getTwin();
        HalfEdge r3C = r1A.getTwin();

        HalfEdge r4A = builder.addHalfEdgePair(r4, r5, translateAt(0));
        HalfEdge r4B = r0B.getTwin();
        HalfEdge r4C = r3A.getTwin();

        HalfEdge r5A = r0C.getTwin();
        HalfEdge r5B = r4A.getTwin();
        HalfEdge r5C = r2B.getTwin();

        builder.setRotationOrder(r0, r0A, r0B, r0C);
        builder.setRotationOrder(r1, r1A, r1B, r1C);
        builder.setRotationOrder(r2, r2A, r2B, r2C);
        builder.setRotationOrder(r3, r3A, r3B, r3C);
        builder.setRotationOrder(r4, r4A, r4B, r4C);
        builder.setRotationOrder(r5, r5A, r5B, r5C);

        return builder.build();
    }

    private static OrientedPoint translateAt(double degrees) {
        double radians = Math.toRadians(degrees);
        return new OrientedPoint(EDGE_LENGTH * Math.cos(radians), EDGE_LENGTH * Math.sin(radians), 0);
    }

}
