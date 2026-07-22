package org.graphs.voltage;

import org.graphs.util.OrientedPoint;

public class HexagonTriangleVoltageGraph {
    private static final double EDGE_LENGTH = 50.0;

    private HexagonTriangleVoltageGraph() {
    }

    public static VoltageGraph build() {
        VoltageGraphBuilder builder = new VoltageGraphBuilder();

        Role r0 = builder.addRole(0,new OrientedPoint(0,0,0));
        Role r1 = builder.addRole(1, new OrientedPoint(0,0,0));
        Role r2 = builder.addRole(2, new OrientedPoint(0,0,0));
        builder.setPrimaryRole(r0);

        //HalfEdges for r0
        HalfEdge r0A = builder.addHalfEdgePair(r0, r1, translateAt(0));
        HalfEdge r0B = builder.addHalfEdgePair(r0, r2, translateAt(60));
        HalfEdge r0C = builder.addHalfEdgePair(r0, r1, translateAt(180));
        HalfEdge r0D = builder.addHalfEdgePair(r0, r2, translateAt(240));

        HalfEdge r1A = r0C.getTwin();
        HalfEdge r1B = builder.addHalfEdgePair(r1, r2, translateAt(120));
        HalfEdge r1C = r0A.getTwin();
        HalfEdge r1D = builder.addHalfEdgePair(r1, r2, translateAt(300));

        HalfEdge r2A = r0D.getTwin();
        HalfEdge r2B = r1D.getTwin();
        HalfEdge r2C = r0B.getTwin();
        HalfEdge r2D = r1B.getTwin();

        builder.setRotationOrder(r0, r0A, r0D, r0C, r0B);
        builder.setRotationOrder(r1, r1A, r1D, r1C, r1B);
        builder.setRotationOrder(r2, r2A, r2D, r2C, r2B);

        return builder.build();
    }

    private static OrientedPoint translateAt(double degrees) {
        double radians = Math.toRadians(degrees);
        return new OrientedPoint(EDGE_LENGTH * Math.cos(radians), EDGE_LENGTH * Math.sin(radians), 0);
    }
}
