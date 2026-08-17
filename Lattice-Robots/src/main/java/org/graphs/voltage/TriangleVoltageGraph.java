package org.graphs.voltage;

import org.graphs.util.OrientedPoint;

/**
 * The triangular (3.3.3.3.3.3) lattice as a VoltageGraph: one role with six
 * neighbors, declared as three half-edge pairs -- outgoing at 0/60/120 degrees,
 * their twins covering 180/240/300. Every edge is a pure translation, so a
 * robot's orientation never has to accumulate anything (same property the
 * OctagonSquareVoltageGraph comment argues for).
 *
 * The single role is both endpoints of every edge, so an edge and its twin
 * point in opposite directions and are never adjacent in true rotation order --
 * rotation order is set explicitly rather than left to insertion order, exactly
 * as in SquareVoltageGraph. See DCEL-Implementation-Plan.md sec 5 / primer
 * sec 3-4.
 *
 * The six half-edges fall into two 3-cycles: the "up" triangles (0 -> 240 ->
 * 120) and the "down" triangles (180 -> 60 -> 300), which is the expected
 * quotient -- a vertex touches six triangles, each triangle has three vertices,
 * so two faces per role.
 */
public final class TriangleVoltageGraph {
    private static final double EDGE_LENGTH = 50.0;

    private TriangleVoltageGraph() {
    }

    public static VoltageGraph build() {
        VoltageGraphBuilder builder = new VoltageGraphBuilder();

        Role role = builder.addRole(0, new OrientedPoint(0, 0, 0));
        builder.setPrimaryRole(role);

        HalfEdge e0 = builder.addHalfEdgePair(role, role, translateAt(0));
        HalfEdge e60 = builder.addHalfEdgePair(role, role, translateAt(60));
        HalfEdge e120 = builder.addHalfEdgePair(role, role, translateAt(120));
        HalfEdge e180 = e0.getTwin();
        HalfEdge e240 = e60.getTwin();
        HalfEdge e300 = e120.getTwin();

        // Clockwise rotation order (decreasing departure angle: 0, 300, 240,
        // 180, 120, 60), matching the convention used across the other lattice
        // graphs. Under Edmonds' sigma(twin(h)) a clockwise order traces faces
        // counter-clockwise.
        builder.setRotationOrder(role, e0, e300, e240, e180, e120, e60);

        return builder.build();
    }

    private static OrientedPoint translateAt(double degrees) {
        double radians = Math.toRadians(degrees);
        return new OrientedPoint(EDGE_LENGTH * Math.cos(radians), EDGE_LENGTH * Math.sin(radians), 0);
    }
}
