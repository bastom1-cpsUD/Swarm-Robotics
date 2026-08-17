package org.graphs.voltage;

import org.graphs.util.OrientedPoint;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Verifies VoltageGraphBuilder against the two lattices worked through in
 * DCEL-Implementation-Plan.md and the primer: the real HexagonLattice numbers
 * (org.graphs.HexagonLattice), and a square-lattice-equivalent that exercises
 * the self-loop rotation-order case HexagonLattice never triggers.
 *
 * Both fixtures declare their rotations CLOCKWISE (decreasing departure angle),
 * matching the convention every lattice in org.graphs.voltage uses. The builder
 * applies Edmonds' rule next(h) = sigma(twin(h)) -- a forward step in that
 * declared order -- so the faces below come out traced counter-clockwise.
 */
class VoltageGraphBuilderTest {

    private static final double EDGE_LENGTH = 50.0;
    private static final double COS60 = Math.cos(Math.toRadians(60));
    private static final double SIN60 = Math.sin(Math.toRadians(60));

    private static Map<String, VoltageGraph> allLattices() {
        Map<String, VoltageGraph> lattices = new LinkedHashMap<>();
        lattices.put("Square", SquareVoltageGraph.build());
        lattices.put("Hexagon", HexagonVoltageGraph.build());
        lattices.put("Triangle", TriangleVoltageGraph.build());
        lattices.put("OctagonSquare", OctagonSquareVoltageGraph.build());
        lattices.put("SnubSquare", SnubSquareVoltageGraph.build());
        lattices.put("SnubHexagon", SnubHexagonVoltageGraph.build());
        lattices.put("HexagonTriangle", HexagonTriangleVoltageGraph.build());
        lattices.put("HexagonSquareTriangle", HexagonSquareTriangleVoltageGraph.build());
        lattices.put("DodecagonTriangle", DodecagonTriangleVoltageGraph.build());
        lattices.put("DodecagonHexagonSquare", DodecagonHexagonSquareVoltageGraph.build());
        lattices.put("ElongatedTriangular", ElongatedTriangularVoltageGraph.build());
        return lattices;
    }

    @Test
    @DisplayName("Edmonds' rule: next(h) is the successor of twin(h) in twin(h)'s rotation order, in every lattice")
    void everyLattice_nextIsSuccessorOfTwinInRotationOrder() {
        for (Map.Entry<String, VoltageGraph> entry : allLattices().entrySet()) {
            VoltageGraph graph = entry.getValue();
            for (Role role : graph.getRoles()) {
                for (HalfEdge h : graph.getOutgoingHalfEdges(role)) {
                    HalfEdge twin = h.getTwin();
                    List<HalfEdge> order = graph.getOutgoingHalfEdges(twin.getOrigin());
                    int i = order.indexOf(twin);
                    assertTrue(i >= 0, entry.getKey() + ": twin of edge " + h.getId()
                            + " missing from its own role's rotation order");

                    HalfEdge expected = order.get((i + 1) % order.size());
                    assertSame(expected, h.getNext(), entry.getKey()
                            + ": next(" + h.getId() + ") must be sigma(twin(h)), the entry AFTER"
                            + " twin " + twin.getId() + " in its rotation order");
                }
            }
        }
    }

    @Test
    @DisplayName("every lattice declares its rotation order clockwise (decreasing departure angle)")
    void everyLattice_declaresClockwiseRotationOrder() {
        for (Map.Entry<String, VoltageGraph> entry : allLattices().entrySet()) {
            VoltageGraph graph = entry.getValue();
            for (Role role : graph.getRoles()) {
                List<HalfEdge> order = graph.getOutgoingHalfEdges(role);
                if (order.size() < 3) {
                    continue; // fewer than three edges cannot distinguish CW from CCW
                }

                // Walking a strictly clockwise cyclic order, the departure angle
                // decreases at every step but one -- the single wrap past 0.
                int ascents = 0;
                for (int i = 0; i < order.size(); i++) {
                    double a = departureAngle(order.get(i));
                    double b = departureAngle(order.get((i + 1) % order.size()));
                    if (b > a) {
                        ascents++;
                    }
                }
                assertEquals(1, ascents, entry.getKey() + ", role " + role.getId()
                        + ": rotation order is not clockwise (expected exactly one wrap"
                        + " in an otherwise decreasing angle sequence)");
            }
        }
    }

    private static double departureAngle(HalfEdge h) {
        OrientedPoint step = h.getVoltage().asPose();
        double angle = Math.atan2(step.y, step.x);
        return angle < 0 ? angle + 2 * Math.PI : angle;
    }

    @Test
    @DisplayName("hexagon lattice: clockwise rotation order traces the six-step cycle e1->e5->e3->e4->e2->e6->e1")
    void hexagonLattice_tracesSixStepCycle() {
        VoltageGraphBuilder builder = new VoltageGraphBuilder();

        Role v0 = builder.addRole(1, new OrientedPoint(0, 0, 0));
        Role v1 = builder.addRole(2, new OrientedPoint(EDGE_LENGTH * COS60, EDGE_LENGTH * SIN60, 0));
        builder.setPrimaryRole(v0);

        // Inserted clockwise (departure angles 60, 300, 180), as HexagonVoltageGraph
        // does. Neither role is an endpoint of its own edges, so insertion order is
        // already the true rotation order -- no setRotationOrder call needed.
        HalfEdge e1 = builder.addHalfEdgePair(v0, v1, new OrientedPoint(EDGE_LENGTH * COS60, EDGE_LENGTH * SIN60, 0));  // UP-RIGHT
        HalfEdge e2 = builder.addHalfEdgePair(v0, v1, new OrientedPoint(EDGE_LENGTH * COS60, -EDGE_LENGTH * SIN60, 0)); // DOWN-RIGHT
        HalfEdge e3 = builder.addHalfEdgePair(v0, v1, new OrientedPoint(-EDGE_LENGTH, 0, 0));                           // LEFT

        HalfEdge e4 = e1.getTwin(); // DOWN-LEFT, from v1
        HalfEdge e5 = e2.getTwin(); // UP-LEFT,   from v1
        HalfEdge e6 = e3.getTwin(); // RIGHT,     from v1

        VoltageGraph graph = builder.build();

        // sigma(twin(h)) at each hop: twin(e1) = e4 sits at index 0 of v1's order
        // [e4, e5, e6], so next(e1) is e5; and so on around the face.
        assertSame(e5, e1.getNext());
        assertSame(e3, e5.getNext());
        assertSame(e4, e3.getNext());
        assertSame(e2, e4.getNext());
        assertSame(e6, e2.getNext());
        assertSame(e1, e6.getNext());

        List<HalfEdge> walk = List.of(e1, e5, e3, e4, e2, e6);
        assertTrue(graph.validateCycle(walk));

        assertEquals(1, graph.getFaces().size());
        assertEquals(6, graph.getFaces().get(0).getCycleLength());
    }

    @Test
    @DisplayName("square lattice: clockwise rotation order traces RIGHT->UP->LEFT->DOWN->RIGHT")
    void squareLattice_withExplicitRotationOrder_tracesFourStepCycle() {
        VoltageGraphBuilder builder = new VoltageGraphBuilder();

        Role role = builder.addRole(0, new OrientedPoint(0, 0, 0));
        builder.setPrimaryRole(role);

        HalfEdge right = builder.addHalfEdgePair(role, role, new OrientedPoint(EDGE_LENGTH, 0, 0));
        HalfEdge up = builder.addHalfEdgePair(role, role, new OrientedPoint(0, EDGE_LENGTH, 0));
        HalfEdge left = right.getTwin();
        HalfEdge down = up.getTwin();

        // Clockwise (departure angles 0, 270, 180, 90), as SquareVoltageGraph does.
        builder.setRotationOrder(role, right, down, left, up);
        VoltageGraph graph = builder.build();

        // twin(RIGHT) = LEFT at index 2, so next(RIGHT) is index 3 = UP.
        assertSame(up, right.getNext());
        assertSame(left, up.getNext());
        assertSame(down, left.getNext());
        assertSame(right, down.getNext());

        assertTrue(graph.validateCycle(List.of(right, up, left, down)));
        assertEquals(1, graph.getFaces().size());
        assertEquals(4, graph.getFaces().get(0).getCycleLength());
    }

    @Test
    @DisplayName("square lattice: omitting the explicit rotation order for a self-loop role fails to close")
    void squareLattice_withoutExplicitRotationOrder_failsToClose() {
        VoltageGraphBuilder builder = new VoltageGraphBuilder();

        Role role = builder.addRole(0, new OrientedPoint(0, 0, 0));
        builder.setPrimaryRole(role);

        builder.addHalfEdgePair(role, role, new OrientedPoint(EDGE_LENGTH, 0, 0));
        builder.addHalfEdgePair(role, role, new OrientedPoint(0, EDGE_LENGTH, 0));

        // No setRotationOrder call: default insertion order is RIGHT, LEFT, UP, DOWN,
        // which is not the true rotation order for a role that is both endpoints
        // of its own edges -- build() should catch this via the holonomy check.
        // Under sigma(twin(h)) that order yields the two-step orbit RIGHT -> UP,
        // whose voltage accumulates (50, 50) per lap and never reaches identity.
        assertThrows(IllegalStateException.class, builder::build);
    }

    @Test
    @DisplayName("build() requires a primary role to be set")
    void build_withoutPrimaryRole_throws() {
        VoltageGraphBuilder builder = new VoltageGraphBuilder();
        builder.addRole(0, new OrientedPoint(0, 0, 0));

        assertThrows(IllegalStateException.class, builder::build);
    }
}
