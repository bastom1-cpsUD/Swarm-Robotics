package org.graphs.voltage;

import org.graphs.util.OrientedPoint;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Verifies VoltageGraphBuilder against the two lattices worked through in
 * DCEL-Implementation-Plan.md and the primer: the real HexagonLattice numbers
 * (org.graphs.HexagonLattice), and a square-lattice-equivalent that exercises
 * the self-loop rotation-order case HexagonLattice never triggers.
 */
class VoltageGraphBuilderTest {

    private static final double EDGE_LENGTH = 50.0;
    private static final double COS60 = Math.cos(Math.toRadians(60));
    private static final double SIN60 = Math.sin(Math.toRadians(60));

    @Test
    @DisplayName("hexagon lattice: next() traces the six-step cycle e1->e4->e2->e5->e3->e6->e1")
    void hexagonLattice_tracesSixStepCycle() {
        VoltageGraphBuilder builder = new VoltageGraphBuilder();

        Role v0 = builder.addRole(1, new OrientedPoint(0, 0, 0));
        Role v1 = builder.addRole(2, new OrientedPoint(EDGE_LENGTH * COS60, EDGE_LENGTH * SIN60, 0));
        builder.setPrimaryRole(v0);

        HalfEdge e1 = builder.addHalfEdgePair(v0, v1, new OrientedPoint(-EDGE_LENGTH, 0, 0));               // LEFT
        HalfEdge e2 = builder.addHalfEdgePair(v0, v1, new OrientedPoint(EDGE_LENGTH * COS60, -EDGE_LENGTH * SIN60, 0)); // DOWN-RIGHT
        HalfEdge e3 = builder.addHalfEdgePair(v0, v1, new OrientedPoint(EDGE_LENGTH * COS60, EDGE_LENGTH * SIN60, 0));  // UP-RIGHT

        HalfEdge e5 = e1.getTwin(); // RIGHT, from v1
        HalfEdge e6 = e2.getTwin(); // UP-LEFT, from v1
        HalfEdge e4 = e3.getTwin(); // DOWN-LEFT, from v1

        VoltageGraph graph = builder.build();

        assertSame(e4, e1.getNext());
        assertSame(e2, e4.getNext());
        assertSame(e5, e2.getNext());
        assertSame(e3, e5.getNext());
        assertSame(e6, e3.getNext());
        assertSame(e1, e6.getNext());

        List<HalfEdge> walk = List.of(e1, e4, e2, e5, e3, e6);
        assertTrue(graph.validateCycle(walk));

        assertEquals(1, graph.getFaces().size());
        assertEquals(6, graph.getFaces().get(0).getCycleLength());
    }

    @Test
    @DisplayName("square lattice: explicit rotation order traces RIGHT->UP->LEFT->DOWN->RIGHT")
    void squareLattice_withExplicitRotationOrder_tracesFourStepCycle() {
        VoltageGraphBuilder builder = new VoltageGraphBuilder();

        Role role = builder.addRole(0, new OrientedPoint(0, 0, 0));
        builder.setPrimaryRole(role);

        HalfEdge right = builder.addHalfEdgePair(role, role, new OrientedPoint(EDGE_LENGTH, 0, 0));
        HalfEdge up = builder.addHalfEdgePair(role, role, new OrientedPoint(0, EDGE_LENGTH, 0));
        HalfEdge left = right.getTwin();
        HalfEdge down = up.getTwin();

        builder.setRotationOrder(role, right, up, left, down);
        VoltageGraph graph = builder.build();

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
