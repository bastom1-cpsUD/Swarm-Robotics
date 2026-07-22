package org.graphs.voltage;

import org.graphs.util.OrientedPoint;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Smoke tests for the concrete lattice graph files: each should build without
 * throwing (i.e. every face's holonomy actually closes) and produce the
 * expected face shapes.
 */
class VoltageGraphsTest {

    @Test
    @DisplayName("SquareVoltageGraph: one four-step face")
    void squareVoltageGraph_hasOneFourStepFace() {
        VoltageGraph graph = SquareVoltageGraph.build();
        assertEquals(1, graph.getFaces().size());
        assertEquals(4, graph.getFaces().get(0).getCycleLength());
    }

    @Test
    @DisplayName("HexagonVoltageGraph: one six-step face")
    void hexagonVoltageGraph_hasOneSixStepFace() {
        VoltageGraph graph = HexagonVoltageGraph.build();
        assertEquals(1, graph.getFaces().size());
        assertEquals(6, graph.getFaces().get(0).getCycleLength());
    }

    @Test
    @DisplayName("OctagonSquareVoltageGraph: one square face and one octagon face, from four roles")
    void octagonSquareVoltageGraph_hasSquareAndOctagonFaces() {
        VoltageGraph graph = OctagonSquareVoltageGraph.build();

        // Four roles -- one per rotation state (0/90/180/270 degrees) a copy
        // of the tiling's single vertex-transitive vertex can appear in --
        // connected entirely by pure translations, so no edge ever needs a
        // rotation component and no role needs to be self-referencing.
        assertEquals(4, graph.getRoles().size());
        for (Role role : graph.getRoles()) {
            assertEquals(3, graph.getOutgoingHalfEdges(role).size());
        }

        assertEquals(2, graph.getFaces().size());
        Set<Integer> cycleLengths = graph.getFaces().stream()
                .map(Face::getCycleLength)
                .collect(Collectors.toSet());
        assertEquals(Set.of(4, 8), cycleLengths);
    }

    @Test
    @DisplayName("OctagonSquareVoltageGraph: every edge is a pure translation (zero rotation)")
    void octagonSquareVoltageGraph_everyEdgeIsPureTranslation() {
        VoltageGraph graph = OctagonSquareVoltageGraph.build();

        for (Role role : graph.getRoles()) {
            for (HalfEdge h : graph.getOutgoingHalfEdges(role)) {
                OrientedPoint delta = h.getVoltage().apply(new OrientedPoint(0, 0, 0));
                assertEquals(0.0, delta.getOrientation(), 1e-9);
            }
        }
    }

    @Test
    @DisplayName("OctagonSquareVoltageGraph: the square face visits four distinct roles via four distinct edges")
    void octagonSquareVoltageGraph_squareFaceUsesFourDistinctEdges() {
        VoltageGraph graph = OctagonSquareVoltageGraph.build();

        Face squareFace = graph.getFaces().stream()
                .filter(f -> f.getCycleLength() == 4)
                .findFirst().orElseThrow();

        List<HalfEdge> boundary = squareFace.getBoundary();
        assertEquals(4, boundary.size());
        assertEquals(4, Set.copyOf(boundary).size());
        assertEquals(4, boundary.stream().map(HalfEdge::getOrigin).collect(Collectors.toSet()).size());
        assertTrue(graph.validateCycle(boundary));
    }

    @Test
    @DisplayName("OctagonSquareVoltageGraph: the octagon face uses eight distinct edges")
    void octagonSquareVoltageGraph_octagonFaceUsesEightDistinctEdges() {
        VoltageGraph graph = OctagonSquareVoltageGraph.build();

        Face octagonFace = graph.getFaces().stream()
                .filter(f -> f.getCycleLength() == 8)
                .findFirst().orElseThrow();

        List<HalfEdge> boundary = octagonFace.getBoundary();
        assertEquals(8, boundary.size());
        assertEquals(8, Set.copyOf(boundary).size());
        assertTrue(graph.validateCycle(boundary));
    }
}
