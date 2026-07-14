package org.graphs.voltage;

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
    @DisplayName("OctagonSquareVoltageGraph: one square face and one octagon face, from a single degree-3 role")
    void octagonSquareVoltageGraph_hasSquareAndOctagonFaces() {
        VoltageGraph graph = OctagonSquareVoltageGraph.build();

        assertEquals(1, graph.getRoles().size());
        assertEquals(3, graph.getOutgoingHalfEdges(graph.getPrimaryRole()).size());

        assertEquals(2, graph.getFaces().size());
        Set<Integer> cycleLengths = graph.getFaces().stream()
                .map(Face::getCycleLength)
                .collect(Collectors.toSet());
        assertEquals(Set.of(4, 8), cycleLengths);
    }

    @Test
    @DisplayName("OctagonSquareVoltageGraph: the square face repeats edge A four times")
    void octagonSquareVoltageGraph_squareFaceRepeatsSingleEdge() {
        VoltageGraph graph = OctagonSquareVoltageGraph.build();

        Face squareFace = graph.getFaces().stream()
                .filter(f -> f.getCycleLength() == 4)
                .findFirst().orElseThrow();

        List<HalfEdge> boundary = squareFace.getBoundary();
        assertEquals(4, boundary.size());
        assertTrue(boundary.stream().allMatch(h -> h == boundary.get(0)));
        assertTrue(graph.validateCycle(boundary));
    }

    @Test
    @DisplayName("OctagonSquareVoltageGraph: the octagon face alternates two edges eight times over")
    void octagonSquareVoltageGraph_octagonFaceAlternatesTwoEdges() {
        VoltageGraph graph = OctagonSquareVoltageGraph.build();

        Face octagonFace = graph.getFaces().stream()
                .filter(f -> f.getCycleLength() == 8)
                .findFirst().orElseThrow();

        List<HalfEdge> boundary = octagonFace.getBoundary();
        assertEquals(8, boundary.size());
        assertEquals(2, Set.copyOf(boundary).size());
        assertTrue(graph.validateCycle(boundary));
    }
}
