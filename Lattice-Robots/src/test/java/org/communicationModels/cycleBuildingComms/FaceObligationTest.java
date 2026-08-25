package org.communicationModels.cycleBuildingComms;

import java.util.List;
import java.util.stream.Stream;

import org.graphs.voltage.Face;
import org.graphs.voltage.HalfEdge;
import org.graphs.voltage.Role;
import org.graphs.voltage.VoltageGraph;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.simulation.Edge;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Pure unit tests for one communication tuple. No robots, no ticks, no lattice simulation
 * beyond reading a graph -- which is the reason {@link FaceObligation} is a top-level
 * class rather than an inner class of the comms system.
 */
class FaceObligationTest {

    static Stream<Arguments> lattices() {
        return Stream.of(
                Arguments.of("Square", org.graphs.voltage.SquareVoltageGraph.build()),
                Arguments.of("Triangle", org.graphs.voltage.TriangleVoltageGraph.build()),
                Arguments.of("Hexagon", org.graphs.voltage.HexagonVoltageGraph.build()),
                Arguments.of("OctagonSquare", org.graphs.voltage.OctagonSquareVoltageGraph.build()),
                Arguments.of("SnubSquare", org.graphs.voltage.SnubSquareVoltageGraph.build()),
                Arguments.of("SnubHexagon", org.graphs.voltage.SnubHexagonVoltageGraph.build()),
                Arguments.of("HexagonTriangle", org.graphs.voltage.HexagonTriangleVoltageGraph.build()),
                Arguments.of("HexagonSquareTriangle", org.graphs.voltage.HexagonSquareTriangleVoltageGraph.build()),
                Arguments.of("DodecagonTriangle", org.graphs.voltage.DodecagonTriangleVoltageGraph.build()),
                Arguments.of("DodecagonHexagonSquare", org.graphs.voltage.DodecagonHexagonSquareVoltageGraph.build()),
                Arguments.of("ElongatedTriangular", org.graphs.voltage.ElongatedTriangularVoltageGraph.build()));
    }

    @Test
    @DisplayName("a new obligation has no child and is outstanding")
    void newObligationHasNullChildAndIsUnfulfilled() {
        FaceObligation obligation = new FaceObligation(7, 3);

        assertEquals(7, obligation.getParentId());
        assertEquals(3, obligation.getEdgeId());
        assertNull(obligation.getChildId());
        assertNull(obligation.getChildEdge());
        assertTrue(obligation.isUnfulfilled());
    }

    @Test
    @DisplayName("fulfil records the child and clears outstanding")
    void fulfilSetsChildAndClearsUnfulfilled() {
        FaceObligation obligation = new FaceObligation(7, 3);
        Edge drawn = new Edge(7, 9);

        obligation.fulfil(9, drawn);

        assertEquals(9, obligation.getChildId());
        assertSame(drawn, obligation.getChildEdge());
        assertFalse(obligation.isUnfulfilled());
    }

    /**
     * The rejection semantics. A rejection must leave the obligation able to re-offer,
     * which means keeping the parent it routes to, the edge it owes, and every ban it has
     * accumulated -- otherwise the next offer goes straight back to the robot that just
     * declined.
     */
    @Test
    @DisplayName("release clears the child but keeps parent, edge and bans")
    void releaseClearsChildButKeepsParentAndEdge() {
        FaceObligation obligation = new FaceObligation(7, 3);
        Edge drawn = new Edge(7, 9);
        obligation.fulfil(9, drawn);
        obligation.ban(9);

        Edge released = obligation.release();

        assertSame(drawn, released, "release must hand back the edge so the caller can undraw it");
        assertNull(obligation.getChildId());
        assertNull(obligation.getChildEdge());
        assertTrue(obligation.isUnfulfilled());
        assertEquals(7, obligation.getParentId());
        assertEquals(3, obligation.getEdgeId());
        assertTrue(obligation.isBanned(9), "a released child must stay banned, or it is re-offered at once");
    }

    /**
     * Structural guard. Giving the obligation a certificate field is the obvious fix for a
     * robot that accepts a walk it must travel to, and it silently breaks one-tuple-per-edge
     * as soon as two certificates target the same edge before either is relayed. The
     * certificate belongs in the message and in the inbox, never here.
     */
    @Test
    @DisplayName("an obligation exposes no certificate")
    void obligationHoldsNoCertificate() {
        boolean exposesCertificate = java.util.Arrays.stream(FaceObligation.class.getDeclaredFields())
                .anyMatch(field -> field.getType().getSimpleName().contains("Certificate"));

        assertFalse(exposesCertificate,
                "FaceObligation gained a certificate field. Certificates travel in messages and "
                        + "ride back on rejections and statuses; storing one here breaks the "
                        + "one-tuple-per-edge rule when two walks share an edge.");
    }

    @Test
    @DisplayName("matchesChild is false for other children, and safe when unfulfilled")
    void matchesChildIsFalseForOtherChildren() {
        FaceObligation obligation = new FaceObligation(7, 3);

        // The unfulfilled case is the one that used to throw: Integer unboxed against an
        // int is a null dereference, and findByChild walks past unfulfilled entries.
        assertDoesNotThrow(() -> obligation.matchesChild(9));
        assertFalse(obligation.matchesChild(9));

        obligation.fulfil(9, null);
        assertTrue(obligation.matchesChild(9));
        assertFalse(obligation.matchesChild(10));
    }

    /**
     * Ids above 127 fall outside the {@code Integer.valueOf} cache. A reference comparison
     * on the boxed child would pass for small ids and fail here, which is exactly the kind
     * of defect that survives every small test.
     */
    @Test
    @DisplayName("child matching and equality hold for ids beyond the Integer cache")
    void childIdentityHoldsAboveTheIntegerCache() {
        FaceObligation a = new FaceObligation(7, 3);
        FaceObligation b = new FaceObligation(7, 3);
        a.fulfil(5000, null);
        b.fulfil(5000, null);

        assertTrue(a.matchesChild(5000));
        assertEquals(a, b, "two obligations with the same large child id must compare equal");
        assertEquals(a.hashCode(), b.hashCode());
    }

    @Test
    @DisplayName("bans are independent across obligations")
    void banListsAreIndependentAcrossObligations() {
        FaceObligation faceA = new FaceObligation(7, 3);
        FaceObligation faceB = new FaceObligation(7, 4);

        faceA.ban(9);

        assertTrue(faceA.isBanned(9));
        assertFalse(faceB.isBanned(9), "a ban on one face must say nothing about another");
    }

    @Test
    @DisplayName("bans do not accumulate duplicates and are exposed read-only")
    void banListIsASetAndUnmodifiable() {
        FaceObligation obligation = new FaceObligation(7, 3);
        obligation.ban(9);
        obligation.ban(9);
        obligation.ban(11);

        assertEquals(2, obligation.getBans().size());
        assertThrows(UnsupportedOperationException.class, () -> obligation.getBans().add(13));
    }

    @ParameterizedTest(name = "{0}")
    @MethodSource("lattices")
    @DisplayName("faceId is stable for an edge, and several edges may share one face")
    void faceIdIsStableForAnEdgeAcrossAllLattices(String name, VoltageGraph graph) {
        for (Role role : graph.getRoles()) {
            for (HalfEdge edge : graph.getOutgoingHalfEdges(role)) {
                FaceObligation obligation = new FaceObligation(0, edge.getId());

                int first = obligation.faceId(graph);
                assertEquals(first, obligation.faceId(graph), name + ": faceId is not stable");
                assertEquals(edge.getFace().getId(), first, name + ": faceId disagrees with the graph");
            }
        }

        // The reason the edge key survives and a face key would not: Face is a face TYPE.
        // On Square all four of the role's outgoing edges share one face id, so rekeying
        // completedCycles from edge to face would collapse four corners into one.
        List<Face> faces = graph.getFaces();
        int edgeCount = 0;
        for (Role role : graph.getRoles()) {
            edgeCount += graph.getOutgoingHalfEdges(role).size();
        }
        assertTrue(edgeCount >= faces.size(),
                name + ": fewer half-edges than faces, which would make the edge key coarser "
                        + "than the face key -- the opposite of what the design assumes");
    }
}
