package org.communicationModels.cycleBuildingComms;

import java.util.ArrayList;
import java.util.List;

import org.graphs.util.OrientedPoint;
import org.graphs.voltage.HalfEdge;
import org.graphs.voltage.Role;
import org.graphs.voltage.SquareVoltageGraph;
import org.graphs.voltage.VoltageGraph;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.robots.GeometricCycleLatticeRobot;
import org.utils.logging.OutgoingMessageRecord;
import org.utils.logging.TickRecord;

import static org.junit.jupiter.api.Assertions.*;

/**
 * <strong>These tests assert behaviour that is wrong.</strong>
 *
 * <p>They pin down the two defects the communication-tuple migration exists to fix, both
 * of which come from the same root cause: face completion is decided by <em>who</em> a
 * walk reaches rather than <em>whether the walk closed</em>. Until that changes, this is
 * what the protocol does, and a green test here means the defects are still present and
 * still reproducible.
 *
 * <p>They are written as passing tests rather than disabled ones on purpose. A disabled
 * test proves nothing and rots silently; a passing one that documents a defect keeps
 * running, so if some unrelated change alters the behaviour, that shows up immediately as
 * a failure to investigate rather than as an unnoticed drift.
 *
 * <p><strong>Both assertions invert when the closure predicate lands.</strong>
 * {@code rootRingDefersForever} becomes {@code rootRingClosesFaces}, and
 * {@code pathBetweenTwoRootsIsCertifiedAsFace} becomes
 * {@code pathBetweenTwoRootsIsRejected}. That inversion is the evidence the fix works --
 * so when they start failing, read this file before assuming something broke.
 */
class CycleClosureCharacterizationTest {

    /** Square: one role, four outgoing half-edges, one four-step face. */
    private static final VoltageGraph GRAPH = SquareVoltageGraph.build();

    private static HalfEdge firstOutgoingEdge() {
        Role role = GRAPH.getPrimaryRole();
        List<HalfEdge> outgoing = GRAPH.getOutgoingHalfEdges(role);
        assertFalse(outgoing.isEmpty(), "lattice has no outgoing half-edges");
        return outgoing.get(0);
    }

    /**
     * <strong>Defect 1 — root-to-root deferral never terminates.</strong>
     *
     * <p>A root receiving a {@code PositioningMessage} whose walk was initiated by the
     * sender checks whether the <em>next</em> edge's cycle is already complete, and if not
     * it emits {@code AttemptLaterMessage} back upstream. Put roots all the way around a
     * face and each defers to the next: nobody is ever the one whose next edge is already
     * done, so the deferral chases itself around the ring.
     *
     * <p>Asserted here as: no face reaches {@code complete}, and "Attempt Later" traffic
     * keeps being generated rather than dying out.
     */
    @Test
    @DisplayName("DEFECT: a ring of adjacent roots defers forever and closes nothing")
    void rootRingDefersForever() {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(GRAPH, firstOutgoingEdge(), new OrientedPoint(0, 0, 0));

        // Every robot on the face is a root, each trying to build the same face from its
        // own corner. This is reachable in the real simulation: promoteAdjacentVerticesToRoots
        // turns a completed root's neighbours into roots, and clicking a robot promotes it.
        for (GeometricCycleLatticeRobot robot : robots) {
            robot.promoteToPrimaryRoot();
        }

        List<TickRecord> records = LatticeHarness.tick(robots, 40);

        for (GeometricCycleLatticeRobot robot : robots) {
            assertFalse(LatticeHarness.anyCycleComplete(records, robot.getRobotId()),
                    "robot " + robot.getRobotId() + " completed a cycle. If the closure "
                            + "predicate has landed, this test should now be rootRingClosesFaces "
                            + "-- see the class javadoc.");
        }

        List<OutgoingMessageRecord> deferrals = LatticeHarness.messagesOfType(records, "Attempt Later");
        assertFalse(deferrals.isEmpty(),
                "expected roots to defer to one another; none did, so this scenario no longer "
                        + "reproduces defect 1 and the test needs rebuilding rather than deleting");
    }

    /**
     * <strong>Defect 2 — a path between two distinct roots is certified as a face.</strong>
     *
     * <p>A root reports SUCCESS purely because it is a root sitting at the right pose. It
     * never asks whether the walk that arrived is the walk it started. Combined with
     * {@code findBestNeighborForEdge} closing onto <em>any</em> neighbour already at the
     * target, a walk that starts at root A and ends at a different root D is accepted as a
     * closed face.
     *
     * <p>The walk here is a genuine face boundary and does return to the initiator's
     * <em>site</em>, which is exactly why the bug is invisible: the geometry is right and
     * the identity is not checked. What makes it a defect is that nothing in the protocol
     * would have noticed if the walk had ended somewhere else entirely.
     */
    @Test
    @DisplayName("DEFECT: closure is granted on role and pose, never on the walk returning")
    void pathBetweenTwoRootsIsCertifiedAsFace() {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(GRAPH, firstOutgoingEdge(), new OrientedPoint(0, 0, 0));

        GeometricCycleLatticeRobot initiator = robots.get(0);
        GeometricCycleLatticeRobot closer = robots.get(robots.size() - 1);

        // Two roots on one face, with two plain builders between them. The walk leaves the
        // initiator and arrives at a DIFFERENT root, which is the case the protocol cannot
        // currently distinguish from a real closure.
        initiator.promoteToPrimaryRoot();
        closer.promoteToPrimaryRoot();

        List<TickRecord> records = LatticeHarness.tick(robots, 40);

        boolean anyoneClosed = false;
        for (GeometricCycleLatticeRobot robot : robots) {
            anyoneClosed |= LatticeHarness.anyCycleComplete(records, robot.getRobotId());
        }

        assertTrue(anyoneClosed,
                "no cycle was marked complete. Either the closure predicate has landed -- in "
                        + "which case this test should now be pathBetweenTwoRootsIsRejected -- or "
                        + "the scenario stopped exercising the closing branch at all.");

        // The defect in one line: closure was granted without any robot checking that the
        // certificate came back to the robot that minted it.
        assertNotEquals(initiator.getRobotId(), closer.getRobotId(),
                "scenario error: the two roots must be distinct for this to be defect 2");
    }

    /**
     * A guard on the harness rather than on the protocol: if placement is wrong, every
     * test built on it is measuring the motion model instead of the message passing.
     */
    @Test
    @DisplayName("harness places every robot on its exact lattice site")
    void placementIsExact() {
        HalfEdge start = firstOutgoingEdge();
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(GRAPH, start, new OrientedPoint(0, 0, 0));

        assertEquals(start.getFace().getCycleLength(), robots.size(),
                "one robot per site, and no duplicate on the origin");

        List<HalfEdge> boundary = LatticeHarness.boundaryFrom(start);
        for (int i = 0; i < robots.size() - 1; i++) {
            OrientedPoint from = robots.get(i).getPosition();
            OrientedPoint to = robots.get(i + 1).getPosition();
            OrientedPoint offset = boundary.get(i).getVoltage().asPose();

            double expected = Math.hypot(offset.getX(), offset.getY());
            assertEquals(expected, from.distance(to), 1e-9,
                    "robots " + i + " and " + (i + 1) + " are not one edge apart");
        }

        // Mutual neighbours, so no test has to remember to wire them.
        for (GeometricCycleLatticeRobot robot : robots) {
            assertEquals(robots.size() - 1, robot.getNeighbors().size(),
                    "robot " + robot.getRobotId() + " is not wired to every other");
        }
    }

    /** Placement must work on every lattice, not just the one these defects use. */
    @Test
    @DisplayName("harness places a face on every shipped lattice")
    void placementWorksOnEveryLattice() {
        List<VoltageGraph> graphs = new ArrayList<>(List.of(
                SquareVoltageGraph.build(),
                org.graphs.voltage.TriangleVoltageGraph.build(),
                org.graphs.voltage.HexagonVoltageGraph.build(),
                org.graphs.voltage.OctagonSquareVoltageGraph.build(),
                org.graphs.voltage.SnubSquareVoltageGraph.build(),
                org.graphs.voltage.SnubHexagonVoltageGraph.build(),
                org.graphs.voltage.HexagonTriangleVoltageGraph.build(),
                org.graphs.voltage.HexagonSquareTriangleVoltageGraph.build(),
                org.graphs.voltage.DodecagonTriangleVoltageGraph.build(),
                org.graphs.voltage.DodecagonHexagonSquareVoltageGraph.build(),
                org.graphs.voltage.ElongatedTriangularVoltageGraph.build()));

        for (VoltageGraph graph : graphs) {
            for (org.graphs.voltage.Face face : graph.getFaces()) {
                HalfEdge start = face.getBoundary().get(0);
                List<GeometricCycleLatticeRobot> robots =
                        LatticeHarness.placeOnFace(graph, start, new OrientedPoint(0, 0, 0));

                assertEquals(face.getCycleLength(), robots.size(),
                        "face " + face.getId() + " of " + graph.getRoles().size() + "-role lattice");

                // No two robots on the same site -- that would mean the face doubles back,
                // which the tuple key cannot represent.
                for (int i = 0; i < robots.size(); i++) {
                    for (int j = i + 1; j < robots.size(); j++) {
                        assertTrue(robots.get(i).getPosition().distance(robots.get(j).getPosition()) > 1e-6,
                                "two robots placed on the same site, face " + face.getId());
                    }
                }
            }
        }
    }
}
