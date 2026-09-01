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
 * <strong>These two tests used to assert behaviour that was wrong. Phase 5 inverted them,
 * and the inversion is the evidence the fix works.</strong>
 *
 * <p>Through Phases 0-4 they pinned down two defects, both from one root cause: face
 * completion was decided by <em>who</em> a walk reached rather than <em>whether the walk
 * closed</em>. They were written as passing tests rather than disabled ones precisely so
 * this moment would be a visible edit rather than a silent one -- a disabled test proves
 * nothing and rots, whereas flipping a green assertion is a deliberate act with a diff
 * attached.
 *
 * <p>What replaced the defects, in both cases, is the closure predicate: a face closes when
 * a certificate returns to the robot that <em>minted</em> it, having taken exactly the
 * face's cycle length in hops, with the closed product the identity. Nothing about role or
 * pose enters into it.
 *
 * <ul>
 *   <li>{@code rootRingDefersForever} became {@link #rootRingClosesFaces()}. Roots no
 *       longer ask whether their own next edge is complete before carrying a neighbour's
 *       walk -- they simply carry it -- so the ring closes instead of deferring around
 *       itself forever.</li>
 *   <li>{@code pathBetweenTwoRootsIsCertifiedAsFace} became
 *       {@link #pathBetweenTwoRootsIsRelayedNotCertified()}. A root that is not the
 *       initiator relays, so the three-hop path is no longer mistaken for a four-hop
 *       face.</li>
 * </ul>
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
     * <strong>Defect 1, fixed — a ring of adjacent roots now closes its face.</strong>
     *
     * <p>The old branch, on seeing a walk initiated by the sender, asked whether its own
     * next edge was already complete and emitted {@code AttemptLaterMessage} upstream if
     * not. Around a ring of roots nobody is ever the one whose next edge is already done,
     * so the deferral chased itself forever.
     *
     * <p>A root now carries a walk it did not initiate without asking anything about its own
     * progress -- only the initiator may judge a certificate. The second assertion is the
     * load-bearing one: deferral is gone as a mechanism, not merely unused in this scenario,
     * because {@code AttemptLaterMessage} no longer exists to be sent.
     */
    @Test
    @DisplayName("a ring of adjacent roots closes its face instead of deferring")
    void rootRingClosesFaces() {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(GRAPH, firstOutgoingEdge(), new OrientedPoint(0, 0, 0));

        // Every robot on the face is a root, each trying to build the same face from its
        // own corner. This is reachable in the real simulation: promoteAdjacentVerticesToRoots
        // turns a completed root's neighbours into roots, and clicking a robot promotes it.
        for (GeometricCycleLatticeRobot robot : robots) {
            robot.promoteToPrimaryRoot();
        }

        List<TickRecord> records = LatticeHarness.tick(robots, 40);

        boolean anyoneClosed = false;
        for (GeometricCycleLatticeRobot robot : robots) {
            anyoneClosed |= LatticeHarness.anyCycleComplete(records, robot.getRobotId());
        }
        assertTrue(anyoneClosed,
                "a ring of roots closed nothing in 40 ticks. This is defect 1 back: a root "
                        + "must carry a walk it did not initiate rather than waiting on its own "
                        + "progress first -- see the class javadoc.");

        List<OutgoingMessageRecord> deferrals = LatticeHarness.messagesOfType(records, "Attempt Later");
        assertTrue(deferrals.isEmpty(),
                "an 'Attempt Later' message was sent. That message type was deleted with the "
                        + "closure predicate, and its only bound on a wandering walk was replaced "
                        + "by the hop cap; if it is back, so is the deferral loop.");
    }

    /**
     * <strong>Defect 2, fixed — a root that did not mint the certificate relays it.</strong>
     *
     * <p>The old code reported SUCCESS purely because it was a root sitting at the right
     * pose, so a walk that left root A and arrived at a different root D was certified as a
     * closed face -- here, a <em>three</em>-hop path accepted as a four-hop square. The
     * geometry was right and the identity was never checked, which is exactly why the bug
     * was invisible.
     *
     * <p>Closure now requires the certificate to come home to its minter. Robot 3 is a root
     * standing on the right site with the right pose and is <em>still</em> not entitled to
     * decide the walk, so it carries it one more hop, to the initiator, where the four-hop
     * face closes properly. The assertion that matters is the relay: an Assignment leaving
     * robot 3 for robot 0 is the walk continuing past the robot that used to terminate it.
     */
    @Test
    @DisplayName("a walk reaching a different root is relayed onward, not certified")
    void pathBetweenTwoRootsIsRelayedNotCertified() {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(GRAPH, firstOutgoingEdge(), new OrientedPoint(0, 0, 0));

        GeometricCycleLatticeRobot initiator = robots.get(0);
        GeometricCycleLatticeRobot farRoot = robots.get(robots.size() - 1);

        // Two roots on one face, with two plain builders between them. The walk leaves the
        // initiator and arrives at a DIFFERENT root -- the case the old protocol could not
        // distinguish from a real closure.
        initiator.promoteToPrimaryRoot();
        farRoot.promoteToPrimaryRoot();
        assertNotEquals(initiator.getRobotId(), farRoot.getRobotId(),
                "scenario error: the two roots must be distinct for this to be defect 2");

        List<TickRecord> records = LatticeHarness.tick(robots, 40);

        // The far root passed the walk on rather than ending it. Nothing else in this
        // scenario makes robot 3 assign robot 0 -- it is the closing hop of the face.
        boolean relayedToInitiator = false;
        for (TickRecord record : records) {
            if (record.robotId() != farRoot.getRobotId()) {
                continue;
            }
            for (OutgoingMessageRecord sent : record.sent()) {
                if (sent.messageType().equals("Assignment")
                        && sent.recipientId() == initiator.getRobotId()) {
                    relayedToInitiator = true;
                }
            }
        }
        assertTrue(relayedToInitiator,
                "root " + farRoot.getRobotId() + " never relayed the walk to its initiator. "
                        + "Either it certified the three-hop path itself -- defect 2 -- or it "
                        + "deferred, which is defect 1 wearing the other hat.");

        // And the face really did close, at the robot that minted the certificate.
        assertTrue(LatticeHarness.anyCycleComplete(records, initiator.getRobotId()),
                "the walk was relayed but the initiator never marked a corner complete, so the "
                        + "certificate did not survive the extra hop back");
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
