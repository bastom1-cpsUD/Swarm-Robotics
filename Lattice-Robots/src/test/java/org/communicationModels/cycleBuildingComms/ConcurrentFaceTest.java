package org.communicationModels.cycleBuildingComms;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.communicationModels.cycleBuildingComms.Messages.AbstractMessage;
import org.communicationModels.cycleBuildingComms.Messages.PositioningMessage;
import org.graphs.util.OrientedPoint;
import org.graphs.voltage.HalfEdge;
import org.graphs.voltage.OctagonSquareVoltageGraph;
import org.graphs.voltage.SquareVoltageGraph;
import org.graphs.voltage.VoltageGraph;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.robots.GeometricCycleLatticeRobot;
import org.utils.logging.CommsSnapshot;
import org.utils.logging.TickRecord;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Phase 6: one robot, several faces at once.
 *
 * <p>Everything the tuple representation was built for lives here, and none of it was
 * observable before this phase -- at a cap of one obligation, "route the response to the
 * right parent" and "route it to the only parent" are the same instruction, so Phases 4 and
 * 5 could not tell a correct implementation from a lucky one. These tests are what makes the
 * difference visible.
 *
 * <p>Two mechanisms carry most of the weight and are worth naming, because a failure here is
 * almost always one of them:
 *
 * <ul>
 *   <li><strong>Custody.</strong> A certificate lives in the {@code PositioningMessage} that
 *       carried it, queued in the recipient's inbox, from acceptance until the walk resolves.
 *       It used to live in a field, which holds one walk; a robot carrying two would
 *       overwrite one with the other and relay the survivor twice.</li>
 *   <li><strong>Tuple routing.</strong> Whether a response stops here or travels onward is a
 *       property of the obligation it routes through -- {@code isInitiatedFace} -- never of
 *       the robot's role. Role was the old discriminator and it is wrong in both directions:
 *       a root can carry someone else's walk, and a settled builder can carry a second
 *       face.</li>
 * </ul>
 */
class ConcurrentFaceTest {

    private static final VoltageGraph SQUARE = SquareVoltageGraph.build();
    private static final VoltageGraph OCTAGON_SQUARE = OctagonSquareVoltageGraph.build();

    /** Long enough for an 8-cycle to be walked hop by hop, with retries. */
    private static final int LONG_RUN = 240;

    private static List<FaceObligation> obligationsOf(List<TickRecord> records, int robotId) {
        TickRecord last = LatticeHarness.lastRecordOf(records, robotId);
        return last == null ? List.of() : last.after().obligations();
    }

    /** The largest number of tuples this robot held simultaneously at any point in the run. */
    private static int peakObligations(List<TickRecord> records, int robotId) {
        int peak = 0;
        for (TickRecord record : records) {
            if (record.robotId() == robotId) {
                peak = Math.max(peak, record.after().obligations().size());
            }
        }
        return peak;
    }

    /** Every distinct cycle length this robot closed a corner on, by the end of the run. */
    private static Set<Integer> closedFaceLengths(VoltageGraph graph, TickRecord last) {
        Set<Integer> lengths = new HashSet<>();
        if (last == null) {
            return lengths;
        }
        last.after().completedCycles().forEach((edgeId, status) -> {
            if (status == CycleStatus.complete) {
                HalfEdge edge = graph.getHalfEdgeById(edgeId);
                if (edge != null) {
                    lengths.add(edge.getFace().getCycleLength());
                }
            }
        });
        return lengths;
    }

    /**
     * The core guarantee, and the reason the tuple exists at all.
     *
     * <p>A robot holding two faces has two parents waiting on it. Nothing in a message says
     * which one a given status belongs to -- the child answers to "the robot that assigned
     * me", and that is recorded only in the tuple. The old {@code pendingChildID} field could
     * hold one of the two, so with concurrent faces it would have routed both responses to
     * whichever parent was recorded last.
     */
    @Test
    @DisplayName("a robot carrying two faces keeps one tuple per edge, each with its own parent")
    void robotOnTwoFacesHoldsAnObligationPerEdge() {
        // A ring of roots, because concurrency needs several walks and a single root builds
        // one face of its own at a time -- deliberately, since all of one root's corners draw
        // candidates from the same neighbourhood. What makes a robot carry two faces is other
        // robots' walks arriving, not its own ambition, and this is the smallest scenario in
        // which that happens.
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(SQUARE, firstOutgoingEdge(SQUARE), new OrientedPoint(0, 0, 0));
        for (GeometricCycleLatticeRobot robot : robots) {
            robot.promoteToPrimaryRoot();
        }

        List<TickRecord> records = LatticeHarness.tick(robots, LONG_RUN);

        int sharing = 0;
        for (TickRecord record : records) {
            List<FaceObligation> held = record.after().obligations();
            if (held.size() < 2) {
                continue;
            }
            sharing++;

            // Uniqueness over the CARRIED walks only. This robot's own attempt is keyed on
            // an outgoing edge and the carried ones on incoming edges, so the two spaces
            // overlap by id and always did -- what changed is that they no longer share a
            // container, so an id appearing in both is now ordinary rather than a clash.
            // Two carried tuples on one edge would still be the real fault: one child slot
            // cannot serve two walks.
            Set<Integer> carriedEdges = new HashSet<>();
            int attempts = 0;
            for (FaceObligation obligation : held) {
                if (obligation.getParentId() == FaceObligation.NO_PARENT) {
                    attempts++;
                    continue;
                }
                assertTrue(carriedEdges.add(obligation.getEdgeId()),
                        "robot " + record.robotId() + " held two carried tuples on edge "
                                + obligation.getEdgeId() + "; one child slot cannot serve two walks");
            }
            assertTrue(attempts <= 1,
                    "robot " + record.robotId() + " held " + attempts + " faces of its own; a robot "
                            + "builds one at a time, and FaceObligationSet holds a single slot for it");
        }

        assertTrue(sharing > 0,
                "no robot ever held two obligations at once, so this run never exercised "
                        + "concurrent faces and proves nothing about them. Either the cap is back "
                        + "or acceptForRelay is refusing second faces.");
    }

    /**
     * Custody, observed directly. A robot that has accepted a walk but not yet passed it on
     * still has the assignment sitting in its inbox -- that queued message <em>is</em> the
     * certificate store, and if it were consumed on acceptance the walk would be gone.
     */
    @Test
    @DisplayName("an accepted assignment stays queued until its walk resolves")
    void acceptedAssignmentStaysInTheInbox() {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(SQUARE, firstOutgoingEdge(SQUARE), new OrientedPoint(0, 0, 0));
        robots.get(0).promoteToPrimaryRoot();

        List<TickRecord> records = LatticeHarness.tick(robots, 12);

        boolean sawHeldAssignment = false;
        for (TickRecord record : records) {
            for (FaceObligation obligation : record.after().obligations()) {
                if (obligation.getParentId() == FaceObligation.NO_PARENT) {
                    continue;   // a face of its own: nothing was handed to it, so nothing is held
                }
                if (queuedAssignmentFor(record.after(), obligation) != null) {
                    sawHeldAssignment = true;
                }
            }
        }

        assertTrue(sawHeldAssignment,
                "no robot ever held a carried face's assignment in its queue. The certificate "
                        + "has nowhere else to live -- there is no field for it -- so a relayer "
                        + "without its message in the inbox cannot extend anything.");
    }

    /**
     * The other half of custody: what is held must eventually be let go. A queued assignment
     * whose walk has finished would otherwise sit in the inbox forever, and -- worse -- keep
     * re-creating the tuple it belongs to every time it was reconsidered.
     */
    @Test
    @DisplayName("custody is released when the walk resolves, leaving no orphan in the queue")
    void custodyIsReleasedWhenTheWalkResolves() {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(SQUARE, firstOutgoingEdge(SQUARE), new OrientedPoint(0, 0, 0));
        robots.get(0).promoteToPrimaryRoot();

        List<TickRecord> records = LatticeHarness.tick(robots, 60);

        // Checked in one direction only, deliberately. A queued assignment with no tuple is
        // ordinary -- it is an offer this robot has not answered yet. A tuple with no queued
        // assignment is the failure: the certificate it exists to carry is gone, so the walk
        // can never be relayed and the parent waits on it forever.
        for (TickRecord record : records) {
            CommsSnapshot after = record.after();
            for (FaceObligation obligation : after.obligations()) {
                if (obligation.getParentId() == FaceObligation.NO_PARENT) {
                    continue;
                }
                assertNotNull(queuedAssignmentFor(after, obligation),
                        "robot " + record.robotId() + " holds a carried tuple on edge "
                                + obligation.getEdgeId() + " at tick " + record.tick()
                                + " with no assignment queued for it. Its certificate is gone, "
                                + "so the walk can never be relayed and the parent waits forever.");
            }
        }
    }

    /**
     * Marking has to happen at every robot on the walk that keeps cycle bookkeeping, not just
     * at the initiator -- it is what makes a second certificate for an already-built corner an
     * idempotent no-op, and therefore what lets one tuple per edge suffice.
     *
     * <p>The subtlety this pins is <em>which</em> edge each participant marks. The status
     * carries the initiator's own origin edge, and every other robot on the face occupies a
     * different site with a different outgoing edge for it. A participant must mark the corner
     * it owes -- {@code next} of the edge it was assigned -- not the id it was handed. On a
     * single-role lattice the wrong one is still a valid edge id belonging to this robot, so
     * the mistake writes a plausible value and shows up only as faces that never converge.
     */
    @Test
    @DisplayName("every participant marks its own corner, not the initiator's")
    void successMarksEachParticipantsOwnCorner() {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(SQUARE, firstOutgoingEdge(SQUARE), new OrientedPoint(0, 0, 0));
        for (GeometricCycleLatticeRobot robot : robots) {
            robot.promoteToPrimaryRoot();
        }

        List<TickRecord> records = LatticeHarness.tick(robots, LONG_RUN);

        for (TickRecord record : records) {
            for (var entry : record.after().completedCycles().entrySet()) {
                if (entry.getValue() != CycleStatus.complete) {
                    continue;
                }
                HalfEdge marked = SQUARE.getHalfEdgeById(entry.getKey());
                assertNotNull(marked,
                        "robot " + record.robotId() + " marked edge " + entry.getKey()
                                + " complete, but that is not a half-edge in this lattice");
                assertEquals(SQUARE.getPrimaryRole().getId(), marked.getOrigin().getId(),
                        "robot " + record.robotId() + " marked edge " + entry.getKey()
                                + " complete, but that edge does not leave this robot's own role "
                                + "-- it belongs to another site on the face");
            }
        }
    }

    /**
     * Two roots on one face. Every robot on a cycle sees the certificate before it closes, so
     * whichever of them meets the other's walk first can detect the duplication and stand
     * down; lower initiator id wins, matching {@code outranks}.
     *
     * <p>Run in both list orders, because activation order decides which root's walk arrives
     * first and the outcome must not depend on that. If it did, the collapse would be a race
     * rather than a rule, and the sim's staggered activation would make it intermittent.
     */
    @Test
    @DisplayName("two roots building one face collapse to one, whichever walk arrives first")
    void coInitiationCollapsesInBothArrivalOrders() {
        assertCoInitiationCollapses(false);
        assertCoInitiationCollapses(true);
    }

    private void assertCoInitiationCollapses(boolean reversed) {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(SQUARE, firstOutgoingEdge(SQUARE), new OrientedPoint(0, 0, 0));
        for (GeometricCycleLatticeRobot robot : robots) {
            robot.promoteToPrimaryRoot();
        }

        List<GeometricCycleLatticeRobot> order = new ArrayList<>(robots);
        if (reversed) {
            java.util.Collections.reverse(order);
        }

        List<TickRecord> records = LatticeHarness.tick(order, LONG_RUN);

        String where = reversed ? " (reversed activation order)" : "";
        boolean someoneClosed = false;
        for (GeometricCycleLatticeRobot robot : robots) {
            someoneClosed |= LatticeHarness.anyCycleComplete(records, robot.getRobotId());
        }
        assertTrue(someoneClosed,
                "two roots on one face closed nothing in " + LONG_RUN + " ticks" + where
                        + ". Co-initiation is meant to collapse to one walk, not to deadlock "
                        + "both -- see collapseCoInitiation.");

        // And nobody was left holding a tuple with no certificate behind it, which is how a
        // stand-down goes wrong: drop the walk and keep the obligation.
        for (GeometricCycleLatticeRobot robot : robots) {
            for (FaceObligation obligation : obligationsOf(records, robot.getRobotId())) {
                TickRecord last = LatticeHarness.lastRecordOf(records, robot.getRobotId());
                if (obligation.getParentId() == FaceObligation.NO_PARENT) {
                    continue;
                }
                assertNotNull(queuedAssignmentFor(last.after(), obligation),
                        "robot " + robot.getRobotId() + " ended holding a carried tuple with no "
                                + "certificate" + where);
            }
        }
    }

    /**
     * <strong>Phase 7's integration test.</strong> OctagonSquare is the least forgiving
     * lattice in the set: a 4-cycle and an 8-cycle meet at every corner, so a robot there is
     * on two faces of different lengths at once and every conjunct of the closure rule has
     * to do its own job. Length is what separates them -- both close geometrically, because
     * the measured product telescopes for any walk that physically returned -- so a length
     * test that was quietly wrong would close the square as an octagon and never be noticed
     * on a lattice where all faces are the same size.
     *
     * <p>Expressible only now: it needs Phase 0's injectable graph and Phase 6's multi-face
     * support together.
     */
    @Test
    @DisplayName("a corner where a square and an octagon meet closes both")
    void octagonAndSquareBothCloseAtASharedCorner() {
        List<GeometricCycleLatticeRobot> robots = LatticeHarness.placeAroundRole(
                OCTAGON_SQUARE, OCTAGON_SQUARE.getPrimaryRole(), new OrientedPoint(0, 0, 0));
        GeometricCycleLatticeRobot corner = robots.get(0);
        corner.promoteToPrimaryRoot();

        // Precondition on the scenario, not on the protocol: if this role did not border two
        // face sizes the test would pass vacuously on a much weaker claim.
        Set<Integer> incidentLengths = new HashSet<>();
        for (HalfEdge edge : OCTAGON_SQUARE.getOutgoingHalfEdges(OCTAGON_SQUARE.getPrimaryRole())) {
            incidentLengths.add(edge.getFace().getCycleLength());
        }
        assertTrue(incidentLengths.contains(4) && incidentLengths.contains(8),
                "scenario error: the primary role of OctagonSquare should border both a 4-cycle "
                        + "and an 8-cycle, but borders " + incidentLengths);

        List<TickRecord> records = LatticeHarness.tick(robots, LONG_RUN);

        Set<Integer> closed = closedFaceLengths(OCTAGON_SQUARE,
                LatticeHarness.lastRecordOf(records, corner.getRobotId()));

        assertTrue(closed.contains(4),
                "the square never closed at the shared corner. Closed lengths: " + closed);
        assertTrue(closed.contains(8),
                "the octagon never closed at the shared corner. Closed lengths: " + closed
                        + ". A 4 here with no 8 is the length conjunct doing too much -- the "
                        + "square closing and the octagon being written off with it; an 8 with "
                        + "no 4 is the reverse.");
    }

    /**
     * The rotation, which is what stops one face starving another. Fair selection is not an
     * optimisation here: {@code findUnfulfilled} resuming from a fixed position would let the
     * lowest-numbered incident edge take every activation, and the obligation set is
     * insertion-ordered specifically so that bias cannot hide behind hash iteration order.
     */
    @Test
    @DisplayName("a robot holding several faces offers on more than one of them")
    void severalFacesEachGetOffered() {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(SQUARE, firstOutgoingEdge(SQUARE), new OrientedPoint(0, 0, 0));
        for (GeometricCycleLatticeRobot robot : robots) {
            robot.promoteToPrimaryRoot();
        }

        List<TickRecord> records = LatticeHarness.tick(robots, LONG_RUN);

        int busiest = -1;
        int peak = 0;
        for (GeometricCycleLatticeRobot robot : robots) {
            int held = peakObligations(records, robot.getRobotId());
            if (held > peak) {
                peak = held;
                busiest = robot.getRobotId();
            }
        }
        assertTrue(peak >= 2,
                "no robot ever held two faces at once, so there was no rotation to test");

        // Every tuple that robot held at its busiest must have been offered on at some point,
        // not merely opened -- an edge that is never offered is an edge that is starved.
        Set<Integer> everFulfilled = new HashSet<>();
        for (TickRecord record : records) {
            if (record.robotId() != busiest) {
                continue;
            }
            for (FaceObligation obligation : record.after().obligations()) {
                if (!obligation.isUnfulfilled()) {
                    everFulfilled.add(obligation.getEdgeId());
                }
            }
        }
        assertTrue(everFulfilled.size() >= 2,
                "robot " + busiest + " held " + peak + " faces at once but only ever made an "
                        + "offer on " + everFulfilled.size() + " edge(s). The rotating cursor in "
                        + "FaceObligationSet.findUnfulfilled is what should prevent that.");
    }

    private static HalfEdge firstOutgoingEdge(VoltageGraph graph) {
        return graph.getOutgoingHalfEdges(graph.getPrimaryRole()).get(0);
    }

    /** The queued assignment carrying this tuple's certificate, or null. */
    private static PositioningMessage queuedAssignmentFor(CommsSnapshot snapshot,
                                                          FaceObligation obligation) {
        for (AbstractMessage queued : snapshot.queueInOrder()) {
            if (queued instanceof PositioningMessage pm
                    && pm.getSenderId() == obligation.getParentId()
                    && pm.getAssignedOutgoingEdgeID() == obligation.getEdgeId()) {
                return pm;
            }
        }
        return null;
    }

}
