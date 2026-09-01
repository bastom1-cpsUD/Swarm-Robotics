package org.communicationModels.cycleBuildingComms;

import org.communicationModels.cycleBuildingComms.Messages.PositioningMessage;
import org.communicationModels.cycleBuildingComms.Messages.VoltageCertificate;
import org.graphs.util.OrientedPoint;
import org.graphs.util.RigidBodyTransformation;
import org.graphs.voltage.HalfEdge;
import org.graphs.voltage.SquareVoltageGraph;
import org.graphs.voltage.VoltageGraph;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.robots.GeometricCycleLatticeRobot;
import org.utils.logging.TickRecord;

import static org.junit.jupiter.api.Assertions.*;

/**
 * A robot still driving to its site answers nothing until it is standing still.
 *
 * <p>The reason is honesty, not courtesy to the offerer. Every question an assignment asks is
 * answered by {@code checkAssignmentForCurrentPosition} against where the robot is <em>right
 * now</em>, so a moving robot's "no" describes a pose it is about to leave — and that "no" is
 * permanent. Every rejection in the assignment path is non-retryable, so the offerer writes the
 * rejecter onto that face's ban list, {@link FaceObligation} never lifts a ban, and an offerer
 * that runs through its neighbours kills the face outright: a root's corner goes {@code failed},
 * or a carried walk sends a FAILURE all the way back to its initiator.
 *
 * <p>Measured on {@code sim-2026-08-29T21-43-00}, the current-code run: robot 78 offered one
 * edge to eight robots over ticks 211-248, banned every one, and ended holding no child at all.
 * Three of the eight were mid-journey when they refused, and three more tore their own
 * assignment down inside the same activation in which they refused — a robot going
 * <em>unassigned</em> being precisely the robot that could have taken the site. Only four of the
 * eight refusals were durable facts.
 *
 * <p>The harness does not run the motion model — {@code robot.move(dt)} is a separate task in
 * the simulation, and every scenario starts exactly placed so tests are about message passing.
 * So "travelling" is staged by displacing a robot and "arrived" by putting it back, the same way
 * {@link PromotionTest} stages the promotion guard.
 */
class InTransitDeferralTest {

    private static final VoltageGraph SQUARE = SquareVoltageGraph.build();

    private static HalfEdge firstOutgoingEdge() {
        return SQUARE.getOutgoingHalfEdges(SQUARE.getPrimaryRole()).get(0);
    }

    /**
     * Every robot here is built on {@link #SQUARE} explicitly.
     *
     * <p>The two-argument constructor binds to {@code GeometricCycleLatticeRobot.GRAPH}, which
     * is whichever lattice the simulation is currently configured for — SnubSquare today. Half-
     * edge ids overlap across lattices while their voltages do not, so a robot resolving edge 0
     * in one graph while the test computes its site from edge 0 in another silently disagrees
     * about where "the target" is, and the robot simply never arrives.
     */
    private static GeometricCycleLatticeRobot robotAt(int id, OrientedPoint pose) {
        return new GeometricCycleLatticeRobot(id, pose, SQUARE);
    }

    /** Where a robot at {@code from} is pointing whoever it hands {@code edge}. */
    private static OrientedPoint targetOf(OrientedPoint from, HalfEdge edge) {
        return new RigidBodyTransformation(from).compose(edge.getVoltage()).asPose();
    }

    private static OrientedPoint offset(OrientedPoint base, double dx, double dy) {
        return new OrientedPoint(base.getX() + dx, base.getY() + dy, base.getOrientation());
    }

    private static void wireAsNeighbors(GeometricCycleLatticeRobot... robots) {
        for (GeometricCycleLatticeRobot a : robots) {
            for (GeometricCycleLatticeRobot b : robots) {
                if (a.getRobotId() != b.getRobotId()) {
                    a.addNeighbor(b);
                }
            }
        }
    }

    private static PositioningMessage offerOf(int from, int to, HalfEdge edge) {
        int vertexId = edge.getOrigin().getId();
        return new PositioningMessage(from, to, vertexId, edge.getId(), vertexId, edge.getId(),
                new VoltageCertificate(from));
    }

    /** The wire name for a RejectAssignmentMessage, as OutgoingMessageRecord records it. */
    private static final String REJECTION = "Rejection";

    private static boolean sentAnyOfType(TickRecord record, String messageType) {
        return record.sent().stream().anyMatch(m -> messageType.equals(m.messageType()));
    }

    /**
     * The deferral, and — as load-bearing as the deferral itself — that arrival ends it.
     *
     * <p>An offerer has no timeout on a filled child slot: {@code collectDepartedChildren} only
     * fires when a child stops being observable, and a robot driving about inside comm range
     * never does. So a deferral that did not release would hang the offerer's face forever,
     * which is strictly worse than the wrong answer it replaces.
     */
    @Test
    @DisplayName("a robot in transit leaves an offer queued, and answers it once it has arrived")
    void inTransitLeavesTheOfferQueuedThenAnswersOnArrival() {
        HalfEdge edge = firstOutgoingEdge();
        OrientedPoint parentPose = new OrientedPoint(0, 0, 0);
        OrientedPoint site = targetOf(parentPose, edge);

        GeometricCycleLatticeRobot parent = robotAt(100, parentPose);
        // Off its site: far enough that the arrival check cannot pass, near enough to stay in
        // comm range of both the parent and the bystander.
        GeometricCycleLatticeRobot traveller = robotAt(1, offset(site, 0, 22));
        GeometricCycleLatticeRobot bystander = robotAt(2, offset(site, 30, 0));
        wireAsNeighbors(parent, traveller, bystander);

        // Accept an assignment it has to drive to.
        traveller.enqueueMessage(offerOf(parent.getRobotId(), traveller.getRobotId(), edge));
        TickRecord accepted = traveller.executeTimeStep(1.0, 1);
        assertEquals(CycleRole.cycleBuilder, accepted.after().role(),
                "scenario error: the traveller never took the assignment, so it is not in transit");
        assertTrue(traveller.isMovingToAssignedPosition(),
                "scenario error: the traveller already considers itself arrived, so nothing here "
                        + "is in transit and this test would pass without exercising the gate");

        // A second robot now offers it a site while it is still driving.
        traveller.enqueueMessage(offerOf(bystander.getRobotId(), traveller.getRobotId(), edge));
        int queuedBefore = accepted.after().queueInOrder().size() + 1;

        TickRecord whileMoving = traveller.executeTimeStep(1.0, 2);

        assertFalse(sentAnyOfType(whileMoving, REJECTION),
                "a robot still driving to its site answered an offer: " + whileMoving.sent()
                        + ". That answer is about a pose it is about to leave, and it is permanent "
                        + "-- every rejection in the assignment path is non-retryable, so the "
                        + "offerer bans it on that face for good and may run out of candidates and "
                        + "kill the face while this robot was only early.");
        assertEquals(queuedBefore, whileMoving.after().queueInOrder().size(),
                "the offer was consumed in transit rather than left queued. Processed: "
                        + whileMoving.processedDescription());

        // Arrival releases it. More than one activation, because processMessages runs before
        // sendMessage and hasArrived latches in the latter -- the same lag PromotionTest documents.
        traveller.setPosition(new OrientedPoint(site));
        boolean answered = false;
        for (int tick = 3; tick <= 8 && !answered; tick++) {
            answered = sentAnyOfType(traveller.executeTimeStep(1.0, tick), REJECTION);
        }
        assertTrue(answered,
                "the deferred offer was never answered, even once the robot was standing on its "
                        + "site. Deferral is only acceptable because arrival ends it: an offerer "
                        + "has no timeout on a filled child slot -- collectDepartedChildren needs "
                        + "the child to leave comm range -- so a deferral that never releases "
                        + "hangs that face forever.");
    }
}
