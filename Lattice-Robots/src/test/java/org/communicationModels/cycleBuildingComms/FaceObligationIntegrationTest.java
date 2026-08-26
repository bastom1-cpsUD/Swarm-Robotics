package org.communicationModels.cycleBuildingComms;

import java.util.ArrayList;
import java.util.List;

import org.graphs.util.OrientedPoint;
import org.graphs.voltage.HalfEdge;
import org.graphs.voltage.SquareVoltageGraph;
import org.graphs.voltage.VoltageGraph;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.robots.GeometricCycleLatticeRobot;
import org.utils.logging.CommsSnapshot;
import org.utils.logging.OutgoingMessageRecord;
import org.utils.logging.TickRecord;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Phase 4 of the migration: communication tuples replace {@code pendingChildID}, capped at
 * one concurrent face.
 *
 * <p>The cap is what makes this phase testable as a migration rather than as a feature. It
 * buys nothing on purpose -- with one obligation the robot serves one face at a time, gates
 * its inbox the same way, and retries the same edge -- so a regression here is a fault in
 * the representation, not in newly-parallel face building. The evidence for that is the
 * whole existing suite staying green, including the two defect characterizations.
 *
 * <p>These tests cover what the suite cannot see from outside: that the tuple records the
 * right parent and edge, that a rejection <em>releases</em> the child slot while a status
 * <em>removes</em> the tuple, and that vacating a site clears it.
 */
class FaceObligationIntegrationTest {

    private static final VoltageGraph GRAPH = SquareVoltageGraph.build();

    private static HalfEdge firstOutgoingEdge() {
        return GRAPH.getOutgoingHalfEdges(GRAPH.getPrimaryRole()).get(0);
    }

    /** The obligations a robot held at the end of its last activation. */
    private static List<FaceObligation> obligationsOf(List<TickRecord> records, int robotId) {
        TickRecord last = LatticeHarness.lastRecordOf(records, robotId);
        return last == null ? List.of() : last.after().obligations();
    }

    /** Every snapshot of one robot, oldest first, so a test can watch a slot change. */
    private static List<CommsSnapshot> historyOf(List<TickRecord> records, int robotId) {
        List<CommsSnapshot> history = new ArrayList<>();
        for (TickRecord record : records) {
            if (record.robotId() == robotId) {
                history.add(record.after());
            }
        }
        return history;
    }

    @Test
    @DisplayName("a root's first assignment creates exactly one obligation on the edge it is building")
    void assignmentCreatesExactlyOneObligation() {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(GRAPH, firstOutgoingEdge(), new OrientedPoint(0, 0, 0));
        GeometricCycleLatticeRobot root = robots.get(0);
        root.promoteToPrimaryRoot();

        List<TickRecord> records = LatticeHarness.tick(robots, 2);

        List<FaceObligation> held = obligationsOf(records, root.getRobotId());
        assertEquals(1, held.size(), "one face at a time at a cap of one");

        FaceObligation obligation = held.get(0);
        assertEquals(root.getRobotId(), obligation.getParentId(),
                "a root initiates its own face, so it is its own tuple's parent");
        assertNotNull(obligation.getChildId(), "the offer went out, so the slot is filled");
        assertNotNull(obligation.getChildEdge(),
                "the drawn edge belongs to the obligation, so it can be undrawn per-face");
    }

    /**
     * The cap, asserted rather than assumed. If this ever fails the phase has stopped being
     * a migration and started being the concurrency change it is meant to precede.
     */
    @Test
    @DisplayName("no robot ever holds more than one obligation while the cap is one")
    void capOfOneHoldsThroughoutARun() {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(GRAPH, firstOutgoingEdge(), new OrientedPoint(0, 0, 0));
        robots.get(0).promoteToPrimaryRoot();

        List<TickRecord> records = LatticeHarness.tick(robots, 40);

        for (TickRecord record : records) {
            assertTrue(record.after().obligations().size() <= 1,
                    "robot " + record.robotId() + " held " + record.after().obligations().size()
                            + " obligations at tick " + record.tick());
        }
    }

    /**
     * The tuple guarantee that motivates the whole migration: a response routes back to the
     * robot recorded as that face's parent, rather than to whoever this robot last spoke to.
     */
    @Test
    @DisplayName("a fulfilled obligation names the child the offer actually went to")
    void obligationChildMatchesTheAssignmentSent() {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(GRAPH, firstOutgoingEdge(), new OrientedPoint(0, 0, 0));
        GeometricCycleLatticeRobot root = robots.get(0);
        root.promoteToPrimaryRoot();

        List<TickRecord> records = LatticeHarness.tick(robots, 2);

        List<FaceObligation> held = obligationsOf(records, root.getRobotId());
        assertEquals(1, held.size());
        int recordedChild = held.get(0).getChildId();

        boolean assignmentWentToThatRobot = false;
        for (TickRecord record : records) {
            if (record.robotId() != root.getRobotId()) {
                continue;
            }
            for (OutgoingMessageRecord sent : record.sent()) {
                if (sent.messageType().equals("Assignment") && sent.recipientId() == recordedChild) {
                    assignmentWentToThatRobot = true;
                }
            }
        }
        assertTrue(assignmentWentToThatRobot,
                "the obligation records child " + recordedChild + ", but no assignment was sent there");
    }

    /**
     * Promotion must not touch the obligation set. Obligation lifetime keys on
     * <em>position</em>, and a promoted robot has not moved -- the natural mistake is to
     * clear tuples alongside the other per-role state, which would drop responses the robot
     * still owes.
     */
    @Test
    @DisplayName("promoting a robot leaves its obligations intact")
    void promotionLeavesObligationsIntact() {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(GRAPH, firstOutgoingEdge(), new OrientedPoint(0, 0, 0));
        GeometricCycleLatticeRobot root = robots.get(0);
        root.promoteToPrimaryRoot();

        LatticeHarness.tick(robots, 2);
        List<FaceObligation> before = List.copyOf(root.executeTimeStep(1.0, 3).after().obligations());
        assertFalse(before.isEmpty(), "precondition: the root is mid-face with a live obligation");

        // Promote again. promoteToPrimaryRoot is the same call the click affordance uses,
        // and it must not be a way to silently drop outstanding work.
        root.promoteToPrimaryRoot();
        List<FaceObligation> after = root.executeTimeStep(1.0, 4).after().obligations();

        assertEquals(before.size(), after.size(),
                "a promotion left the robot where it was, so its tuples are still true");
    }

    /**
     * The vacate path. Contention, liveness give-up and finding your spot occupied all route
     * through {@code resetToUnassigned}, and all of them mean the local topology this
     * robot's tuples describe has stopped being true.
     */
    @Test
    @DisplayName("vacating the lattice site clears every obligation")
    void vacatingClearsObligations() {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(GRAPH, firstOutgoingEdge(), new OrientedPoint(0, 0, 0));
        GeometricCycleLatticeRobot root = robots.get(0);
        root.promoteToPrimaryRoot();

        // Scan tick by tick rather than probing one fixed tick: a chain forms and collapses
        // repeatedly, so whether any given robot is a builder at tick N is luck.
        GeometricCycleLatticeRobot builder = null;
        for (int t = 1; t <= 20 && builder == null; t++) {
            for (TickRecord record : LatticeHarness.tick(robots, 1)) {
                if (record.robotId() != root.getRobotId()
                        && record.after().role() == CycleRole.cycleBuilder
                        && !record.after().obligations().isEmpty()) {
                    builder = robots.get(record.robotId());
                    break;
                }
            }
        }
        assumeBuilderFound(builder);

        // Drive the real vacate path rather than reaching into comms: a squatter parked
        // exactly on this robot's assigned pose trips the assignment-occupied branch of
        // makeObservations, which rejects to the parent and then resets to unassigned.
        GeometricCycleLatticeRobot squatter =
                new GeometricCycleLatticeRobot(900, new OrientedPoint(builder.getPosition()), GRAPH);
        builder.addNeighbor(squatter);
        squatter.addNeighbor(builder);

        TickRecord afterYield = builder.executeTimeStep(1.0, 8);

        assertEquals(CycleRole.unassigned, afterYield.after().role(),
                "precondition: the occupancy check should have made this robot stand down");
        assertTrue(afterYield.after().obligations().isEmpty(),
                "a robot that gave up its site must hold no tuples describing it");
    }

    private static void assumeBuilderFound(GeometricCycleLatticeRobot builder) {
        assertNotNull(builder,
                "no cycleBuilder reached a live obligation within the run, so this scenario is "
                        + "not exercising the vacate path at all");
    }

    /**
     * A departed child takes the certificate with it. Nothing upstream holds a copy -- the
     * certificate lives in messages -- so the tuple goes and the loss is reported, rather
     * than the parent re-offering with something it does not have.
     */
    @Test
    @DisplayName("a departed child removes the obligation and reports the certificate lost")
    void departedChildRemovesObligationAndReports() {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(GRAPH, firstOutgoingEdge(), new OrientedPoint(0, 0, 0));
        GeometricCycleLatticeRobot root = robots.get(0);
        root.promoteToPrimaryRoot();

        LatticeHarness.tick(robots, 2);
        List<FaceObligation> held = root.executeTimeStep(1.0, 3).after().obligations();
        assertEquals(1, held.size());
        int childId = held.get(0).getChildId();

        // The child leaves communication range.
        GeometricCycleLatticeRobot departed = robots.stream()
                .filter(r -> r.getRobotId() == childId).findFirst().orElseThrow();
        root.removeNeighbor(departed);

        TickRecord afterDeparture = root.executeTimeStep(1.0, 4);

        // The dead tuple is gone. The set need not be empty: a root that drops one
        // obligation is free again, so it picks its next edge and opens a new tuple in the
        // same tick. What must not survive is any tuple still naming the departed child --
        // that would mean the root is waiting on a certificate nobody holds.
        for (FaceObligation obligation : afterDeparture.after().obligations()) {
            assertNotEquals(Integer.valueOf(childId), obligation.getChildId(),
                    "a tuple still names the departed child " + childId + "; its certificate "
                            + "left with it and cannot be re-offered");
        }
        assertNotEquals(childId, afterDeparture.after().pendingChildID(),
                "the root must stop waiting on a child that is gone");
    }

    /**
     * Bans live on the obligation, so they are scoped to one face and disappear with it.
     * This is what let the four hand-written {@code unableToDoAssignmentIDs.clear()} calls
     * in the reset family go away.
     */
    @Test
    @DisplayName("a robot's exclusions are scoped to the face it was excluded from")
    void bansAreScopedToTheirFace() {
        FaceObligationSet set = new FaceObligationSet();
        FaceObligation faceA = set.getOrCreate(1, 10);
        FaceObligation faceB = set.getOrCreate(1, 11);

        faceA.ban(42);

        assertTrue(faceA.isBanned(42));
        assertFalse(faceB.isBanned(42),
                "an exclusion from one face must not follow a robot onto another");

        // And it dies with the face rather than needing a hand-written clear.
        set.remove(faceA);
        assertNull(set.findByEdge(10));
        assertFalse(set.getOrCreate(1, 10).isBanned(42),
                "a fresh obligation on the same edge starts with no exclusions");
    }
}