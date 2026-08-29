package org.communicationModels.cycleBuildingComms;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

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
 * The tuple itself, one face at a time: that it records the right parent and edge, that a
 * rejection <em>releases</em> the child slot while a status <em>removes</em> the tuple, that
 * a promotion keeps it and vacating a site clears it. None of that needs concurrency to be
 * true, which is why it stays here rather than moving to {@link ConcurrentFaceTest}.
 *
 * <p>Written for Phase 4, where communication tuples replaced {@code pendingChildID} behind
 * a cap of one concurrent face. The cap is what made that phase testable as a migration
 * rather than as a feature -- it bought nothing on purpose, so a regression was a fault in
 * the representation and not in newly-parallel face building, and the evidence was the whole
 * suite staying green.
 *
 * <p>Two of the rules these tests were written against have since been superseded, and both
 * were the cap in disguise:
 *
 * <ul>
 *   <li><strong>The inbox gate is gone.</strong> Phase 5 replaced the pending-child gate with
 *       "an unfulfilled obligation takes the tick"; Phase 6 removed gating altogether. What
 *       was being guarded is one tuple per edge, and the obligation set enforces that
 *       directly.</li>
 *   <li><strong>The cap is a structure, not a number.</strong> {@code capOfOneHoldsThroughoutARun}
 *       became {@link #oneObligationPerEdgeHoldsThroughoutARun()} -- see there.</li>
 * </ul>
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
        assertEquals(FaceObligation.NO_PARENT, obligation.getParentId(),
                "a face this robot initiated is owed to nobody, so it has no parent. This used "
                        + "to read as the robot being its own parent, which said the same thing "
                        + "by riddle and put the tuple in the carried set alongside real debts.");
        assertNotNull(obligation.getChildId(), "the offer went out, so the slot is filled");
        assertNotNull(obligation.getChildEdge(),
                "the drawn edge belongs to the obligation, so it can be undrawn per-face");
    }

    /**
     * <strong>Was {@code capOfOneHoldsThroughoutARun}. Phase 6 lifted the cap, so the
     * assertion inverts from a number to a structure.</strong>
     *
     * <p>The flat cap of one was scaffolding: it made Phases 4 and 5 behave exactly as the
     * {@code pendingChildID} version did, so a regression in either was a fault in the tuple
     * representation rather than in newly-parallel face building. What replaces it is not a
     * bigger number but a different kind of bound -- one tuple per incident edge, enforced by
     * {@link FaceObligationSet#getOrCreate} rather than counted. That bound is what actually
     * matters: a second tuple on one edge would mean two walks being carried across the same
     * hop with one child slot between them, and the responses would route to the wrong parent.
     *
     * <p>Stated over the <em>carried</em> walks, since separating this robot's own attempt into
     * its own slot. The bound the cap used to spell as {@code outgoing + 1} decomposes into the
     * two assertions below, each of which is a fact about one container rather than a number
     * needing an explanation.
     */
    @Test
    @DisplayName("no robot ever holds two carried walks on the same edge, or two faces of its own")
    void oneObligationPerEdgeHoldsThroughoutARun() {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(GRAPH, firstOutgoingEdge(), new OrientedPoint(0, 0, 0));
        robots.get(0).promoteToPrimaryRoot();

        List<TickRecord> records = LatticeHarness.tick(robots, 40);

        int outgoingEdges = GRAPH.getOutgoingHalfEdges(GRAPH.getPrimaryRole()).size();
        for (TickRecord record : records) {
            List<FaceObligation> held = record.after().obligations();

            Set<Integer> carriedEdges = new HashSet<>();
            int attempts = 0;
            for (FaceObligation obligation : held) {
                if (obligation.getParentId() == FaceObligation.NO_PARENT) {
                    attempts++;
                    continue;
                }
                assertTrue(carriedEdges.add(obligation.getEdgeId()),
                        "robot " + record.robotId() + " held two carried walks on edge "
                                + obligation.getEdgeId() + " at tick " + record.tick()
                                + "; one child slot cannot serve two walks");
            }

            assertTrue(attempts <= 1,
                    "robot " + record.robotId() + " held " + attempts + " faces of its own at tick "
                            + record.tick() + "; FaceObligationSet holds a single slot for that");
            assertTrue(carriedEdges.size() <= outgoingEdges,
                    "robot " + record.robotId() + " carried " + carriedEdges.size()
                            + " walks at tick " + record.tick() + ", past one per incoming edge");
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