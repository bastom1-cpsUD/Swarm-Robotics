package org.communicationModels.cycleBuildingComms;

import java.util.List;
import java.util.Set;

import org.communicationModels.cycleBuildingComms.Messages.PromotionMessage;
import org.graphs.util.OrientedPoint;
import org.graphs.voltage.HalfEdge;
import org.graphs.voltage.SquareVoltageGraph;
import org.graphs.voltage.VoltageGraph;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.robots.GeometricCycleLatticeRobot;
import org.utils.logging.TickRecord;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Promotion reaching a robot that already occupies a lattice site.
 *
 * <p>{@code promoteAdjacentVerticesToRoots} is the only mechanism that extends the frontier: a
 * root that closes every one of its faces hands a {@code PromotionMessage} to the neighbour on
 * each completed corner, and those neighbours become roots and build outward. The robots it
 * addresses are, by construction, the ones that were recruited onto those corners -- parked
 * {@code cycleBuilder}s.
 *
 * <p>A cycleBuilder used to <em>defer</em> a promotion, re-queueing it untouched. That was a
 * bet that the robot would free up shortly, and it was a good bet for as long as a chain
 * collapsed on its first status. Phase 6 removed the collapse, so the bet stopped paying and
 * the frontier stopped at one cell -- silently, because a deferral is not an error and the
 * formation simply stops growing. These tests are what stops that returning.
 */
class PromotionTest {

    private static final VoltageGraph SQUARE = SquareVoltageGraph.build();

    private static HalfEdge firstOutgoingEdge() {
        return SQUARE.getOutgoingHalfEdges(SQUARE.getPrimaryRole()).get(0);
    }

    private static int cornersClosed(List<TickRecord> records, List<GeometricCycleLatticeRobot> robots) {
        int closed = 0;
        for (GeometricCycleLatticeRobot robot : robots) {
            TickRecord last = LatticeHarness.lastRecordOf(records, robot.getRobotId());
            if (last == null) {
                continue;
            }
            for (CycleStatus status : last.after().completedCycles().values()) {
                if (status == CycleStatus.complete) {
                    closed++;
                }
            }
        }
        return closed;
    }

    /**
     * The headline: a formation that used to stop after one cell now grows.
     *
     * <p>Nine robots, one per site of every face incident to the middle one. The middle robot
     * is the only root, and it can close all four of its corners from its own neighbourhood.
     * Everything past that depends on those four neighbours accepting promotion.
     */
    @Test
    @DisplayName("a completed root's promotions reach its parked builders, and the frontier grows")
    void promotionReachesSettledBuildersAndTheFrontierGrows() {
        List<GeometricCycleLatticeRobot> robots = LatticeHarness.placeAroundRole(
                SQUARE, SQUARE.getPrimaryRole(), new OrientedPoint(0, 0, 0));
        GeometricCycleLatticeRobot seed = robots.get(0);
        seed.promoteToPrimaryRoot();

        List<TickRecord> records = LatticeHarness.tick(robots, 600);

        int seedCorners = SQUARE.getOutgoingHalfEdges(SQUARE.getPrimaryRole()).size();

        int newRoots = 0;
        for (GeometricCycleLatticeRobot robot : robots) {
            if (robot.getRobotId() == seed.getRobotId()) {
                continue;
            }
            TickRecord last = LatticeHarness.lastRecordOf(records, robot.getRobotId());
            if (last != null && last.after().role() == CycleRole.root) {
                newRoots++;
            }
        }

        assertTrue(newRoots > 0,
                "the seed finished its faces and promoted its neighbours, but not one of them "
                        + "became a root. A parked cycleBuilder that defers a promotion can never "
                        + "release it -- chains stopped collapsing in Phase 6 -- so the formation "
                        + "stops at the first cell. See CyclebuilderComms.acceptPromotion.");

        assertTrue(cornersClosed(records, robots) > seedCorners,
                "only " + cornersClosed(records, robots) + " corners closed across the whole swarm, "
                        + "which is no more than the seed could close alone (" + seedCorners + "). "
                        + "The promotions landed but the new roots built nothing.");
    }

    /**
     * The deferral is gone as a mechanism, not merely unused in the scenario above -- so a
     * regression cannot hide behind a run that happened not to need it.
     */
    @Test
    @DisplayName("no promotion is ever deferred by a settled robot")
    void settledRobotsNeverDeferAPromotion() {
        List<GeometricCycleLatticeRobot> robots = LatticeHarness.placeAroundRole(
                SQUARE, SQUARE.getPrimaryRole(), new OrientedPoint(0, 0, 0));
        robots.get(0).promoteToPrimaryRoot();

        List<TickRecord> records = LatticeHarness.tick(robots, 600);

        for (TickRecord record : records) {
            String processed = record.processedDescription();
            if (processed.contains("Promotion") && processed.contains("DEFERRED")) {
                fail("robot " + record.robotId() + " deferred a promotion at tick " + record.tick()
                        + ": " + processed + ". Every robot in this scenario is standing on its "
                        + "exact site, so nothing here is in transit and nothing should defer.");
            }
        }
    }

    /**
     * The one case that <em>must</em> still defer.
     *
     * <p>A root's {@code getAssignedGlobalPosition()} short-circuits to its own pose. Promote a
     * robot that has not reached its site yet and wherever it happens to be standing becomes
     * its lattice site: it stops seeking, and every child it later places is offset from the
     * real lattice by however far it had left to go.
     *
     * <p>Unlike the deferral this replaced, this one is guaranteed to release. Arrival clears
     * it; and a robot that never arrives gives its assignment up on the liveness ladder,
     * becomes unassigned, and takes the promotion from there.
     *
     * <p>The harness does not run the motion model -- {@code robot.move(dt)} is a separate task
     * in the simulation, and every scenario here starts exactly placed so that tests are about
     * message passing rather than about {@code TimeStepDiffDrive} covering 70 units. So
     * "off its site" is staged by displacing the robot, and "arrived" by putting it back.
     */
    @Test
    @DisplayName("a builder off its site defers promotion, then takes it once placed")
    void promotionInTransitIsDeferredThenAccepted() {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(SQUARE, firstOutgoingEdge(), new OrientedPoint(0, 0, 0));
        GeometricCycleLatticeRobot root = robots.get(0);
        GeometricCycleLatticeRobot traveller = robots.get(1);

        // Off its site: far enough that the arrival check cannot pass, near enough that it is
        // still the obvious candidate for that corner.
        OrientedPoint site = new OrientedPoint(traveller.getPosition());
        traveller.setPosition(new OrientedPoint(site.getX(), site.getY() + 22, site.getOrientation()));

        root.promoteToPrimaryRoot();

        // Run until it has accepted the assignment while standing somewhere it is not meant to be.
        TickRecord enRoute = null;
        for (int tick = 1; tick <= 20 && enRoute == null; tick++) {
            for (TickRecord record : LatticeHarness.tick(robots, 1)) {
                if (record.robotId() == traveller.getRobotId()
                        && record.after().role() == CycleRole.cycleBuilder
                        && record.poseAfter().distance(site) > 1.0) {
                    enRoute = record;
                }
            }
        }
        assertNotNull(enRoute,
                "scenario error: the displaced robot never became an off-site cycleBuilder, so "
                        + "this test is not exercising the in-transit guard at all");

        // Hand it a promotion mid-journey, the way a completed root would.
        HalfEdge corner = firstOutgoingEdge();
        traveller.enqueueMessage(new PromotionMessage(root.getRobotId(), traveller.getRobotId(),
                corner.getOrigin().getId(), corner.getId(), true));

        TickRecord afterPromotion = traveller.executeTimeStep(1.0, 99);
        assertEquals(CycleRole.cycleBuilder, afterPromotion.after().role(),
                "a robot promoted mid-journey became a root and froze off-lattice: its pose is "
                        + "now its declared site. It must wait until it has arrived.");

        // Asserted structurally rather than on the description, because the deferral moved:
        // acceptPromotion used to re-queue this itself, and Phase 9 absorbed that into the
        // in-transit gate at the top of processMessages, which never dispatches far enough to
        // reach the promotion branch. What must remain true either way is that the message is
        // still sitting in the inbox, unconsumed.
        boolean stillQueued = afterPromotion.after().queueInOrder().stream()
                .anyMatch(message -> message instanceof PromotionMessage);
        assertTrue(stillQueued,
                "the promotion was consumed while the robot was in transit rather than left "
                        + "queued. Processed: " + afterPromotion.processedDescription());

        // And it is a real deferral, not a black hole. Arrival is what releases it, so put the
        // robot on its site -- placement is this harness's stand-in for driving, which is why
        // every scenario here starts exactly placed rather than converging. Two ticks, because
        // processMessages runs before sendMessage: the first sets hasArrived, the second sees it.
        traveller.setPosition(site);
        boolean becameRoot = false;
        for (int tick = 100; tick <= 120 && !becameRoot; tick++) {
            for (TickRecord record : LatticeHarness.tick(robots, 1)) {
                if (record.robotId() == traveller.getRobotId()
                        && record.after().role() == CycleRole.root) {
                    becameRoot = true;
                }
            }
        }
        assertTrue(becameRoot,
                "the deferred promotion was never released, even once the robot was standing on "
                        + "its site. Deferral is only acceptable because arrival ends it -- if it "
                        + "does not, this is the old forever-deferral wearing a narrower hat.");
    }

    /**
     * Promotion is a change of ambition, not of position, so everything the robot was carrying
     * for other robots has to survive it. This is the property Phase 6 bought when
     * {@code resetToRoot} stopped clearing the obligation set, and it is what makes promoting a
     * mid-relay builder safe rather than merely tolerable.
     */
    @Test
    @DisplayName("a promoted builder keeps the walks it was carrying")
    void promotedBuilderKeepsCarryingItsWalks() {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(SQUARE, firstOutgoingEdge(), new OrientedPoint(0, 0, 0));
        GeometricCycleLatticeRobot root = robots.get(0);
        root.promoteToPrimaryRoot();

        // Find a builder mid-relay: it holds a tuple owed to somebody else.
        GeometricCycleLatticeRobot carrier = null;
        List<FaceObligation> carried = List.of();
        for (int tick = 1; tick <= 30 && carrier == null; tick++) {
            for (TickRecord record : LatticeHarness.tick(robots, 1)) {
                if (record.robotId() == root.getRobotId()
                        || record.after().role() != CycleRole.cycleBuilder) {
                    continue;
                }
                for (FaceObligation obligation : record.after().obligations()) {
                    if (obligation.getParentId() != FaceObligation.NO_PARENT) {
                        carrier = robots.get(record.robotId());
                        carried = record.after().obligations();
                    }
                }
            }
        }
        assertNotNull(carrier, "scenario error: no builder ever carried a walk in this run");

        HalfEdge corner = firstOutgoingEdge();
        carrier.enqueueMessage(new PromotionMessage(root.getRobotId(), carrier.getRobotId(),
                corner.getOrigin().getId(), corner.getId(), true));
        TickRecord afterPromotion = carrier.executeTimeStep(1.0, 99);

        int stillCarried = 0;
        for (FaceObligation obligation : afterPromotion.after().obligations()) {
            if (obligation.getParentId() != FaceObligation.NO_PARENT) {
                stillCarried++;
            }
        }
        assertTrue(stillCarried > 0,
                "the promotion dropped every walk this robot was carrying. It held " + carried
                        + " beforehand. resetToRoot must not clear the obligation set -- the "
                        + "parents upstream are still waiting on those, and the certificates are "
                        + "in this robot's inbox with nowhere to go.");
    }

    /**
     * A promotion must not throw away the corners a builder already closed.
     *
     * <p>This is the payoff for builders tracking corners at all, and the single place it can be
     * silently undone. {@code initializeEdgeMap()} clears the map and sets every corner back to
     * {@code unattempted}; run it on the promotion path and a builder arrives as a root having
     * forgotten every face it helped close, then re-derives each one with a certificate of its own.
     * Nothing else would look wrong -- the lattice still converges, just slower -- which is exactly
     * why it is pinned here.
     *
     * <p>{@code acceptPromotion} therefore asks two separate questions. "Am I already a root?" is
     * {@code role == CycleRole.root}. "Do I have a corner map yet?" is the map-emptiness test, and
     * it is what decides whether to build one -- yes for a robot promoted straight from
     * unassigned, no for a builder that made its map when it took its site.
     */
    @Test
    @DisplayName("a promoted builder keeps the corners it closed while building")
    void aPromotedBuilderKeepsItsClosedCorners() {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(SQUARE, firstOutgoingEdge(), new OrientedPoint(0, 0, 0));
        robots.get(0).promoteToPrimaryRoot();

        // Watch each robot for the transition into root, and compare what it had closed on the
        // activation before against what it holds on the activation after.
        List<TickRecord> records = LatticeHarness.tick(robots, 240);

        boolean sawAPromotionWithMarks = false;
        for (TickRecord record : records) {
            if (record.before().role() == CycleRole.cycleBuilder
                    && record.after().role() == CycleRole.root) {
                Set<Integer> closedBefore = completeCorners(record.before());
                Set<Integer> closedAfter = completeCorners(record.after());
                if (closedBefore.isEmpty()) {
                    continue;   // nothing to lose on this promotion; not the case under test
                }
                sawAPromotionWithMarks = true;
                assertTrue(closedAfter.containsAll(closedBefore),
                        "robot " + record.robotId() + " was promoted at tick " + record.tick()
                                + " holding closed corners " + closedBefore + " and came out with "
                                + closedAfter + ". A promotion leaves the robot exactly where it "
                                + "was standing, so every corner it had closed is still closed. "
                                + "initializeEdgeMap must not run on this path.");
            }
        }

        assertTrue(sawAPromotionWithMarks,
                "no builder was ever promoted while holding a closed corner, so this run never "
                        + "exercised the property. Builders are meant to record their corner when "
                        + "the closing status laps them, well before they are promoted.");
    }

    /** The corners this snapshot has closed. */
    private static Set<Integer> completeCorners(org.utils.logging.CommsSnapshot snapshot) {
        Set<Integer> closed = new java.util.HashSet<>();
        snapshot.completedCycles().forEach((edgeId, status) -> {
            if (status == CycleStatus.complete) {
                closed.add(edgeId);
            }
        });
        return closed;
    }
}
