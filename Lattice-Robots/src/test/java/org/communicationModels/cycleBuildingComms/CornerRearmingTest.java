package org.communicationModels.cycleBuildingComms;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.communicationModels.cycleBuildingComms.Messages.CertificateLostMessage;
import org.communicationModels.cycleBuildingComms.Messages.PositioningMessage;
import org.communicationModels.cycleBuildingComms.Messages.VoltageCertificate;
import org.graphs.util.RigidBodyTransformation;
import org.graphs.voltage.HalfEdge;
import org.graphs.util.OrientedPoint;
import org.graphs.voltage.SquareVoltageGraph;
import org.graphs.voltage.VoltageGraph;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.robots.GeometricCycleLatticeRobot;
import org.utils.logging.TickRecord;

import static org.junit.jupiter.api.Assertions.*;

/**
 * {@code rearmTwinOfIncomingEdge}: which corner it re-arms, and which states it is allowed
 * to re-arm out of.
 *
 * <p>A walk arriving over an edge is evidence that somebody occupies the site on the far
 * side of it -- source and target swap across the twin -- so a corner written off because
 * {@code findBestNeighborForEdge} found nobody there is worth trying again. That much is the
 * whole point of the mechanism.
 *
 * <p>What is easy to get wrong is the <em>scope</em>. The obvious guard is "anything that is
 * not complete", and it is too wide: {@code attempted} is not a stale verdict, it is a
 * deliberate parking place. {@link CyclebuilderComms#routeCertificateLostThroughTuple} writes
 * it so a corner whose walk lost its certificate waits behind corners never tried. Since
 * {@link CyclebuilderComms#determineNextCycleToComplete()} prefers {@code unattempted}, a
 * re-arm out of {@code attempted} does not merely lose a hint -- it sends the robot back to
 * the front of the queue on a face it had just conceded, against the walk that won.
 *
 * <p>Both properties are read off {@code completedCycles} in the tick snapshots, so neither
 * needs a handle on the comms object.
 */
class CornerRearmingTest {

    private static final VoltageGraph SQUARE = SquareVoltageGraph.build();
    private static final int LONG_RUN = 400;

    /** Every (edge, from, to) corner-status change a robot made within a single activation. */
    private record StatusChange(int tick, int robotId, int edgeId, CycleStatus from, CycleStatus to) {
        @Override
        public String toString() {
            return "tick " + tick + ": robot " + robotId + " edge " + edgeId + " " + from + " -> " + to;
        }
    }

    private static List<StatusChange> statusChanges(List<TickRecord> records) {
        List<StatusChange> changes = new ArrayList<>();
        for (TickRecord record : records) {
            Map<Integer, CycleStatus> before = record.before().completedCycles();
            for (Map.Entry<Integer, CycleStatus> entry : record.after().completedCycles().entrySet()) {
                CycleStatus was = before.get(entry.getKey());
                if (was != null && was != entry.getValue()) {
                    changes.add(new StatusChange(record.tick(), record.robotId(),
                            entry.getKey(), was, entry.getValue()));
                }
            }
        }
        return changes;
    }

    /**
     * Four roots on one square face, all building and relaying at once, with a lost-certificate
     * report injected at the robot that minted the walk.
     *
     * <p>The scenario used to rely on co-initiation: four roots on one face each detected the
     * duplication and three stood down, parking their conceded corners as {@code attempted}. That
     * arbitration is gone -- duplicate walks are now carried rather than collapsed, because every
     * refusal it produced landed on a robot standing exactly on the site it was refusing -- so the
     * only remaining writer of {@code attempted} is
     * {@link CyclebuilderComms#routeCertificateLostThroughTuple}, and it is driven directly here.
     *
     * <p>The property under test is unchanged and is the reason the test exists: whatever puts a
     * corner into {@code attempted}, nothing may take it back out to {@code unattempted}.
     */
    @Test
    @DisplayName("a corner parked as attempted is never re-armed out from under the walk that won")
    void attemptedCornersAreNotRearmed() {
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(SQUARE, SQUARE.getOutgoingHalfEdges(SQUARE.getPrimaryRole()).get(0),
                        new OrientedPoint(0, 0, 0));
        for (GeometricCycleLatticeRobot robot : robots) {
            robot.promoteToPrimaryRoot();
        }

        List<TickRecord> records = new ArrayList<>(LatticeHarness.tick(robots, 6));

        // Park a corner as attempted the one way that is still reachable: tell the minter that the
        // walk it has in flight lost its certificate below its own child.
        GeometricCycleLatticeRobot seed = robots.get(0);
        FaceObligation attempt = null;
        for (FaceObligation o : LatticeHarness.lastRecordOf(records, seed.getRobotId())
                .after().obligations()) {
            if (o.getParentId() == FaceObligation.NO_PARENT && o.getChildId() != null) {
                attempt = o;
            }
        }
        assertNotNull(attempt, "scenario error: the seed never got a walk of its own in flight");
        seed.enqueueMessage(new CertificateLostMessage(attempt.getChildId(), seed.getRobotId(),
                SQUARE.getPrimaryRole().getId(), attempt.getEdgeId(), seed.getRobotId()));

        records.addAll(LatticeHarness.tick(robots, LONG_RUN));
        List<StatusChange> changes = statusChanges(records);

        // attempted -> complete and attempted -> failed are ordinary: a status decided the
        // face, or the retry found no candidate. attempted -> unattempted is the re-arm, and
        // it is the one transition nothing is entitled to make.
        List<StatusChange> rearmed = changes.stream()
                .filter(c -> c.from() == CycleStatus.attempted && c.to() == CycleStatus.unattempted)
                .toList();
        assertTrue(rearmed.isEmpty(),
                "a corner was re-armed out of attempted: " + rearmed
                        + ". attempted is where routeCertificateLostThroughTuple parks a corner "
                        + "whose walk lost its certificate; determineNextCycleToComplete prefers "
                        + "unattempted, so re-arming it jumps the queue on a corner that was "
                        + "deliberately sent to the back. rearmTwinOfIncomingEdge must re-arm out "
                        + "of failed only.");

        // Non-vacuous: the scenario has to have actually parked something there, or the
        // assertion above passes by never being tested.
        assertTrue(changes.stream().anyMatch(c -> c.to() == CycleStatus.attempted),
                "no corner was ever parked as attempted, so this scenario is not exercising the "
                        + "guard at all. The injected CertificateLostMessage should have parked "
                        + "the seed's corner. Saw: " + changes);
    }

    /**
     * The mechanism still fires, so narrowing the guard did not quietly disable it.
     *
     * <p>Four robots on one bare square face, every one a root. Each closes the corner the ring
     * can serve and writes off the three pointing into empty space -- and then keeps relaying its
     * neighbours' walks, which arrive over edges whose twins are exactly those written-off
     * corners. A robot standing on the far side of one is proof the verdict has expired.
     *
     * <p>{@code placeAroundRole} is deliberately not used here any more: it fills every site
     * around the seed, so the seed's corners all close and nothing is ever written off for the
     * re-arm to undo. The test passed on it only because the old code produced spurious failures.
     */
    @Test
    @DisplayName("a corner written off for want of a candidate is re-armed when a walk arrives over its twin")
    void failedCornersAreRearmed() {
        // A seed with all four of its lattice neighbours present but NO diagonals, so every square
        // it tries has three of its four sites and can close none of them. That is the one shape
        // that produces the state under test: corners written off while the robots standing on
        // them are still there and still observable.
        //
        // placeAroundRole cannot do it -- it fills every site around the seed, so every corner
        // closes and nothing is ever written off for a re-arm to undo. placeOnFace cannot either:
        // a bare ring closes its one face before any corner fails, and that closure is the last
        // traffic the seed sees.
        //
        // Every robot is a root, and that is what keeps the scenario about bookkeeping rather than
        // motion. An UNASSIGNED robot offered a site it is not on accepts and drives there, and
        // LatticeHarness deliberately never calls move() -- so it would sit in transit forever,
        // answering nothing, and the walk would stall instead of failing. A root refuses a site it
        // is not standing on, which is the answer this test needs and the one a settled formation
        // actually gives.
        GeometricCycleLatticeRobot seed = new GeometricCycleLatticeRobot(0, new OrientedPoint(0, 0, 0), SQUARE);
        List<GeometricCycleLatticeRobot> robots = new ArrayList<>();
        robots.add(seed);
        Map<Integer, GeometricCycleLatticeRobot> occupantOfCorner = new java.util.HashMap<>();
        int nextId = 1;
        for (HalfEdge corner : SQUARE.getOutgoingHalfEdges(SQUARE.getPrimaryRole())) {
            OrientedPoint site = new RigidBodyTransformation(seed.getPosition())
                    .compose(corner.getVoltage()).asPose();
            GeometricCycleLatticeRobot occupant = new GeometricCycleLatticeRobot(nextId++, site, SQUARE);
            occupantOfCorner.put(corner.getId(), occupant);
            robots.add(occupant);
        }
        LatticeHarness.makeAllNeighbors(robots);
        for (GeometricCycleLatticeRobot robot : robots) {
            robot.promoteToPrimaryRoot();
        }

        List<TickRecord> records = new ArrayList<>();
        int failedCorner = -1;
        for (int t = 1; t <= LONG_RUN && failedCorner == -1; t++) {
            for (GeometricCycleLatticeRobot robot : robots) {
                records.add(robot.executeTimeStep(1.0, t));
            }
            for (Map.Entry<Integer, CycleStatus> e :
                    LatticeHarness.lastRecordOf(records, 0).after().completedCycles().entrySet()) {
                if (e.getValue() == CycleStatus.failed) {
                    failedCorner = e.getKey();
                }
            }
        }
        assertTrue(failedCorner != -1,
                "scenario error: the seed never wrote off a corner, so there is nothing to re-arm");

        // The evidence that the verdict has expired: a walk arriving from the robot standing on
        // that corner's far end, over the twin of the corner. Injected rather than waited for,
        // because the occupant is a cycleBuilder and will not start a walk of its own.
        HalfEdge corner = SQUARE.getHalfEdgeById(failedCorner);
        HalfEdge incoming = corner.getTwin();
        GeometricCycleLatticeRobot occupant = occupantOfCorner.get(failedCorner);
        seed.enqueueMessage(new PositioningMessage(occupant.getRobotId(), seed.getRobotId(),
                incoming.getOrigin().getId(), incoming.getId(),
                incoming.getOrigin().getId(), incoming.getId(),
                new VoltageCertificate(occupant.getRobotId())));

        for (int t = 1; t <= 20; t++) {
            for (GeometricCycleLatticeRobot robot : robots) {
                records.add(robot.executeTimeStep(1.0, t));
            }
        }

        List<StatusChange> changes = statusChanges(records);
        assertTrue(changes.stream()
                        .anyMatch(c -> c.edgeId() == corner.getId()
                                && c.from() == CycleStatus.failed && c.to() == CycleStatus.unattempted),
                "corner " + corner.getId() + " was written off and then a walk arrived over its "
                        + "twin, and it was not re-armed. A corner is only failed because nobody "
                        + "was standing on it; a walk arriving over its twin proves somebody is "
                        + "now, and that verdict has to be revisited or the formation stops at "
                        + "whatever the first root could reach. Saw: " + changes);
    }
}
