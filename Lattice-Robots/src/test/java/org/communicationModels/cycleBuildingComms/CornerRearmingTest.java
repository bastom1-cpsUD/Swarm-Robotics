package org.communicationModels.cycleBuildingComms;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

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
 * deliberate parking place. {@link CyclebuilderComms#collapseCoInitiation} writes it when
 * this robot stands down on a face a lower-id robot is already building, and
 * {@link CyclebuilderComms#routeCertificateLostThroughTuple} writes it so a corner short of
 * candidates waits behind corners never tried. Since
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
     * Four roots on one square face, so every one of them co-initiates against the others and
     * three of the four stand down -- which is what puts corners into {@code attempted} in the
     * first place. Walks then keep arriving at all four for the rest of the run, over every
     * incoming edge, which is exactly the traffic a too-wide guard converts into a re-arm.
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

        List<TickRecord> records = LatticeHarness.tick(robots, LONG_RUN);
        List<StatusChange> changes = statusChanges(records);

        // attempted -> complete and attempted -> failed are ordinary: a status decided the
        // face, or the retry found no candidate. attempted -> unattempted is the re-arm, and
        // it is the one transition nothing is entitled to make.
        List<StatusChange> rearmed = changes.stream()
                .filter(c -> c.from() == CycleStatus.attempted && c.to() == CycleStatus.unattempted)
                .toList();
        assertTrue(rearmed.isEmpty(),
                "a corner was re-armed out of attempted: " + rearmed
                        + ". attempted is where collapseCoInitiation parks a face it conceded and "
                        + "where routeCertificateLostThroughTuple parks one short of candidates; "
                        + "determineNextCycleToComplete prefers unattempted, so re-arming it "
                        + "re-launches the walk that stood down. rearmTwinOfIncomingEdge must "
                        + "re-arm out of failed only.");

        // Non-vacuous: the scenario has to have actually parked something there, or the
        // assertion above passes by never being tested.
        assertTrue(changes.stream().anyMatch(c -> c.to() == CycleStatus.attempted),
                "no corner was ever parked as attempted in " + LONG_RUN + " ticks, so this "
                        + "scenario is not exercising the guard at all. Four roots on one face "
                        + "are meant to co-initiate and all but one stand down.");
    }

    /**
     * The mechanism still fires, so narrowing the guard did not quietly disable it.
     *
     * <p>One seed root on a bare face: it closes the corner its neighbours can serve and
     * writes off the three pointing into empty space. Those neighbours are then promoted and
     * start walks of their own, which arrive back here over edges whose twins are exactly
     * those written-off corners -- and the seed learns its verdicts have expired.
     */
    @Test
    @DisplayName("a corner written off for want of a candidate is re-armed when a walk arrives over its twin")
    void failedCornersAreRearmed() {
        List<GeometricCycleLatticeRobot> robots = LatticeHarness.placeAroundRole(
                SQUARE, SQUARE.getPrimaryRole(), new OrientedPoint(0, 0, 0));
        robots.get(0).promoteToPrimaryRoot();

        List<TickRecord> records = LatticeHarness.tick(robots, LONG_RUN);
        List<StatusChange> changes = statusChanges(records);

        assertTrue(changes.stream()
                        .anyMatch(c -> c.from() == CycleStatus.failed && c.to() == CycleStatus.unattempted),
                "no corner was ever re-armed out of failed in " + LONG_RUN + " ticks. A corner "
                        + "is only failed because nobody was standing on it; a walk arriving over "
                        + "its twin proves somebody is now, and that verdict has to be revisited "
                        + "or the formation stops at whatever the first root could reach. Saw: "
                        + changes);
    }
}
