package org.communicationModels.cycleBuildingComms;

import java.util.List;

import org.communicationModels.cycleBuildingComms.Messages.VoltageCertificate;
import org.communicationModels.cycleBuildingComms.Messages.PositioningMessage;
import org.graphs.util.OrientedPoint;
import org.graphs.util.RigidBodyTransformation;
import org.graphs.voltage.HalfEdge;
import org.graphs.voltage.Role;
import org.robots.GeometricCycleLatticeRobot;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Drives real {@link GeometricCycleLatticeRobot}s through the two-phase tick, headlessly,
 * to check that assignment contention is actually detected and resolved end to end --
 * beacon emitted, claim received, frames reconciled, one robot yielding.
 *
 * <p>This covers the wiring that the pure-geometry tests in {@code TargetClaimTest}
 * cannot: that the claim is emitted at all, that it reaches neighbours, that phase two
 * runs, and that a yield actually clears the assignment rather than merely logging.
 */
class AssignmentContentionTest {

    /** Neighbourhood wiring, standing in for the panel's proximity check. */
    private static void makeAllNeighbors(GeometricCycleLatticeRobot... robots) {
        for (GeometricCycleLatticeRobot a : robots) {
            for (GeometricCycleLatticeRobot b : robots) {
                if (a.getRobotId() != b.getRobotId()) {
                    a.addNeighbor(b);
                }
            }
        }
    }

    /** An arbitrary but valid (role, outgoing edge) pair from the configured lattice. */
    private static HalfEdge anyOutgoingEdge() {
        for (Role role : GeometricCycleLatticeRobot.GRAPH.getRoles()) {
            List<HalfEdge> outgoing = GeometricCycleLatticeRobot.GRAPH.getOutgoingHalfEdges(role);
            if (!outgoing.isEmpty()) {
                return outgoing.get(0);
            }
        }
        throw new IllegalStateException("lattice has no outgoing half-edges");
    }

    /** Where a parent at {@code parentPose} is pointing a child it hands {@code edge}. */
    private static OrientedPoint targetOf(OrientedPoint parentPose, HalfEdge edge) {
        return new RigidBodyTransformation(parentPose).compose(edge.getVoltage()).asPose();
    }

    /**
     * A unit vector perpendicular to the parent-to-target step, as {x, y}.
     *
     * <p>Rivals are offset along this rather than along the step itself so that both stay
     * inside {@code COMM_RANGE} of both parents: the offset adds in quadrature to the edge
     * length instead of adding to it, and the shipped lattices run up to 70 units against a
     * range of 75.
     */
    private static double[] perpendicularToStep(HalfEdge edge) {
        OrientedPoint step = edge.getVoltage().asPose();
        double length = Math.hypot(step.getX(), step.getY());
        return new double[] { -step.getY() / length, step.getX() / length };
    }

    private static OrientedPoint offsetFrom(OrientedPoint base, double[] direction, double distance) {
        return new OrientedPoint(base.getX() + direction[0] * distance,
                                 base.getY() + direction[1] * distance,
                                 base.getOrientation());
    }

    private static void assign(GeometricCycleLatticeRobot parent,
                               GeometricCycleLatticeRobot child,
                               HalfEdge edge) {
        int vertexId = edge.getOrigin().getId();
        child.enqueueMessage(new PositioningMessage(
                parent.getRobotId(), child.getRobotId(),
                vertexId, edge.getId(), vertexId, edge.getId(),
                new VoltageCertificate(parent.getRobotId())));
    }

    /**
     * Two parents at the same pose handing out the same edge produce, by construction, the
     * same target -- precisely the failure this mechanism exists for: two independent
     * roots pointing two robots at one spot.
     *
     * @param highActivatesFirst which of the rivals takes its turn first each tick
     */
    private static void runContendedScenario(boolean highActivatesFirst, int ticks,
                                             GeometricCycleLatticeRobot low,
                                             GeometricCycleLatticeRobot high) {
        runContendedScenario(highActivatesFirst, ticks, low, high,
                new OrientedPoint(0, 0, 0), new OrientedPoint(0, 0, 0));
    }

    /**
     * As above, but with the two parents placed independently. Parents at the same pose point
     * both children at one point, which isolates the tie-break; parents a little apart point
     * them at two points close enough to be one lattice spot, which is what tests symmetry.
     */
    private static void runContendedScenario(boolean highActivatesFirst, int ticks,
                                             GeometricCycleLatticeRobot low,
                                             GeometricCycleLatticeRobot high,
                                             OrientedPoint parentOfLowPose,
                                             OrientedPoint parentOfHighPose) {
        HalfEdge edge = anyOutgoingEdge();
        GeometricCycleLatticeRobot parentOfLow = new GeometricCycleLatticeRobot(100, parentOfLowPose);
        GeometricCycleLatticeRobot parentOfHigh = new GeometricCycleLatticeRobot(101, parentOfHighPose);

        makeAllNeighbors(parentOfLow, parentOfHigh, low, high);
        assign(parentOfLow, low, edge);
        assign(parentOfHigh, high, edge);

        for (int tick = 1; tick <= ticks; tick++) {
            if (highActivatesFirst) {
                high.executeTimeStep(1.0, tick);
                low.executeTimeStep(1.0, tick);
            } else {
                low.executeTimeStep(1.0, tick);
                high.executeTimeStep(1.0, tick);
            }
        }
    }

    /**
     * The end-to-end wiring, and rule 3 -- id -- where it belongs: after possession and
     * distance have both tied.
     *
     * <p>The rivals are placed symmetrically about the shared target, so they are exactly
     * equidistant and id is the only thing left that can separate them. They used to be at
     * two arbitrary poses, which put the lower id considerably <em>further</em> from the
     * target; distance decided, id never ran, and because the verdict was inverted at the
     * time the wrong winner looked like the right one. Symmetric placement is what makes
     * this test about the thing its name claims.
     */
    @Test
    @DisplayName("Two robots equidistant from the same spot: the higher id yields, whoever activates first")
    void higherIdYieldsTheContestedSpot() {
        HalfEdge edge = anyOutgoingEdge();
        OrientedPoint parentPose = new OrientedPoint(0, 0, 0);
        OrientedPoint target = targetOf(parentPose, edge);
        double[] sideways = perpendicularToStep(edge);

        // Four ticks is two full time steps. One is not enough in the adverse activation
        // order: on the first tick neither robot has processed its assignment yet when it
        // emits, so nothing is claimed; on the second, whichever evaluates first has an
        // empty inbox because its rival has still not emitted. No TTL can cover that --
        // there is no claim yet to keep alive -- and closing it entirely would need a
        // global barrier between emission and evaluation, which is exactly what the
        // asynchronous scheduler rules out. Convergence within two time steps is the real
        // guarantee, and it holds in both orders.
        for (boolean highFirst : new boolean[] { false, true }) {
            GeometricCycleLatticeRobot low = new GeometricCycleLatticeRobot(1, offsetFrom(target, sideways, -20));
            GeometricCycleLatticeRobot high = new GeometricCycleLatticeRobot(2, offsetFrom(target, sideways, 20));

            assertEquals(low.getPosition().distance(target), high.getPosition().distance(target), 1e-9,
                    "scenario error: the rivals must be equidistant or distance, not id, decides this");

            runContendedScenario(highFirst, 4, low, high);

            String order = highFirst ? " (higher id activating first)" : " (lower id activating first)";
            assertEquals(CycleRole.cycleBuilder, low.getRole(),
                    "lower id must keep the contested assignment" + order);
            assertEquals(CycleRole.unassigned, high.getRole(),
                    "higher id must yield the contested assignment" + order);
        }
    }

    @Test
    @DisplayName("Neither robot yields before both have had a chance to declare")
    void noOneYieldsOnEvidenceThatDoesNotExistYet() {
        GeometricCycleLatticeRobot low = new GeometricCycleLatticeRobot(1, new OrientedPoint(-20, 12, 0.3));
        GeometricCycleLatticeRobot high = new GeometricCycleLatticeRobot(2, new OrientedPoint(18, -9, -1.2));

        // One time step: assignments have just landed and the first beacons are only now
        // going out. Whatever happens here, it must never be *both* robots yielding -- that
        // would leave the spot unclaimed and waste two reassignments.
        runContendedScenario(true, 2, low, high);

        assertFalse(low.getRole() == CycleRole.unassigned && high.getRole() == CycleRole.unassigned,
                "both robots must never yield the same spot simultaneously");
    }

    /**
     * Rule 2, distance, beating rule 3, id.
     *
     * <p>Both parents stand at the same pose, so both children are pointed at exactly one
     * point and there is nothing to arbitrate but who has less ground left to cover. The
     * higher id is put nearer, so an id-only tie-break would give the wrong answer and this
     * cannot pass by accident.
     */
    @Test
    @DisplayName("The nearer robot keeps the spot even holding the higher id")
    void nearerRobotKeepsTheSpotDespiteTheHigherId() {
        HalfEdge edge = anyOutgoingEdge();
        OrientedPoint parentPose = new OrientedPoint(0, 0, 0);
        OrientedPoint target = targetOf(parentPose, edge);
        double[] sideways = perpendicularToStep(edge);

        for (boolean highFirst : new boolean[] { false, true }) {
            // Offsets bounded by COMM_RANGE: a sideways offset s from a target one edge away
            // leaves the robot sqrt(edge^2 + s^2) from the parent, and the longest shipped
            // edge is 70 against a range of 75. They must also stay KEEP_OUT (30) apart.
            GeometricCycleLatticeRobot far = new GeometricCycleLatticeRobot(1, offsetFrom(target, sideways, -24));
            GeometricCycleLatticeRobot near = new GeometricCycleLatticeRobot(2, offsetFrom(target, sideways, 9));

            runContendedScenario(highFirst, 4, far, near, parentPose, parentPose);

            String order = highFirst ? " (nearer activating first)" : " (further activating first)";
            assertEquals(CycleRole.cycleBuilder, near.getRole(),
                    "the nearer robot must keep the contested spot, id notwithstanding" + order);
            assertEquals(CycleRole.unassigned, far.getRole(),
                    "the further robot must yield even though it holds the lower id" + order);
        }
    }

    /**
     * The predicate has to be symmetric, or "I am closer, so the other one will find itself
     * further and yield" does not follow.
     *
     * <p>Two parents a fraction of gamma apart point their children at two <em>different</em>
     * points -- close enough to be the same lattice spot as far as the bounding-circle test
     * is concerned, which is exactly when contention fires. The children straddle those two
     * points and are each an equal distance {@code d} from their own target and {@code d +
     * delta} from the other's. So a robot that ranks both rivals against <em>its own</em>
     * claim point concludes it is the nearer one -- and so does the other, and neither
     * yields, and both drive onto one spot.
     *
     * <p>Ranking on the distance each robot broadcast, each to its own target, gives both of
     * them the same pair of scalars: a tie, broken by id, one winner. That is the property
     * under test, and the scenario is built so that it is the only thing that can produce
     * one.
     */
    @Test
    @DisplayName("Rivals that each measure themselves nearest do not both keep the spot")
    void contentionIsSymmetricWhenTheTwoTargetsDiffer() {
        HalfEdge edge = anyOutgoingEdge();
        double delta = 0.5 * GeometricCycleLatticeRobot.tickTravel();
        double[] sideways = perpendicularToStep(edge);

        OrientedPoint parentOfLowPose = new OrientedPoint(0, 0, 0);
        OrientedPoint parentOfHighPose = offsetFrom(parentOfLowPose, sideways, delta);
        OrientedPoint targetOfLow = targetOf(parentOfLowPose, edge);
        OrientedPoint targetOfHigh = targetOf(parentOfHighPose, edge);

        // Each robot sits distance d beyond its own target, on the far side from the other's,
        // so each is nearer its own by exactly delta.
        double d = 15;
        for (boolean highFirst : new boolean[] { false, true }) {
            GeometricCycleLatticeRobot low = new GeometricCycleLatticeRobot(1, offsetFrom(targetOfLow, sideways, -d));
            GeometricCycleLatticeRobot high = new GeometricCycleLatticeRobot(2, offsetFrom(targetOfHigh, sideways, d));

            // Guard the construction rather than trusting it: if the geometry ever stops
            // being the both-think-they-are-nearest case, this test stops proving anything.
            assertTrue(low.getPosition().distance(targetOfLow) < low.getPosition().distance(targetOfHigh),
                    "scenario error: the low-id robot is not nearer its own target");
            assertTrue(high.getPosition().distance(targetOfHigh) < high.getPosition().distance(targetOfLow),
                    "scenario error: the high-id robot is not nearer its own target");

            runContendedScenario(highFirst, 4, low, high, parentOfLowPose, parentOfHighPose);

            String order = highFirst ? " (higher id activating first)" : " (lower id activating first)";
            assertFalse(low.getRole() == CycleRole.cycleBuilder && high.getRole() == CycleRole.cycleBuilder,
                    "both robots kept a spot they are both converging on" + order + ". Each ranked "
                            + "the pair against its OWN claim point and concluded it was nearer. "
                            + "Rival distance must be read off the claim -- each robot's distance "
                            + "to its own target -- so both compare the same two scalars. See "
                            + "CyclebuilderComms.detectAssignmentContention.");
            assertFalse(low.getRole() == CycleRole.unassigned && high.getRole() == CycleRole.unassigned,
                    "both robots yielded the same spot" + order + ", leaving it unclaimed");
        }
    }

    @Test
    @DisplayName("A robot with no rival keeps its assignment across a full time step")
    void uncontestedAssignmentSurvives() {
        HalfEdge edge = anyOutgoingEdge();

        GeometricCycleLatticeRobot parent = new GeometricCycleLatticeRobot(100, new OrientedPoint(0, 0, 0));
        GeometricCycleLatticeRobot child = new GeometricCycleLatticeRobot(2, new OrientedPoint(18, -9, -1.2));
        GeometricCycleLatticeRobot bystander = new GeometricCycleLatticeRobot(1, new OrientedPoint(-25, 20, 0.9));

        makeAllNeighbors(parent, child, bystander);
        assign(parent, child, edge);

        // The bystander has a lower id but declares nothing, so it must not cause a yield
        // -- only a matching claim counts, never mere proximity.
        for (int tick = 1; tick <= 4; tick++) {
            child.executeTimeStep(1.0, tick);
            bystander.executeTimeStep(1.0, tick);
        }

        assertEquals(CycleRole.cycleBuilder, child.getRole(),
                "an uncontested assignment must not be given up");
    }
}
