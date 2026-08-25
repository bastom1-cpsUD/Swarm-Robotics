package org.communicationModels.cycleBuildingComms;

import java.util.List;

import org.communicationModels.cycleBuildingComms.Messages.VoltageCertificate;
import org.communicationModels.cycleBuildingComms.Messages.PositioningMessage;
import org.graphs.util.OrientedPoint;
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
        HalfEdge edge = anyOutgoingEdge();
        GeometricCycleLatticeRobot parentOfLow = new GeometricCycleLatticeRobot(100, new OrientedPoint(0, 0, 0));
        GeometricCycleLatticeRobot parentOfHigh = new GeometricCycleLatticeRobot(101, new OrientedPoint(0, 0, 0));

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

    @Test
    @DisplayName("Two robots assigned the same spot: the higher id yields, whoever activates first")
    void higherIdYieldsTheContestedSpot() {
        // Four ticks is two full time steps. One is not enough in the adverse activation
        // order: on the first tick neither robot has processed its assignment yet when it
        // emits, so nothing is claimed; on the second, whichever evaluates first has an
        // empty inbox because its rival has still not emitted. No TTL can cover that --
        // there is no claim yet to keep alive -- and closing it entirely would need a
        // global barrier between emission and evaluation, which is exactly what the
        // asynchronous scheduler rules out. Convergence within two time steps is the real
        // guarantee, and it holds in both orders.
        for (boolean highFirst : new boolean[] { false, true }) {
            GeometricCycleLatticeRobot low = new GeometricCycleLatticeRobot(1, new OrientedPoint(-20, 12, 0.3));
            GeometricCycleLatticeRobot high = new GeometricCycleLatticeRobot(2, new OrientedPoint(18, -9, -1.2));

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
