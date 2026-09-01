package org.communicationModels.cycleBuildingComms;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.graphs.util.OrientedPoint;
import org.graphs.voltage.HalfEdge;
import org.graphs.voltage.OctagonSquareVoltageGraph;
import org.graphs.voltage.Role;
import org.graphs.voltage.SquareVoltageGraph;
import org.graphs.voltage.VoltageGraph;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.robots.GeometricCycleLatticeRobot;

import static org.junit.jupiter.api.Assertions.*;

/**
 * {@code completedCycles} as an invariant rather than an accident: it holds the corners of the
 * site this robot is standing on, and nothing else, at every moment.
 *
 * <p>Every writer in {@code CyclebuilderComms} keys off either the open attempt -- whose edge id
 * came out of {@code determineNextCycleToComplete()}, so it is a key by construction -- or an
 * explicit {@code containsKey} guard. None of them can currently get it wrong. But that safety
 * rested on a fact stated nowhere: a populated map can never reach a robot standing somewhere else,
 * because a root is never dropped back to unassigned (all three vacate paths are gated on
 * {@code role == cycleBuilder}). True today, invisible, and exactly the kind of global argument a
 * later change breaks without anything noticing.
 *
 * <p>So the invariant is made local, and these tests pin the three pieces that make it so: the
 * setter refuses a key it does not already hold, vacating forgets the map, and initialising it
 * clears before it fills.
 *
 * <p>A phantom corner is not inert, which is why this matters at all. {@code hasFailed()} counts it
 * and {@code determineNextCycleToComplete()} hands it out, so a root either chases a corner that
 * does not exist or stands down on one -- and {@code acceptPromotion} reads a non-empty map as
 * "I am already a root", so a robot carrying a stale one is never given an edge map for its new
 * site at all.
 */
class CycleStatusBookkeepingTest {

    private static final VoltageGraph SQUARE = SquareVoltageGraph.build();
    private static final VoltageGraph OCTAGON_SQUARE = OctagonSquareVoltageGraph.build();

    /**
     * A comms system on its own, with no neighbours and no run behind it.
     *
     * <p>Built directly rather than driven through {@code GeometricCycleLatticeRobot} because every
     * property here is about the corner map itself, and a scenario that had to reach these states
     * emergently would be testing the scenario.
     */
    private static CyclebuilderComms standalone(VoltageGraph graph) {
        return new CyclebuilderComms(
                new GeometricCycleLatticeRobot(0, new OrientedPoint(0, 0, 0), graph), graph);
    }

    private static Set<Integer> cornersOf(VoltageGraph graph, Role role) {
        Set<Integer> ids = new HashSet<>();
        for (HalfEdge edge : graph.getOutgoingHalfEdges(role)) {
            ids.add(edge.getId());
        }
        return ids;
    }

    /** The tracked corners, read back through the public accessor one id at a time. */
    private static Map<Integer, CycleStatus> trackedAmong(CyclebuilderComms comms, Set<Integer> candidates) {
        Map<Integer, CycleStatus> tracked = new java.util.LinkedHashMap<>();
        for (int id : candidates) {
            CycleStatus status = comms.getCycleStatusOf(id);
            if (status != null) {
                tracked.put(id, status);
            }
        }
        return tracked;
    }

    /** Every half-edge id in the lattice, so a test can ask what is tracked without guessing. */
    private static Set<Integer> allEdgeIds(VoltageGraph graph) {
        Set<Integer> ids = new HashSet<>();
        for (Role role : graph.getRoles()) {
            ids.addAll(cornersOf(graph, role));
        }
        return ids;
    }

    @Test
    @DisplayName("marking an edge this robot does not track is refused, not silently created")
    void unknownEdgeIsRefused() {
        CyclebuilderComms comms = standalone(SQUARE);
        comms.promoteToPrimaryRoot();

        Set<Integer> mine = cornersOf(SQUARE, SQUARE.getPrimaryRole());
        Map<Integer, CycleStatus> before = trackedAmong(comms, allEdgeIds(SQUARE));
        assertEquals(mine, before.keySet(), "scenario error: a fresh root should track its own corners");

        // An id no role in this lattice owns. Marking it complete used to add it to the map, where
        // hasFailed() would count it and determineNextCycleToComplete() would hand it out.
        int phantom = 9999;
        assertNull(comms.setCycleStatusOf(phantom, CycleStatus.complete),
                "the setter reported a previous status for an edge it never tracked");
        assertNull(comms.getCycleStatusOf(phantom),
                "marking an untracked edge created a corner for it. A phantom corner is not inert: "
                        + "hasFailed() counts it and determineNextCycleToComplete() hands it out, so "
                        + "the root chases a corner with no site behind it.");
        assertEquals(before, trackedAmong(comms, allEdgeIds(SQUARE)),
                "a refused write changed the map anyway");
    }

    @Test
    @DisplayName("a refused write cannot push a root into its failure verdict")
    void refusedWriteDoesNotAffectHasFailed() {
        CyclebuilderComms comms = standalone(SQUARE);
        comms.promoteToPrimaryRoot();

        // Settle every real corner as complete. hasFailed() is "nothing left to attempt, AND at
        // least one corner failed", so this is the state where a single phantom `failed` entry
        // flips the verdict -- on a fresh root the four unattempted corners mask it, and the test
        // would pass whether or not the guard existed.
        for (int id : cornersOf(SQUARE, SQUARE.getPrimaryRole())) {
            comms.setCycleStatusOf(id, CycleStatus.complete);
        }
        assertFalse(comms.hasFailed(), "scenario error: every corner complete is not a failure");

        comms.setCycleStatusOf(9999, CycleStatus.failed);

        assertFalse(comms.hasFailed(),
                "a write to an edge this robot does not own moved its failure verdict. That verdict "
                        + "is what stops a root building and sends it to promote-and-stand-down.");
    }

    @Test
    @DisplayName("a robot dropped out of the formation forgets the corners of the site it left")
    void vacatingForgetsTheCornerMap() {
        CyclebuilderComms comms = standalone(SQUARE);
        comms.promoteToPrimaryRoot();
        assertFalse(trackedAmong(comms, allEdgeIds(SQUARE)).isEmpty(),
                "scenario error: a root should be tracking corners before it vacates");

        comms.resetToUnassigned();

        assertTrue(trackedAmong(comms, allEdgeIds(SQUARE)).isEmpty(),
                "a robot dropped back to unassigned kept the corners of the site it walked away "
                        + "from. acceptPromotion reads a non-empty map as \"I am already a root\", "
                        + "so this robot would never be given an edge map for wherever it ends up "
                        + "next, and would go on marking corners belonging to its old role.");
    }

    /**
     * The multi-role case, which is the only one where this is visible at all.
     *
     * <p>{@code initializeEdgeMap} adds one entry per outgoing edge of the current role. On a
     * single-role lattice re-running it writes the same key set, so failing to clear first is
     * invisible; on OctagonSquare, four roles have four different sets, and the leftovers are
     * corners of somebody else's site.
     */
    @Test
    @DisplayName("re-initialising for a new role leaves no corners of the old one")
    void initialisingClearsBeforeItFills() {
        Role first = null;
        Role second = null;
        for (Role role : OCTAGON_SQUARE.getRoles()) {
            if (first == null) {
                first = role;
            } else if (!cornersOf(OCTAGON_SQUARE, role).equals(cornersOf(OCTAGON_SQUARE, first))) {
                second = role;
                break;
            }
        }
        assertNotNull(second, "scenario error: OctagonSquare should have roles with differing corners");

        CyclebuilderComms comms = standalone(OCTAGON_SQUARE);

        // getCurrentRole() reads the assigned edge's target, so assigning an edge that lands on a
        // role is how a robot comes to occupy it.
        comms.setAssignedEdge(0, someEdgeInto(first).getId());
        comms.promoteToPrimaryRoot();
        assertEquals(cornersOf(OCTAGON_SQUARE, first),
                trackedAmong(comms, allEdgeIds(OCTAGON_SQUARE)).keySet(),
                "scenario error: the first initialisation did not track the first role's corners");

        comms.setAssignedEdge(0, someEdgeInto(second).getId());
        comms.promoteToPrimaryRoot();

        assertEquals(cornersOf(OCTAGON_SQUARE, second),
                trackedAmong(comms, allEdgeIds(OCTAGON_SQUARE)).keySet(),
                "re-initialising kept corners from the previous role. initializeEdgeMap adds an "
                        + "entry per outgoing edge and removes nothing, so without a clear it cannot "
                        + "establish what its name promises -- a corner for every edge leaving this "
                        + "role, and no others.");
    }

    /** Any half-edge whose target is {@code role}, so assigning it puts a robot on that role. */
    private static HalfEdge someEdgeInto(Role role) {
        for (int id : allEdgeIds(OCTAGON_SQUARE)) {
            HalfEdge edge = OCTAGON_SQUARE.getHalfEdgeById(id);
            if (edge.getTarget().getId() == role.getId()) {
                return edge;
            }
        }
        throw new IllegalStateException("no edge into role " + role.getId());
    }
}
