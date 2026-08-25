package org.communicationModels.cycleBuildingComms;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Pure unit tests for the obligation set. Everything here is a property of the collection
 * rather than of any one tuple, which is the whole argument for the set being a class:
 * one-per-edge, deterministic order, fair rotation, and vacate-as-one-operation cannot be
 * enforced by {@link FaceObligation} and would otherwise rest on every call site
 * remembering to check first.
 */
class FaceObligationSetTest {

    @Test
    @DisplayName("a fresh set is empty and has nothing outstanding")
    void freshSetIsEmpty() {
        FaceObligationSet set = new FaceObligationSet();

        assertTrue(set.isEmpty());
        assertEquals(0, set.size());
        assertNull(set.findUnfulfilled());
        assertFalse(set.hasOutstanding());
    }

    @Test
    @DisplayName("findUnfulfilled returns null when every obligation is fulfilled")
    void findUnfulfilledReturnsNullWhenAllFulfilled() {
        FaceObligationSet set = new FaceObligationSet();
        set.getOrCreate(1, 10).fulfil(20, null);
        set.getOrCreate(1, 11).fulfil(21, null);

        assertNull(set.findUnfulfilled());
        assertFalse(set.hasOutstanding(), "a robot with every slot filled is free to process mail");
    }

    /**
     * The one-tuple-per-edge rule. Two certificates arriving for the same edge share one
     * obligation; that is what makes a filtered duplicate's late status an idempotent
     * no-op rather than a message with nowhere to route.
     */
    @Test
    @DisplayName("a second certificate on an edge reuses the existing obligation")
    void secondCertificateOnAnEdgeReusesTheExistingObligation() {
        FaceObligationSet set = new FaceObligationSet();

        FaceObligation first = set.getOrCreate(1, 10);
        FaceObligation second = set.getOrCreate(1, 10);

        assertSame(first, second, "one tuple per edge, or the set is not the invariant it claims");
        assertEquals(1, set.size());
    }

    /**
     * A face is traversed in one direction, so every walk reaching this robot owing a
     * given edge arrived from the same parent. A second parent claiming that edge means
     * the invariant broke upstream and the status this tuple routes would go to the wrong
     * robot -- loud in tests, survivable in the simulation, which is what {@code assert}
     * buys. This test also pins that assertions really are enabled by the test runner; if
     * they ever are not, it fails rather than quietly certifying nothing.
     */
    @Test
    @DisplayName("a conflicting parent on one edge trips an assertion")
    void conflictingParentOnAnEdgeIsAnAnomaly() {
        assertTrue(FaceObligationSetTest.class.desiredAssertionStatus(),
                "assertions are disabled, so getOrCreate's invariant check is inert here");

        FaceObligationSet set = new FaceObligationSet();
        set.getOrCreate(1, 10);

        assertThrows(AssertionError.class, () -> set.getOrCreate(2, 10),
                "two parents cannot both own one edge of a face");
    }

    @Test
    @DisplayName("distinct edges get distinct obligations")
    void distinctEdgesGetDistinctObligations() {
        FaceObligationSet set = new FaceObligationSet();

        FaceObligation ten = set.getOrCreate(1, 10);
        FaceObligation eleven = set.getOrCreate(1, 11);

        assertNotSame(ten, eleven);
        assertEquals(2, set.size());
        assertSame(ten, set.findByEdge(10));
        assertSame(eleven, set.findByEdge(11));
        assertNull(set.findByEdge(12));
    }

    @Test
    @DisplayName("findByChild picks the correct obligation when two exist")
    void findByChildPicksTheCorrectObligationWhenTwoExist() {
        FaceObligationSet set = new FaceObligationSet();
        FaceObligation ten = set.getOrCreate(1, 10);
        FaceObligation eleven = set.getOrCreate(1, 11);
        ten.fulfil(20, null);
        eleven.fulfil(21, null);

        assertSame(ten, set.findByChild(20));
        assertSame(eleven, set.findByChild(21));
        assertNull(set.findByChild(22));
    }

    /**
     * The concurrency the migration exists for: several children, on several faces, all in
     * flight at once, each routing its response back to its own parent.
     */
    @Test
    @DisplayName("two children route to two different parents")
    void twoChildrenRouteToTwoDifferentParents() {
        FaceObligationSet set = new FaceObligationSet();
        set.getOrCreate(1, 10).fulfil(20, null);
        set.getOrCreate(2, 11).fulfil(21, null);

        assertEquals(1, set.findByChild(20).getParentId());
        assertEquals(2, set.findByChild(21).getParentId());
    }

    @Test
    @DisplayName("remove is by identity, and removeByChild hands the obligation back")
    void removalSemantics() {
        FaceObligationSet set = new FaceObligationSet();
        FaceObligation held = set.getOrCreate(1, 10);
        org.simulation.Edge drawn = new org.simulation.Edge(1, 20);
        held.fulfil(20, drawn);

        // A value-equal but distinct object must not be able to remove the live entry.
        FaceObligation lookalike = new FaceObligation(1, 10);
        lookalike.fulfil(20, null);
        assertFalse(set.remove(lookalike), "removal must be by identity, not by value");
        assertEquals(1, set.size());

        FaceObligation removed = set.removeByChild(20);
        assertSame(held, removed);
        assertSame(drawn, removed.getChildEdge(),
                "the caller needs the drawn edge back to undraw it");
        assertTrue(set.isEmpty());
        assertNull(set.removeByChild(20));
    }

    /**
     * Fairness. Without a rotation the first-inserted obligation is returned every tick,
     * and since a robot services an outstanding obligation before dequeuing a message, a
     * later face would never be offered while an earlier one keeps being re-offered.
     */
    @Test
    @DisplayName("the cursor visits each outstanding obligation in turn, deterministically")
    void rotatingCursorVisitsEachObligationInTurn() {
        FaceObligationSet set = new FaceObligationSet();
        set.getOrCreate(1, 10);
        set.getOrCreate(1, 11);
        set.getOrCreate(1, 12);

        List<Integer> visited = new ArrayList<>();
        for (int i = 0; i < 6; i++) {
            visited.add(set.findUnfulfilled().getEdgeId());
        }

        assertEquals(List.of(10, 11, 12, 10, 11, 12), visited,
                "the rotation must be a stable round robin, never hash order and never a "
                        + "fixed first entry");
    }

    @Test
    @DisplayName("the rotation skips fulfilled obligations without losing its place")
    void rotationSkipsFulfilledObligations() {
        FaceObligationSet set = new FaceObligationSet();
        set.getOrCreate(1, 10);
        FaceObligation middle = set.getOrCreate(1, 11);
        set.getOrCreate(1, 12);

        middle.fulfil(21, null);

        assertEquals(10, set.findUnfulfilled().getEdgeId());
        assertEquals(12, set.findUnfulfilled().getEdgeId());
        assertEquals(10, set.findUnfulfilled().getEdgeId());
    }

    @Test
    @DisplayName("removal keeps the cursor pointing at the same next obligation")
    void removalKeepsTheCursorValid() {
        FaceObligationSet set = new FaceObligationSet();
        FaceObligation ten = set.getOrCreate(1, 10);
        set.getOrCreate(1, 11);
        set.getOrCreate(1, 12);

        assertEquals(10, set.findUnfulfilled().getEdgeId()); // cursor now at 11
        set.remove(ten);                                     // removing behind the cursor

        assertEquals(11, set.findUnfulfilled().getEdgeId(),
                "removing an entry before the cursor must not make the rotation skip one");
        assertEquals(12, set.findUnfulfilled().getEdgeId());
    }

    /**
     * Vacating is the only thing that clears the whole set, and it owes one rejection per
     * obligation -- each to its own parent -- before the state those parents are read from
     * is reset.
     */
    @Test
    @DisplayName("drainForVacate hands back every obligation and empties the set")
    void drainForVacateReturnsEveryObligationThenClears() {
        FaceObligationSet set = new FaceObligationSet();
        set.getOrCreate(1, 10);
        set.getOrCreate(2, 11);
        set.getOrCreate(3, 12);

        List<FaceObligation> drained = set.drainForVacate();

        assertEquals(3, drained.size(), "one rejection is owed per obligation, not one per robot");
        assertEquals(List.of(1, 2, 3), drained.stream().map(FaceObligation::getParentId).toList(),
                "each parent must be told independently, in a stable order");
        assertTrue(set.isEmpty());
        assertNull(set.findUnfulfilled());
    }

    @Test
    @DisplayName("clearAll empties the set and resets the rotation")
    void clearAllEmptiesTheSet() {
        FaceObligationSet set = new FaceObligationSet();
        set.getOrCreate(1, 10);
        set.getOrCreate(1, 11);
        set.findUnfulfilled();

        set.clearAll();

        assertTrue(set.isEmpty());
        set.getOrCreate(1, 20);
        assertEquals(20, set.findUnfulfilled().getEdgeId(),
                "a stale cursor must not survive a clear");
    }

    @Test
    @DisplayName("asList is insertion-ordered and read-only")
    void asListIsOrderedAndUnmodifiable() {
        FaceObligationSet set = new FaceObligationSet();
        set.getOrCreate(1, 12);
        set.getOrCreate(1, 10);
        set.getOrCreate(1, 11);

        assertEquals(List.of(12, 10, 11),
                set.asList().stream().map(FaceObligation::getEdgeId).toList(),
                "insertion order, never sorted and never hash order");
        assertThrows(UnsupportedOperationException.class,
                () -> set.asList().add(new FaceObligation(1, 13)));
    }

    /**
     * An empty set means nothing is in flight, not that this robot's faces are built. If
     * promotion ever keys off this, a robot promotes the moment its first status returns
     * and stops building everything else.
     */
    @Test
    @DisplayName("an empty set is not a completion signal")
    void emptySetIsTransientNotCompletion() {
        FaceObligationSet set = new FaceObligationSet();
        FaceObligation only = set.getOrCreate(1, 10);
        only.fulfil(20, null);
        set.removeByChild(20);

        assertTrue(set.isEmpty());
        assertFalse(set.hasOutstanding(),
                "this says the robot owes no offer right now -- promotion is decided by "
                        + "completedCycles, never by the obligation set being empty");
    }
}
