package org.communicationModels.cycleBuildingComms;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.graphs.voltage.HalfEdge;

/**
 * The faces one robot is working on: the walks it is <em>carrying</em> for other robots, and
 * the one face it is <em>attempting</em> for itself.
 *
 * <p><strong>Those are two different kinds of thing, and this class keeps them in two
 * different places.</strong> A carried obligation is owed to somebody -- it names a parent,
 * a response will travel back to that parent, and it is keyed on the <em>incoming</em> edge
 * the walk arrived over. An attempted cycle is owed to nobody: this robot started it, no
 * report goes upward, and it is keyed on the <em>outgoing</em> edge being built, which is
 * also the key {@code completedCycles} uses for that corner.
 *
 * <p>They used to share one list, and every read had to ask which kind it was holding. On a
 * single-role lattice the two key spaces overlap -- edge 0 can be both "the edge I was
 * assigned" and "the edge I am building" -- so a single-slot lookup could not hold both, and
 * a root in a ring of roots would refuse its neighbours because its own tuple was sitting on
 * the key theirs needed. Separating the containers dissolves that rather than working around
 * it: {@link #getOrCreate} now admits only carried tuples, so its stated invariant is true
 * with no exceptions to reason around, and {@link FaceObligation#getParentId()} is always a
 * real robot.
 *
 * <p>The two are still enumerated together where that is what a caller means --
 * {@link #findUnfulfilled()} rotates over both, {@link #asList()} reports both -- so no
 * caller has to remember to check the slot separately. That is the whole reason the slot
 * lives in here rather than as a field on the comms system: enumeration is the operation that
 * would otherwise silently forget it.
 *
 * <p>Every rule this class enforces is a property of the <em>collection</em>:
 *
 * <ul>
 *   <li><strong>Carried tuples are permanent; the attempt is not.</strong> A carried tuple is a
 *       communication link and is dropped only when this robot vacates its site
 *       ({@link #drainForVacate()}); a status resolving does not touch it. The attempt is an
 *       in-flight marker, opened by {@link #beginAttempt} and dropped by {@link #clearAttempt}
 *       when its corner's status comes home. Conflating the two is what a persistent-link design
 *       must not do: read "a tuple exists" as "a walk is in flight" and every corner a
 *       neighbour's walk has ever crossed becomes permanently unavailable.</li>
 *   <li><strong>One carried tuple per edge.</strong> {@link #getOrCreate} returns the
 *       existing entry rather than adding a second, so the rule cannot be broken by a call
 *       site forgetting to check first. This is what makes a duplicate certificate on an edge
 *       harmless -- both relay through the same tuple, one after the other, and each closes at
 *       its own initiator.</li>
 *   <li><strong>At most one attempt.</strong> A robot builds one face of its own at a time;
 *       all of its corners draw candidates from the same neighbourhood, so starting a second
 *       before the first has a walk in flight spends the same robots twice.</li>
 *   <li><strong>Deterministic order.</strong> Backed by an insertion-ordered list, not a hash
 *       map. The simulation buys reproducible asynchrony with staggered activation and no RNG
 *       anywhere; hash iteration order would smuggle order-dependence into face selection. A
 *       {@code HashMap<Integer, ...>} would be worse than random here -- for small edge ids it
 *       iterates in ascending order, which is a stable starvation bias that no test would
 *       ever flag.</li>
 *   <li><strong>Fair selection.</strong> {@link #findUnfulfilled()} resumes from where it last
 *       handed one out, so no face can monopolise a robot that is servicing one obligation
 *       before dequeuing messages each tick.</li>
 *   <li><strong>Vacating is one operation.</strong> {@link #drainForVacate()} hands back
 *       everything and empties both containers, so "tell each parent, then clear" cannot be
 *       written in the wrong order at any of the three sites that give up a site.</li>
 * </ul>
 *
 * <p>Linear scans throughout. A role has at most six outgoing edges in any shipped lattice
 * (six on Triangle, five on the snubs, fewer elsewhere), so a scan is both fast enough and
 * obviously order-stable, which the hash alternative is not.
 */
public class FaceObligationSet {

    /**
     * Walks this robot is carrying for other robots, keyed on the incoming edge each arrived
     * over. Homogeneous: every entry has a real parent waiting on it.
     */
    private final List<FaceObligation> carried = new ArrayList<>();

    /**
     * The one face this robot is building for itself, keyed on the outgoing edge it is
     * building, or null.
     *
     * <p>Held apart from {@link #carried} rather than in it. It is not owed to anyone, it is
     * keyed in a different space, and it is structurally singular -- three properties none of
     * the carried tuples have, and which a shared list can only express by having every
     * reader test for them.
     */
    private FaceObligation myAttemptedCycle;

    /**
     * Where {@link #findUnfulfilled()} resumes. An index into the virtual sequence
     * {@code carried ++ [myAttemptedCycle]}, kept valid by {@link #removeAt}: removing an
     * entry at or before the cursor shifts it, so the rotation neither skips an obligation
     * nor revisits one.
     */
    private int cursor = 0;

    /*
        ////////////////////////
        CARRIED WALKS
        ////////////////////////
     */

    /**
     * The carried obligation for this incoming edge, created if this robot does not already
     * hold one.
     *
     * <p>An existing entry is returned unchanged, <em>including its parent</em>. That is safe
     * because of an invariant worth stating: the edge determines the face, a face is
     * traversed in one direction, so every walk that reaches this robot owing this edge
     * arrived over the same incoming edge and therefore from the same parent. A differing
     * parent means that invariant has broken somewhere upstream, and the status this tuple
     * routes would go to the wrong robot -- so it is asserted rather than silently accepted.
     * Assertions are on under Gradle's test runner and off in the simulation, which is the
     * right split: fail loudly in tests, keep flying in the sim.
     *
     * <p>The invariant is now literally true. It used to have one exception -- a root's own
     * face, which has no parent and is keyed in the outgoing space -- that had to be reasoned
     * around every time this assertion was read. That face is {@link #beginAttempt} now.
     *
     * @return the live carried obligation for {@code edgeId}, never null
     */
    public FaceObligation getOrCreate(int parentId, int edgeId) {
        FaceObligation existing = findByEdge(edgeId);
        if (existing != null) {
            assert existing.getParentId() == parentId
                    : "edge " + edgeId + " already owed to parent " + existing.getParentId()
                      + ", but parent " + parentId + " is claiming it; a face is traversed in "
                      + "one direction, so two parents cannot both own one edge of it";
            return existing;
        }

        FaceObligation created = new FaceObligation(parentId, edgeId);
        carried.add(created);
        return created;
    }

    public FaceObligation findByEdge(HalfEdge edge) {
        return edge == null ? null : findByEdge(edge.getId());
    }

    /**
     * The <strong>carried</strong> obligation on this incoming edge, by id -- protocol
     * messages carry the id rather than the half-edge.
     *
     * <p>Deliberately does not consider {@link #getAttempt()}. The two are keyed in different
     * spaces -- incoming versus outgoing -- so an id that matches both matches them for
     * unrelated reasons, and every caller of this is asking about a walk that arrived from
     * outside. Ask {@link #getAttempt()} directly for this robot's own face.
     */
    public FaceObligation findByEdge(int edgeId) {
        for (FaceObligation obligation : carried) {
            if (obligation.getEdgeId() == edgeId) {
                return obligation;
            }
        }
        return null;
    }

    /** How many walks this robot is carrying, excluding its own attempt. */
    public int carriedSize() {
        return carried.size();
    }

    /*
        ////////////////////////
        THIS ROBOT'S OWN FACE
        ////////////////////////
     */

    /**
     * Opens this robot's attempt on one of its own outgoing edges, or returns the attempt
     * already in progress.
     *
     * <p>Parented to {@link FaceObligation#NO_PARENT}: nobody handed this face over and
     * nothing will be reported upward for it.
     *
     * @return the live attempt, never null
     */
    public FaceObligation beginAttempt(int outgoingEdgeId) {
        if (myAttemptedCycle == null) {
            myAttemptedCycle = new FaceObligation(FaceObligation.NO_PARENT, outgoingEdgeId);
        }
        return myAttemptedCycle;
    }

    /** The face this robot is building for itself, or null. */
    public FaceObligation getAttempt() {
        return myAttemptedCycle;
    }

    /** Whether this exact tuple is the attempt, by identity. */
    public boolean isAttempt(FaceObligation obligation) {
        return obligation != null && obligation == myAttemptedCycle;
    }

    /** Abandons the attempt, keeping everything this robot is carrying for others. */
    public void clearAttempt() {
        myAttemptedCycle = null;
        clampCursor();
    }

    /*
        ////////////////////////
        BOTH KINDS TOGETHER
        ////////////////////////
     */

    /**
     * The next obligation with no child, resuming from the last one handed out, across
     * <em>both</em> the carried walks and this robot's own attempt.
     *
     * <p>Rotating rather than always returning the first is what keeps a robot serving
     * several incident faces from starving the ones it happened to accept later.
     *
     * <p>The attempt sits last in the rotation, which is a mild and deliberate priority: a
     * carried walk has another robot blocked on it, whereas this robot's own face is blocked
     * on nobody but itself. It still gets its turn every sweep.
     *
     * @return an outstanding obligation, or null if every one is fulfilled
     */
    public FaceObligation findUnfulfilled() {
        int total = size();
        if (total == 0) {
            return null;
        }

        if (cursor >= total) {
            cursor = 0;
        }

        for (int step = 0; step < total; step++) {
            int index = (cursor + step) % total;
            FaceObligation candidate = at(index);
            if (candidate != null && candidate.isUnfulfilled()) {
                cursor = (index + 1) % total;
                return candidate;
            }
        }
        return null;
    }

    /**
     * Like {@link #findUnfulfilled()} but does <em>not</em> advance the rotation cursor.
     *
     * <p>Use this wherever an outstanding obligation is being <em>asked about</em> rather
     * than taken -- rotating on a query would let a mere look skip the obligation that a
     * later {@link #findUnfulfilled()} was meant to hand out.
     */
    public FaceObligation peekUnfulfilled() {
        for (FaceObligation obligation : carried) {
            if (obligation.isUnfulfilled()) {
                return obligation;
            }
        }
        return myAttemptedCycle != null && myAttemptedCycle.isUnfulfilled() ? myAttemptedCycle : null;
    }

    /**
     * The link this robot offered to that child <em>for this particular walk</em>, falling back to
     * the child alone when nothing records the walk.
     *
     * <p>Needed because the attempt is allowed to duplicate a carried link. A root building corner
     * {@code c} and relaying somebody else's walk that also owes {@code c} offers both to the same
     * neighbour, so two tuples name that child -- and {@link #findByChild} answers with the carried
     * one, because it scans {@link #carried} first. A response routed through the wrong one of the
     * two marks the wrong corner and travels to the wrong parent.
     *
     * <p>{@link FaceObligation#getInFlightInitiator()} separates them: it records who minted the
     * walk each link most recently carried, and the two duplicates are carrying different walks or
     * they would not be duplicates. The fallback keeps behaviour unchanged wherever nothing is
     * ambiguous -- a response whose walk this robot has already forgotten still routes by child,
     * which is what it did before this existed.
     */
    public FaceObligation findByChildForWalk(int childId, int initiatorId) {
        for (FaceObligation obligation : carried) {
            if (obligation.matchesChild(childId) && obligation.getInFlightInitiator() == initiatorId) {
                return obligation;
            }
        }
        if (myAttemptedCycle != null && myAttemptedCycle.matchesChild(childId)
                && myAttemptedCycle.getInFlightInitiator() == initiatorId) {
            return myAttemptedCycle;
        }
        return findByChild(childId);
    }

    /** The obligation this robot offered to that child, carried or its own. */
    public FaceObligation findByChild(int childId) {
        for (FaceObligation obligation : carried) {
            if (obligation.matchesChild(childId)) {
                return obligation;
            }
        }
        return myAttemptedCycle != null && myAttemptedCycle.matchesChild(childId)
                ? myAttemptedCycle : null;
    }

    /**
     * Removes exactly this obligation, by identity, from wherever it is held.
     *
     * <p>Identity, not {@code equals}: an obligation's child slot is mutable, so a value
     * comparison can miss the very entry the caller is holding if the slot changed since.
     *
     * @return true if it was present and removed
     */
    public boolean remove(FaceObligation obligation) {
        if (isAttempt(obligation)) {
            clearAttempt();
            return true;
        }
        for (int i = 0; i < carried.size(); i++) {
            if (carried.get(i) == obligation) {
                removeAt(i);
                return true;
            }
        }
        return false;
    }

    /**
     * Removes the obligation whose child is this robot -- the status path knows the child,
     * not the tuple.
     *
     * @return the removed obligation, so the caller can retrieve its
     *         {@link FaceObligation#getChildEdge()}; null if none matched
     */
    public FaceObligation removeByChild(int childId) {
        for (int i = 0; i < carried.size(); i++) {
            if (carried.get(i).matchesChild(childId)) {
                FaceObligation removed = carried.get(i);
                removeAt(i);
                return removed;
            }
        }
        if (myAttemptedCycle != null && myAttemptedCycle.matchesChild(childId)) {
            FaceObligation removed = myAttemptedCycle;
            clearAttempt();
            return removed;
        }
        return null;
    }

    /**
     * Hands back everything this robot holds and empties both containers, for a robot giving
     * up its lattice site.
     *
     * <p>Returning them rather than just clearing is the point: the caller owes one retryable
     * rejection <em>per carried obligation</em>, each to its own parent, and it has to send
     * them <em>before</em> the reset that clears the state they are read from. All three
     * sites that vacate a site already carry a comment saying so; this makes the ordering
     * structural instead of remembered.
     *
     * <p>The attempt comes back too, last, and the caller will find it has
     * {@link FaceObligation#NO_PARENT} -- which is how it knows there is nobody to tell. That
     * is the case where the sentinel earns its keep: once drained, this class no longer knows
     * which of the returned tuples was the attempt.
     *
     * <p>Only vacating clears everything. A promotion must not call this -- a promoted robot
     * has not moved, so its topology is still real and it still owes every response.
     *
     * @return the obligations that were held, carried ones first, in insertion order
     */
    public List<FaceObligation> drainForVacate() {
        List<FaceObligation> drained = asList();
        clearAll();
        return drained;
    }

    /** Empties both containers without reporting to anyone. Prefer {@link #drainForVacate()}. */
    public void clearAll() {
        carried.clear();
        myAttemptedCycle = null;
        cursor = 0;
    }

    /**
     * Read-only snapshot of everything held, carried walks first and the attempt last, for
     * snapshots, logging, and the departed-child sweep.
     *
     * <p>A copy rather than a view, because it spans two containers. Callers that mutate
     * while iterating -- {@code collectDepartedChildren} does -- are safe on it.
     */
    public List<FaceObligation> asList() {
        List<FaceObligation> all = new ArrayList<>(carried.size() + 1);
        all.addAll(carried);
        if (myAttemptedCycle != null) {
            all.add(myAttemptedCycle);
        }
        return Collections.unmodifiableList(all);
    }

    /** Everything held: carried walks plus the attempt, if there is one. */
    public int size() {
        return carried.size() + (myAttemptedCycle == null ? 0 : 1);
    }

    public boolean isEmpty() {
        return size() == 0;
    }

    /**
     * Whether this robot owes an offer on anything. Note that this is <em>not</em> a
     * completion test: an empty set means nothing is in flight right now, which says nothing
     * about whether this robot's faces are built. Promotion is decided by
     * {@code completedCycles}, never by this.
     */
    public boolean hasOutstanding() {
        return peekUnfulfilled() != null;
    }

    @Override
    public String toString() {
        return "FaceObligationSet" + asList();
    }

    /** The virtual sequence {@code carried ++ [myAttemptedCycle]}, by index. */
    private FaceObligation at(int index) {
        return index < carried.size() ? carried.get(index) : myAttemptedCycle;
    }

    /** Removes a carried entry by index, keeping the rotation cursor on the same next entry. */
    private void removeAt(int index) {
        carried.remove(index);
        if (index < cursor) {
            cursor--;
        }
        clampCursor();
    }

    private void clampCursor() {
        if (cursor >= size()) {
            cursor = 0;
        }
    }
}
