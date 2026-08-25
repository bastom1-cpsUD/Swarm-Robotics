package org.communicationModels.cycleBuildingComms;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.graphs.voltage.HalfEdge;

/**
 * The obligations one robot currently holds, at most one per outgoing edge.
 *
 * <p>Every rule this class enforces is a property of the <em>collection</em>, which is why
 * it is a class rather than a bare list on the comms system:
 *
 * <ul>
 *   <li><strong>One tuple per edge.</strong> {@link #getOrCreate} returns the existing
 *       entry rather than adding a second, so the rule cannot be broken by a call site
 *       forgetting to check first. This is what makes a duplicate certificate on an edge
 *       harmless -- both relay through the same tuple, and a filtered duplicate's late
 *       traffic is idempotent.</li>
 *   <li><strong>Deterministic order.</strong> Backed by an insertion-ordered list, not a
 *       hash map. The simulation buys reproducible asynchrony with staggered activation
 *       and no RNG anywhere; hash iteration order would smuggle order-dependence into face
 *       selection. A {@code HashMap<Integer, ...>} would be worse than random here -- for
 *       small edge ids it iterates in ascending order, which is a stable starvation bias
 *       that no test would ever flag.</li>
 *   <li><strong>Fair selection.</strong> {@link #findUnfulfilled()} resumes from where it
 *       last handed one out, so a low-numbered face cannot monopolise a robot that is
 *       servicing an obligation before dequeuing messages each tick.</li>
 *   <li><strong>Vacating is one operation.</strong> {@link #drainForVacate()} hands back
 *       every obligation and empties the set, so "tell each parent, then clear" cannot be
 *       written in the wrong order at any of the three sites that give up a site.</li>
 * </ul>
 *
 * <p>Linear scans throughout. A role has at most six outgoing edges in any shipped lattice
 * (six on Triangle, five on the snubs, fewer elsewhere), so a scan is both fast enough and
 * obviously order-stable, which the hash alternative is not.
 */
public class FaceObligationSet {

    private final List<FaceObligation> obligations = new ArrayList<>();

    /**
     * Where {@link #findUnfulfilled()} resumes. An index into {@link #obligations}, kept
     * valid by {@link #removeAt}: removing an entry at or before the cursor shifts it, so
     * the rotation neither skips an obligation nor revisits one.
     */
    private int cursor = 0;

    /**
     * The obligation for this edge, created if this robot does not already hold one.
     *
     * <p>An existing entry is returned unchanged, <em>including its parent</em>. That is
     * safe because of an invariant worth stating: the edge determines the face, a face is
     * traversed in one direction only, so every walk that reaches this robot owing this
     * edge arrived over the same incoming edge and therefore from the same parent. A
     * differing parent means that invariant has broken somewhere upstream, and the status
     * this tuple routes would go to the wrong robot -- so it is asserted rather than
     * silently accepted. Assertions are on under Gradle's test runner and off in the
     * simulation, which is the right split: fail loudly in tests, keep flying in the sim.
     *
     * @return the live obligation for {@code edgeId}, never null
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
        obligations.add(created);
        return created;
    }

    public FaceObligation findByEdge(HalfEdge edge) {
        return edge == null ? null : findByEdge(edge.getId());
    }

    /** By edge id, because protocol messages carry the id rather than the half-edge. */
    public FaceObligation findByEdge(int edgeId) {
        for (FaceObligation obligation : obligations) {
            if (obligation.getEdgeId() == edgeId) {
                return obligation;
            }
        }
        return null;
    }

    /**
     * The next obligation with no child, resuming from the last one handed out.
     *
     * <p>Rotating rather than always returning the first is what keeps a robot serving
     * several incident faces from starving the ones it happened to accept later.
     *
     * @return an outstanding obligation, or null if every one is fulfilled
     */
    public FaceObligation findUnfulfilled() {
        int size = obligations.size();
        if (size == 0) {
            return null;
        }

        if (cursor >= size) {
            cursor = 0;
        }

        for (int step = 0; step < size; step++) {
            int index = (cursor + step) % size;
            FaceObligation candidate = obligations.get(index);
            if (candidate.isUnfulfilled()) {
                cursor = (index + 1) % size;
                return candidate;
            }
        }
        return null;
    }

    public FaceObligation findByChild(int childId) {
        for (FaceObligation obligation : obligations) {
            if (obligation.matchesChild(childId)) {
                return obligation;
            }
        }
        return null;
    }

    /**
     * Removes exactly this obligation, by identity.
     *
     * <p>Identity, not {@code equals}: an obligation's child slot is mutable, so a value
     * comparison can miss the very entry the caller is holding if the slot changed since.
     *
     * @return true if it was present and removed
     */
    public boolean remove(FaceObligation obligation) {
        for (int i = 0; i < obligations.size(); i++) {
            if (obligations.get(i) == obligation) {
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
        for (int i = 0; i < obligations.size(); i++) {
            if (obligations.get(i).matchesChild(childId)) {
                FaceObligation removed = obligations.get(i);
                removeAt(i);
                return removed;
            }
        }
        return null;
    }

    /**
     * Hands back every obligation and empties the set, for a robot giving up its lattice
     * site.
     *
     * <p>Returning them rather than just clearing is the point: the caller owes one
     * retryable rejection <em>per obligation</em>, each to its own parent, and it has to
     * send them <em>before</em> the reset that clears the state they are read from. All
     * three sites that vacate a site already carry a comment saying so; this makes the
     * ordering structural instead of remembered.
     *
     * <p>Only vacating clears the whole set. A promotion must not call this -- a promoted
     * robot has not moved, so its topology is still real and it still owes every response.
     *
     * @return the obligations that were held, in insertion order
     */
    public List<FaceObligation> drainForVacate() {
        List<FaceObligation> drained = new ArrayList<>(obligations);
        clearAll();
        return drained;
    }

    /** Empties the set without reporting to anyone. Prefer {@link #drainForVacate()}. */
    public void clearAll() {
        obligations.clear();
        cursor = 0;
    }

    /** Read-only view in insertion order, for snapshots and logging. */
    public List<FaceObligation> asList() {
        return Collections.unmodifiableList(obligations);
    }

    public int size() {
        return obligations.size();
    }

    public boolean isEmpty() {
        return obligations.isEmpty();
    }

    /**
     * Whether this robot owes an offer on any edge. Note that this is <em>not</em> a
     * completion test: an empty set means nothing is in flight right now, which says
     * nothing about whether this robot's faces are built. Promotion is decided by
     * {@code completedCycles}, never by this.
     */
    public boolean hasOutstanding() {
        return findAnyUnfulfilled() != null;
    }

    /** Like {@link #findUnfulfilled()} but does not advance the cursor. */
    private FaceObligation findAnyUnfulfilled() {
        for (FaceObligation obligation : obligations) {
            if (obligation.isUnfulfilled()) {
                return obligation;
            }
        }
        return null;
    }

    @Override
    public String toString() {
        return "FaceObligationSet" + obligations;
    }

    /** Removes by index, keeping the rotation cursor pointing at the same next entry. */
    private void removeAt(int index) {
        obligations.remove(index);
        if (index < cursor) {
            cursor--;
        }
        if (cursor >= obligations.size()) {
            cursor = 0;
        }
    }
}
