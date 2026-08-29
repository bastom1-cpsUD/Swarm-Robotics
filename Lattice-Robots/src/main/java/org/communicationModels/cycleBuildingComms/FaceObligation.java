package org.communicationModels.cycleBuildingComms;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Objects;
import java.util.Set;

import org.graphs.voltage.VoltageGraph;
import org.simulation.Edge;

/**
 * One robot's commitment to carry a face certificate across one of its outgoing edges:
 * who handed it the walk, which edge it owes onward, and who it has offered that edge to.
 *
 * <p>The tuple is {@code (parentId, childId, edgeId, bans)}. An obligation is
 * <strong>outstanding</strong> exactly while {@link #isUnfulfilled()} -- a null child.
 * Once a child is recorded the robot is free again: a response will arrive, route through
 * this tuple, and clear it. That per-edge condition is what replaces the old per-robot
 * {@code pendingChildID} gate, and it is what lets one robot serve several incident faces
 * at once.
 *
 * <p><strong>The certificate is deliberately not stored here.</strong> It travels in
 * messages and is handed back on the reject and status paths, so the robot that made an
 * offer gets the exact certificate it sent and can re-offer without ever having kept a
 * copy. A convenience {@code cert} field here would be the obvious shortcut for a robot
 * still in transit, and it silently breaks the one-tuple-per-edge rule as soon as two
 * certificates target the same edge before either is forwarded.
 *
 * <p><strong>Three outcomes, and only one of them removes the tuple.</strong>
 * <ul>
 *   <li><em>Status</em> from the child -- forward it to {@link #getParentId()} and remove
 *       the tuple. The face is settled either way, success or failure.</li>
 *   <li><em>Rejection</em> from the child -- {@link #release()} the child slot and
 *       {@link #ban(int)} the rejecter, then re-offer on the spot. The tuple survives with
 *       its parent, edge and bans intact. A rejection that removed the tuple would
 *       propagate up the chain, which is the behaviour this design exists to stop.</li>
 *   <li><em>Child observed gone</em> -- remove the tuple and report the loss upward,
 *       because the certificate cannot be recovered from here.</li>
 * </ul>
 *
 * <p>Lifetime keys on <strong>position, not role</strong>. A promotion leaves the robot
 * exactly where it is, so its topology is still real and its obligations survive.
 * Vacating the lattice site invalidates every one of them at once -- see
 * {@link FaceObligationSet#drainForVacate()}.
 *
 * <p>Mutable, unlike {@code CommunicationSystem.ClaimEntry} which is an immutable record.
 * The ban list grows in place and the child slot is filled and released repeatedly; every
 * field is touched only from this robot's own tick task, so there is no sharing to
 * protect against.
 */
public class FaceObligation {

    /**
     * The parent of a face this robot is building <em>for itself</em>: there isn't one.
     *
     * <p>A carried obligation is owed <em>to somebody</em> -- that is what makes it an
     * obligation, and {@link #getParentId()} names them. A face this robot initiated is owed
     * to nobody; it is an ambition rather than a debt, and nothing will ever be reported
     * upward for it. The sentinel says so directly, where the previous spelling
     * ({@code parentId == self}) said it by riddle -- a robot listed as its own parent.
     *
     * <p>Kept as a value on the tuple even though {@link FaceObligationSet} holds initiated
     * faces in a slot of their own, because the two answer at different times. The container
     * knows which is which only while it still holds the tuple; the sentinel travels with the
     * tuple after it has been removed or drained, which is exactly when
     * {@code forwardRejectionToParent} and {@code reportCertificateLost} need to ask.
     */
    public static final int NO_PARENT = -1;

    private final int parentId;
    private final int edgeId;

    private Integer childId;

    /**
     * The visualization edge drawn for the current child, held here rather than in one
     * field on the comms system.
     *
     * <p>{@code CyclebuilderComms.removeEdgeToChild} removes this <em>by reference</em>,
     * deliberately, so that undoing one speculative offer leaves any other already-valid
     * edge to the same robot id alone. With several obligations live at once a child id no
     * longer identifies which edge object to pull, so the obligation has to own it.
     */
    private Edge childEdge;

    /** Robots that have declined this face. Scoped here, so bans die with the obligation. */
    private final Set<Integer> unableToDoAssignmentIds = new LinkedHashSet<>();

    public FaceObligation(int parentId, int edgeId) {
        this.parentId = parentId;
        this.edgeId = edgeId;
        this.childId = null;
        this.childEdge = null;
    }

    /**
     * Records the robot this edge has been offered to.
     *
     * @param childId   the robot now carrying the certificate onward
     * @param childEdge the visualization edge drawn for that offer; may be null
     */
    public void fulfil(int childId, Edge childEdge) {
        this.childId = childId;
        this.childEdge = childEdge;
    }

    /**
     * Clears the child slot after a rejection, keeping parent, edge and bans so the offer
     * can go to a different candidate next tick.
     *
     * @return the visualization edge that was drawn for the released child, for the caller
     *         to remove from the robot; null if there was none
     */
    public Edge release() {
        Edge released = this.childEdge;
        this.childId = null;
        this.childEdge = null;
        return released;
    }

    /** Outstanding: nobody is carrying this edge onward yet. */
    public boolean isUnfulfilled() {
        return this.childId == null;
    }

    public boolean matchesChild(int childId) {
        return this.childId != null && this.childId == childId;
    }

    /**
     * The face this obligation belongs to. Stored alongside the edge key rather than
     * replacing it: {@code Face} is a face <em>type</em>, not an instance, so several of a
     * role's outgoing edges can share one face id -- all four on a square lattice, which
     * is why rekeying anything from edge to face collapses four corners into one.
     */
    public int faceId(VoltageGraph graph) {
        return graph.getHalfEdgeById(edgeId).getFace().getId();
    }

    public int getParentId() {
        return parentId;
    }

    /** The robot carrying this edge onward, or null while the obligation is outstanding. */
    public Integer getChildId() {
        return childId;
    }

    public int getEdgeId() {
        return edgeId;
    }

    public Edge getChildEdge() {
        return childEdge;
    }

    /** Excludes a robot from this face only. A ban on one face says nothing about another. */
    public void ban(int bannedId) {
        this.unableToDoAssignmentIds.add(bannedId);
    }

    public boolean isBanned(int robotId) {
        return this.unableToDoAssignmentIds.contains(robotId);
    }

    /** Read-only view, for snapshots and logging. */
    public Set<Integer> getBans() {
        return Collections.unmodifiableSet(unableToDoAssignmentIds);
    }

    @Override
    public String toString() {
        return "FaceObligation[parent: " + parentId
                + " -> edge " + edgeId
                + " -> child: " + (childId == null ? "none" : childId)
                + (unableToDoAssignmentIds.isEmpty() ? "" : " banned: " + unableToDoAssignmentIds)
                + "]";
    }

    /**
     * Value equality across the whole tuple, bans excluded.
     *
     * <p>Note that {@code childId} is mutable, so two references to the same obligation can
     * compare unequal across a {@link #fulfil} or {@link #release}. Anything that needs to
     * find <em>this</em> obligation again -- removal, in particular -- should compare by
     * identity rather than relying on this.
     */
    @Override
    public boolean equals(Object o) {
        if(this == o) return true;

        if(!(o instanceof FaceObligation)) return false;

        FaceObligation other = (FaceObligation) o;
        return this.parentId == other.parentId
            && this.edgeId == other.edgeId
            && Objects.equals(this.childId, other.childId);
    }

    @Override
    public int hashCode() {
        return Objects.hash(parentId, childId, edgeId);
    }
}
