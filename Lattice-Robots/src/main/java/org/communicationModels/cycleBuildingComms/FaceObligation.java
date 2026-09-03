package org.communicationModels.cycleBuildingComms;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Objects;
import java.util.Set;

import org.graphs.voltage.VoltageGraph;

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
 * <p><strong>Nothing removes a carried tuple.</strong> It is a communication link, not a
 * commitment to one walk: a robot in formation has fixed lattice neighbours, so
 * {@code (parent, edge) -> child} is a standing fact that outlives every walk that uses it. The
 * next certificate over this edge is forwarded to the same child with no candidate search at all,
 * which is the whole reason links are kept.
 * <ul>
 *   <li><em>Status</em> from the child -- forward it to {@link #getParentId()} and keep
 *       everything. Both robots are exactly where they were; the face being settled says
 *       nothing about whether they are still adjacent.</li>
 *   <li><em>Rejection</em> from the child -- {@link #release()} the child slot and
 *       {@link #ban(int)} the rejecter, then re-offer on the spot with the certificate the
 *       rejection handed back. Parent, edge and bans survive.</li>
 *   <li><em>Child observed gone</em> -- {@link #release()} the binding and report the loss
 *       upward, because the certificate cannot be recovered from here. The link itself stays:
 *       the site on the far side of the edge is still a site, and the next walk over it rebinds
 *       with the bans already learned.</li>
 * </ul>
 *
 * <p>The one thing that does remove a carried tuple is this robot vacating its lattice site --
 * see {@link FaceObligationSet#drainForVacate()} -- because that is the one event that makes the
 * adjacency untrue. The <em>attempt</em> is different in kind and is transient by design: it is
 * opened when a root picks a corner and dropped when that corner's status comes home.
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

    // Nothing here records which walk is in flight, and nothing should.
    //
    // There used to be an `inFlightInitiatorId`: the minter of the most recent walk sent over this
    // link, stamped by every offer, so that a departed child could be reported to the robot able to
    // mint a replacement. One scalar, and its javadoc said so plainly -- "the latest wins".
    //
    // But a link carries SEVERAL walks at once. Two roots racing to build the same face both push
    // walks through this link, the second offer overwrote the first's record, and the loss went to
    // whichever root offered last while the other sat on an attempt that would never resolve. The
    // slot could not be widened into a set either, because the routing it fed was the problem, not
    // its width: a report addressed to one robot is the wrong shape for news that concerns all of
    // them.
    //
    // CertificateLostMessage is addressed to nobody now. It climbs the parent chain and every root
    // on it cancels its own attempt, so no link has to remember whose walk it carried -- see that
    // class for why the parent chain is exactly the set of walks that died.

    // No visualization edge is held here any more. There used to be one, so that undoing a
    // speculative offer could remove exactly the object that offer had drawn, by reference,
    // leaving any other edge to the same robot alone. Two things retired it. Edge gained value
    // equality, so a robot holds one object per endpoint pair and a stored reference is often not
    // the object actually on the robot -- it answered a question it could no longer answer
    // truthfully. And removal now asks the right question anyway: CyclebuilderComms.undrawEdgeTo
    // keeps or drops an edge on whether a permanent link still connects the two robots, which is a
    // property of this collection, not of who drew what.

    /** Robots that have declined this face. Scoped here, so bans die with the obligation. */
    private final Set<Integer> unableToDoAssignmentIds = new LinkedHashSet<>();

    public FaceObligation(int parentId, int edgeId) {
        this.parentId = parentId;
        this.edgeId = edgeId;
        this.childId = null;
    }

    /**
     * Records the robot this edge has been offered to.
     *
     * @param childId the robot now carrying the certificate onward
     */
    public void fulfil(int childId) {
        this.childId = childId;
    }

    /**
     * Clears the child slot after a rejection, keeping parent, edge and bans so the offer
     * can go to a different candidate next tick.
     *
     * <p>Returns nothing. It used to hand back the visualization edge for the caller to remove, and
     * that made the ordering easy to get right by accident; now that
     * {@code CyclebuilderComms.undrawEdgeTo} decides on topology instead, a caller has to read the
     * child id <em>before</em> calling this and undraw <em>after</em> -- because clearing the slot
     * is exactly what makes this link stop vouching for that robot.
     */
    public void release() {
        this.childId = null;
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

    /**
     * Forgets every exclusion on this link, the walk that accumulated them having resolved.
     *
     * <p><strong>A ban is scoped to one walk, not to the link.</strong> It exists to make a single
     * offer sequence finite -- offer, refuse, ban, offer the next candidate -- and once that walk
     * has resolved it is stale evidence about where robots were, not where they are.
     *
     * <p>That distinction did not matter while a tuple was removed when its walk resolved, because
     * the bans went with it. It matters completely now that links are permanent: a robot that
     * refused once, from wherever it happened to be standing at the time, would be excluded from
     * that link <em>forever</em>. When it later settles onto the very site the link points at, the
     * exclusion hides it -- and because {@code findBestNeighborForEdge} keeps its ban check ahead
     * of its exact-position match (deliberately, or the offer loop loses its bound), the offerer
     * skips the actual occupant and sends somebody else to a site that is already taken.
     */
    public void clearForResolvedWalk() {
        this.unableToDoAssignmentIds.clear();
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
