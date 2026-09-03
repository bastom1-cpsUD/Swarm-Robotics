package org.communicationModels.cycleBuildingComms.Messages;

/**
 * Reports that a walk broke below the sender: the child carrying the certificate left
 * communication range, taking the only copy with it.
 *
 * <p><strong>Not a rejection, and not a failure.</strong> A rejection means "I cannot take
 * this spot, offer it to someone else" -- acting on that here would make the parent replace
 * a robot that is perfectly fine and still holding its role. A failure means the face
 * cannot be built. This means neither: the face is still viable and the robots on it are
 * still correct. Only the evidence is gone.
 *
 * <p><strong>Why it has to travel rather than be handled locally.</strong> A certificate is
 * never stored in a field -- it lives in messages and rides back on the reject and status
 * paths -- so when the child vanishes no robot upstream holds a copy to re-offer. Only a robot
 * that started a face can mint a fresh certificate for it, with {@code hops = 0}, so the news has
 * to reach every robot that did.
 *
 * <p><strong>Addressed to nobody, and that is the whole design.</strong> This used to carry an
 * {@code initiatorId} and stop at the robot that minted the lost walk, exactly like a
 * {@code StatusMessage}. That was wrong, because a link carries <em>several</em> walks at once and
 * recorded only one of them: two roots racing to build the same face both push walks through the
 * same link, the second offer overwrote the first's record, and the report went to whichever root
 * offered last. The other one was never told, sat on an attempt that would never resolve, and
 * answered {@code N/A (Already building a face of my own)} for the rest of the run.
 *
 * <p>So it is not addressed at all. It climbs child -&gt; parent along the communication tuples,
 * every root it passes cancelling its own attempt on the face named by {@link #getLostOnEdgeId()},
 * and dies at the first robot with no parent to pass it to. That set -- the parent chain above the
 * break -- is exactly the set of walks that just died: every walk the reporting robot sent into the
 * departed child arrived over one tuple, from one parent, and the same holds at each hop up.
 * Nothing is over- or under-told, and no robot has to know who minted anything.
 *
 * <p><strong>Delivery is guaranteed, so no timeout backstop is needed.</strong> Every hop of
 * the upward path is a parent parked one lattice edge away and permanently in range -- a
 * robot only becomes a parent after it is in position, and a parked robot never loses
 * contention. {@code FaceClosureTest} pins every lattice's edge length strictly below
 * {@code COMM_RANGE}, which is what makes that true; if that guard ever fails, this
 * recovery path fails with it and a timeout becomes necessary.
 *
 * <p>Cascading departures are self-healing for the same reason: if the reporting robot also
 * vanishes, its own parent observes that and sends its own report.
 *
 * <p>On arrival a root marks the corner {@code attempted} rather than {@code unattempted}, so
 * it tries its other edges first and comes back to this one -- reusing the existing
 * prioritisation instead of spinning on a corner that may be short of candidates.
 */
public class CertificateLostMessage extends AbstractMessage {

    private final int lostOnEdgeId;
    private final int hops;

    /**
     * @param lostOnEdgeId the edge the <em>sender</em> was offered, i.e. the id of the tuple it
     *                     forwarded the lost walk through. See {@link #getLostOnEdgeId()}.
     */
    public CertificateLostMessage(int senderId, int recipient, int lostOnEdgeId) {
        this(senderId, recipient, lostOnEdgeId, 0);
    }

    /** The forwarding form: {@code hops} is the sender's own count plus one. */
    public CertificateLostMessage(int senderId, int recipient, int lostOnEdgeId, int hops) {
        super(senderId, recipient);
        this.lostOnEdgeId = lostOnEdgeId;
        this.hops = hops;
    }

    /**
     * The edge the sender was carrying the lost walk on -- the key of the tuple it forwarded
     * through. This is what identifies <em>which face</em> the report is about, and it is re-stamped
     * at every hop: a robot forwarding this replaces it with its own tuple's edge id.
     *
     * <p>A receiver's attempt is on the same walk iff it {@code owes} this edge, because owing it is
     * precisely what "I offered the sender this edge" means. The two conditions the receiver checks
     * -- owes this edge, and is bound to this sender -- are together exact, which matters: a walk on
     * a genuinely different face must not be cancelled, or its status comes home to a robot that has
     * already forgotten the attempt it belongs to.
     *
     * <p><strong>Not a {@code Face} id, and not the initiator's origin edge.</strong> A face id is a
     * face <em>type</em> -- all four outgoing edges of a role share one on a square lattice -- so
     * comparing those would cancel attempts on genuinely different faces. The origin edge names the
     * corner as the <em>initiator</em> sees it, which is a different half-edge at every other robot
     * on the walk. Only the per-hop edge is comparable at the robot doing the comparing.
     */
    public int getLostOnEdgeId() {
        return lostOnEdgeId;
    }

    /**
     * How many links this report has already climbed.
     *
     * <p>Pure guard. The upward path is a simple path, not a cycle -- the closing hop of a face is
     * handled by {@code evaluateReturningCertificate} and never opens a carried tuple, so there is
     * nothing to loop through -- and the chain ends at the first robot with no parent. But a return
     * message that nothing addresses is one that circulates until something else stops it, which is
     * the failure the old {@code initiatorId} prevented by construction. This is what replaces it.
     */
    public int getHops() {
        return hops;
    }

    /**{@inheritDoc}*/
    public String getMessageType() {
        return "Certificate Lost";
    }

    /**
     * Ranked with the other control messages. A lost certificate should be acted on before
     * new assignment traffic, for the same reason a status is: it frees the corner it names.
     */
    public int getPriority() {
        return 1;
    }

    @Override
    public String toString() {
        return super.toString() + "\n"
            + "Lost on Edge ID: " + lostOnEdgeId + "\n"
            + "Hops: " + hops;
    }

    @Override
    public boolean equals(Object o) {
        if(this == o) {
            return true;
        }
        if(!super.equals(o)) {
            return false;
        }
        if(!(o instanceof CertificateLostMessage)) {
            return false;
        }

        CertificateLostMessage other = (CertificateLostMessage) o;

        return lostOnEdgeId == other.getLostOnEdgeId()
            && hops == other.getHops();
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(super.hashCode(), lostOnEdgeId, hops);
    }
}
