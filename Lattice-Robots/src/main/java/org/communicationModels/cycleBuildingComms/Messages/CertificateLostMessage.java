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
 * paths -- so when the child vanishes no robot upstream holds a copy to re-offer. Only the
 * initiator can mint a fresh one, with {@code hops = 0}. So this propagates all the way up,
 * and an intermediate robot forwards it while <em>keeping its own tuple intact</em>: the
 * topology it describes is still correct, only the message in flight was lost.
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
 * <p>On arrival the initiator marks the corner {@code attempted} rather than
 * {@code unattempted}, so it tries its other edges first and comes back to this one --
 * reusing the existing prioritisation instead of spinning on a corner that may be short of
 * candidates.
 */
public class CertificateLostMessage extends AbstractMessage {

    private final int originVertexID;
    private final int originOutgoingEdgeID;

    public CertificateLostMessage(int senderId, int recipient, int originVertexID, int originOutgoingEdgeID) {
        super(senderId, recipient);
        this.originVertexID = originVertexID;
        this.originOutgoingEdgeID = originOutgoingEdgeID;
    }

    /** Which corner the lost walk belonged to, so the initiator knows what to relaunch. */
    public int getOriginVertexID() {
        return originVertexID;
    }

    public int getOriginOutgoingEdgeID() {
        return originOutgoingEdgeID;
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
            + "Beginning Edge of Cycle Vertex ID: " + originVertexID + " \n"
            + "Beginning Edge of Cycle Edge ID: " + originOutgoingEdgeID;
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

        return originVertexID == other.getOriginVertexID()
            && originOutgoingEdgeID == other.getOriginOutgoingEdgeID();
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(super.hashCode(), originVertexID, originOutgoingEdgeID);
    }
}