package org.communicationModels.cycleBuildingComms.Messages;

public class RejectAssignmentMessage extends AbstractMessage{
    private int originVertexID;
    private int originOutgoingEdgeID;
    private boolean isRetryable;
    private int viaEdgeId;
    private VoltageCertificate certificate;

    /** A rejection carrying no certificate -- used where the walk never had one to return. */
    public RejectAssignmentMessage(int senderId, int recipient, int originVertexID, int originOutgoingEdgeID, boolean isRetryable, int viaEdgeId) {
        this(senderId, recipient, originVertexID, originOutgoingEdgeID, isRetryable, viaEdgeId, null);
    }

    /**
     * @param viaEdgeId   the edge the rejecter was offered -- see {@link #getViaEdgeId()}.
     * @param certificate the certificate that arrived with the rejected offer, handed back
     *                    to the robot that made it. This is what lets that robot re-offer
     *                    on the spot to a different candidate without ever having kept a
     *                    copy: the certificate it needs is the one it just got back.
     */
    public RejectAssignmentMessage(int senderId, int recipient, int originVertexID, int originOutgoingEdgeID, boolean isRetryable, int viaEdgeId, VoltageCertificate certificate) {
        super(senderId, recipient);
        this.originVertexID = originVertexID;
        this.originOutgoingEdgeID = originOutgoingEdgeID;
        this.isRetryable = isRetryable;
        this.viaEdgeId = viaEdgeId;
        this.certificate = certificate;
    }

    /**
     * The edge this rejection is about -- the one the sender was offered, which is the edge exactly
     * one of the receiver's links owes onward.
     *
     * <p>Same role as {@link StatusMessage#getViaEdgeId()} and for the same reason: a robot can hold
     * two links to one neighbour, so "the tuple whose child sent this" does not identify which offer
     * is being refused, and releasing the wrong one strands a live walk while re-offering somebody
     * else's certificate.
     *
     * <p>Unlike a status, a rejection travels exactly one hop, so this is never rewritten.
     */
    public int getViaEdgeId() {
        return viaEdgeId;
    }

    /** The certificate that travelled with the offer being rejected; may be null. */
    public VoltageCertificate getCertificate() {
        return certificate;
    }

    public int getOriginVertexID() {
        return originVertexID;
    }

    public int getOriginOutgoingEdgeID() {
        return originOutgoingEdgeID;
    }

    /**{@inheritDoc}*/
    public String getMessageType() {
        return "Rejection";
    }

    public boolean isRetryable() {
        return isRetryable;
    }

    public int getPriority() {
        return 1;
    }

    @Override
    public String toString() {
        return super.toString() + "\n"
            + "Beginning Edge of Cycle Vertex ID: " + originVertexID + " \n"
            + "Beginning Edge of Cycle Edge ID: " + originOutgoingEdgeID + "\n"
            + "Retryable: " + isRetryable + "\n"
            + "Offered on my link for edge: " + viaEdgeId;
    }
    
    @Override
    public boolean equals(Object o) {
        if(this == o) {
            return true;
        }
        if(!super.equals(o)) {
            return false;
        }
        if(!(o instanceof RejectAssignmentMessage)) {
            return false;
        }

        RejectAssignmentMessage other = (RejectAssignmentMessage) o;

        return originVertexID == other.getOriginVertexID()
            && originOutgoingEdgeID == other.getOriginOutgoingEdgeID()
            && isRetryable == other.isRetryable()
            && viaEdgeId == other.getViaEdgeId();
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(super.hashCode(), originVertexID, originOutgoingEdgeID, isRetryable, viaEdgeId);
    }
}
