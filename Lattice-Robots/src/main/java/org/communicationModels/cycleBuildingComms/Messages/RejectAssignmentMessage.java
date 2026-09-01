package org.communicationModels.cycleBuildingComms.Messages;

public class RejectAssignmentMessage extends AbstractMessage{
    private int originVertexID;
    private int originOutgoingEdgeID;
    private boolean isRetryable;
    private VoltageCertificate certificate;

    /** A rejection carrying no certificate -- used where the walk never had one to return. */
    public RejectAssignmentMessage(int senderId, int recipient, int originVertexID, int originOutgoingEdgeID, boolean isRetryable) {
        this(senderId, recipient, originVertexID, originOutgoingEdgeID, isRetryable, null);
    }

    /**
     * @param certificate the certificate that arrived with the rejected offer, handed back
     *                    to the robot that made it. This is what lets that robot re-offer
     *                    on the spot to a different candidate without ever having kept a
     *                    copy: the certificate it needs is the one it just got back.
     */
    public RejectAssignmentMessage(int senderId, int recipient, int originVertexID, int originOutgoingEdgeID, boolean isRetryable, VoltageCertificate certificate) {
        super(senderId, recipient);
        this.originVertexID = originVertexID;
        this.originOutgoingEdgeID = originOutgoingEdgeID;
        this.isRetryable = isRetryable;
        this.certificate = certificate;
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
            + "Retryable: " + isRetryable;
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
            && isRetryable == other.isRetryable();
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(super.hashCode(), originVertexID, originOutgoingEdgeID, isRetryable);
    }
}
