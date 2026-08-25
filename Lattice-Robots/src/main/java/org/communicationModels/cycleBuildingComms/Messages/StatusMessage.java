package org.communicationModels.cycleBuildingComms.Messages;

public class StatusMessage extends AbstractMessage {
    private boolean isSuccessful;
    private int originVertexID;
    private int originOutgoingEdgeID;
    private VoltageCertificate certificate;

    /** A status carrying no certificate -- used where the walk never had one to return. */
    public StatusMessage(int senderId, int recipient, boolean isSuccessful, int originVertexID, int originOutgoingEdgeID) {
        this(senderId, recipient, isSuccessful, originVertexID, originOutgoingEdgeID, null);
    }

    /**
     * @param certificate the certificate this status is reporting on, handed back to the
     *                    parent unchanged. Carrying it here is what lets a robot avoid
     *                    keeping a copy of its own: whoever made the offer gets the
     *                    certificate back and can re-offer with it directly.
     */
    public StatusMessage(int senderId, int recipient, boolean isSuccessful, int originVertexID, int originOutgoingEdgeID, VoltageCertificate certificate) {
        super(senderId, recipient);
        this.isSuccessful = isSuccessful;
        this.originVertexID = originVertexID;
        this.originOutgoingEdgeID = originOutgoingEdgeID;
        this.certificate = certificate;
    }

    /** The certificate that travelled with the walk this status reports on. */
    public VoltageCertificate getCertificate() {
        return certificate;
    }

    public boolean isSuccessful() {
        return isSuccessful;
    }

    public int getOriginVertexID() {
        return originVertexID;
    }

    public int getOriginOutgoingEdgeID() {
        return originOutgoingEdgeID;
    }

    /**{@inheritDoc}*/
    public String getMessageType() {
        return "Status";
    }

    public int getPriority() {
        return 1;
    }

    @Override
    public String toString() {
        return super.toString() + "\n"
            + "Status: " + (isSuccessful ? "Success" : "Failure") + "\n"
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
        if(!(o instanceof StatusMessage)) {
            return false;
        }

        StatusMessage other = (StatusMessage) o;

        return this.isSuccessful() == other.isSuccessful()
            && originVertexID == other.getOriginVertexID()
            && originOutgoingEdgeID == other.getOriginOutgoingEdgeID();
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(super.hashCode(), isSuccessful, originVertexID, originOutgoingEdgeID);
    }
}
