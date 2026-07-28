package org.communicationModels.cycleBuildingComms.Messages;

public class StatusMessage extends AbstractMessage {
    private boolean isSuccessful;
    private int originVertexID;
    private int originOutgoingEdgeID;

    public StatusMessage(int senderId, int recipient, boolean isSuccessful, int originVertexID, int originOutgoingEdgeID) {
        super(senderId, recipient);
        this.isSuccessful = isSuccessful;
        this.originVertexID = originVertexID;
        this.originOutgoingEdgeID = originOutgoingEdgeID;
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
