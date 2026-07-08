package org.communicationModels.Messages;

public class RejectAssignmentMessage extends AbstractMessage{
    private int originVertexID;
    private int originOutgoingEdgeID;
    private boolean isRetryable;

    public RejectAssignmentMessage(int senderId, int recipient, int originVertexID, int originOutgoingEdgeID, boolean isRetryable) {
        super(senderId, recipient);
        this.originVertexID = originVertexID;
        this.originOutgoingEdgeID = originOutgoingEdgeID;
        this.isRetryable = isRetryable;
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
        return 3;
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
