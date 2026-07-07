package org.communicationModels.Messages;

public class RejectAssignmentMessage extends AbstractMessage{
    private int originVertexID;
    private int originOutgoingEdgeID;

    public RejectAssignmentMessage(int senderId, int recipient, int originVertexID, int originOutgoingEdgeID) {
        super(senderId, recipient);
        this.originVertexID = originVertexID;
        this.originOutgoingEdgeID = originOutgoingEdgeID;
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

    public int getPriority() {
        return 2;
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
        if(!(o instanceof RejectAssignmentMessage)) {
            return false;
        }

        RejectAssignmentMessage other = (RejectAssignmentMessage) o;

        return originVertexID == other.getOriginVertexID()
            && originOutgoingEdgeID == other.getOriginOutgoingEdgeID();
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(super.hashCode(), originVertexID, originOutgoingEdgeID);
    }
}
