package org.communicationModels.Messages;

import org.graphs.LatticeEdge;

public class PromotionMessage extends AbstractMessage {
    private int assignedVertexID;
    private int assignedOutgoingEdgeID;

    public PromotionMessage(int senderId, int recipient, int assignedVertexID, int assignedOutgoingEdgeID) {
        super(senderId, recipient);
        this.assignedVertexID = assignedVertexID;
        this.assignedOutgoingEdgeID = assignedOutgoingEdgeID;
    }

    /**{@inheritDoc}*/
    public String getMessageType() {
        return "Promotion";
    }

    public int getPriority() {
        return 4;
    }

    public int getAssignedVertexID() {
        return assignedVertexID;
    }

    public int getAssignedOutgoingEdgeID() {
        return assignedOutgoingEdgeID;
    }

    /**
     * Provides details of the message
     * @return a string with message details
     */
    private String getMessageInfo() {
        return super.messageInfo() + "\n"
        + "Assigned Vertex ID: " + this.assignedVertexID + "\n"
        + "Assigned Edge ID: " + this.assignedOutgoingEdgeID;
    }
    
    @Override
    public String toString() {
        return getMessageInfo();
    }

    @Override
    public boolean equals(Object o) {
        if(this == o) {
            return true;
        }
        if(!super.equals(o)) {
            return false;
        }
        if(!(o instanceof PromotionMessage)) {
            return false;
        }
        PromotionMessage other = (PromotionMessage) o;

        return this.assignedVertexID == other.getAssignedVertexID()
                && this.assignedOutgoingEdgeID == other.getAssignedOutgoingEdgeID();
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(super.hashCode(), assignedVertexID, assignedOutgoingEdgeID);
    }

}