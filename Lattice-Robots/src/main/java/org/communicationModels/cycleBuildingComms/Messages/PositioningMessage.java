package org.communicationModels.cycleBuildingComms.Messages;

public class PositioningMessage extends AbstractMessage {
    private ChainMemberList chainList;

    private int assignedVertexID;
    private int assignedOutgoingEdgeID;

    private int originVertexID;
    private int originOutgoingEdgeID;



    /**
     * Constructor for a positioning message
     * @param senderId the ID of the sender
     * @param recipient the ID of the recipient
     * @param currentEdge the edge being assigned to the recipient
     * @param rootId the ID of the root whose cycle is being built
     */
    public PositioningMessage(int senderId, int recipient, int assignedVertexID, int assignedOutgoingEdgeID, int originVertexID, int originOutgoingEdgeID, ChainMemberList chainList) {
        super(senderId, recipient);
        this.chainList = chainList;

        this.assignedVertexID = assignedVertexID;
        this.assignedOutgoingEdgeID = assignedOutgoingEdgeID;

        this.originVertexID = originVertexID;
        this.originOutgoingEdgeID = originOutgoingEdgeID;

    }

    /**{@inheritDoc} */
    public String getMessageType() {
        return "Assignment";
    }

    /**
     * Retrieves the list of robots in the chain
     * @return
     */
    public ChainMemberList getChainList() {
        return chainList;
    }

    public int getAssignedVertexID() {
        return assignedVertexID;
    }

    public int getAssignedOutgoingEdgeID() {
        return assignedOutgoingEdgeID;
    }

    public int getOriginVertexID() {
        return originVertexID;
    }

    public int getOriginOutgoingEdgeID() {
        return originOutgoingEdgeID;
    }

    public int getPriority() {
        return 3;
    }

    /**
     * Provides details of the message
     * @return a string with message details
     */
    private String getMessageInfo() {
        return super.messageInfo() + "\n"
        + "Root ID: " + this.chainList.getRootID() + "\n" 
        + "Assigned Vertex ID: " + this.assignedVertexID + "\n"
        + "Assigned Edge ID: " + this.assignedOutgoingEdgeID + "\n"        
        + "Beginning Edge of Cycle Vertex ID: " + this.originVertexID + "\n"
        + "Beginning Edge of Cycle Edge ID: " + this.originOutgoingEdgeID;
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
        if(!(o instanceof PositioningMessage)) {
            return false;
        }
        PositioningMessage other = (PositioningMessage) o;

        return assignedVertexID == other.getAssignedVertexID()
            && assignedOutgoingEdgeID == other.getAssignedOutgoingEdgeID()
            && originVertexID == other.getOriginVertexID()
            && originOutgoingEdgeID == other.getOriginOutgoingEdgeID()
            && this.chainList.equals(other.chainList);
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(super.hashCode(), assignedVertexID, assignedOutgoingEdgeID, originVertexID, originOutgoingEdgeID, chainList);
    }
}
