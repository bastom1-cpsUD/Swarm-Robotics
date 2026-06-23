package org.communicationModels.Messages;

import java.util.ArrayList;

import org.graphs.LatticeEdge;

public class PositioningMessage extends AbstractMessage {
    /**
     * The lattice edge that this positioning messages assigns to the recipient
     */
    private LatticeEdge currentEdge;
    /**
     * The ID of the root whose cycle is being built
     */
    private int rootId;
    
    private ChainMemberList chainList;

    private LatticeEdge originEdge;
    /**
     * Constructor for a positioning message
     * @param senderId the ID of the sender
     * @param recipient the ID of the recipient
     * @param currentEdge the edge being assigned to the recipient
     * @param rootId the ID of the root whose cycle is being built
     */
    public PositioningMessage(int senderId, int recipient, LatticeEdge currentEdge, LatticeEdge originEdge, ChainMemberList chainList) {
        super(senderId, recipient);
        this.currentEdge = currentEdge;
        this.originEdge = originEdge;
        this.chainList = chainList;
    }

    /**{@inheritDoc} */
    public String getMessageType() {
        return "Assignment";
    }

    /**
     * Retrieves the root ID for the cycle
     * @return the root ID
     */
    public int getRootId() {
        return rootId;
    }

    /**
     * Retrieves the list of robots in the chain
     * @return
     */
    public ChainMemberList getChainList() {
        return chainList;
    }

    /**
     * Retrieves the lattice edge assigned to the recipient
     * @return the lattice edge assigned
     */
    public LatticeEdge getCurrentEdge() {
        return currentEdge;
    }

    public LatticeEdge getOriginEdge() {
        return originEdge;
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
        + "Lattice Edge: " + this.currentEdge + "\n"
        + "Beginning Edge of Cycle: " + this.originEdge;
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

        return this.getRootId() == other.getRootId()
            && this.getCurrentEdge().equals(other.getCurrentEdge())
            && this.originEdge.equals(other.getOriginEdge())
            && this.chainList.equals(other.chainList);
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(super.hashCode(), currentEdge, rootId, originEdge, chainList);
    }
}
