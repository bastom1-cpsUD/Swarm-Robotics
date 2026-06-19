package org.communicationModels.Messages;

import org.graphs.LatticeEdge;

    /**

    * Sent through an existing cycle to determine whether the cycle is complete.
    *
    * currentEdge represents the edge that should be occupied by the recipient.
    * cycleOrigin identifies which root edge's cycle is being verified.
    */
    public class VerificationMessage extends AbstractMessage {

    /**

    * The root edge whose cycle is being checked.
        */
        private final LatticeEdge cycleOrigin;

    /**

    * The edge expected to be occupied by the recipient.
        */
        private final LatticeEdge currentEdge;

        private ChainMemberList chainMemberList;

    public VerificationMessage(int senderId, int recipient, LatticeEdge cycleOrigin, LatticeEdge currentEdge, ChainMemberList chainMemberList) {
        super(senderId, recipient);

        this.cycleOrigin = cycleOrigin;
        this.currentEdge = currentEdge;
        this.chainMemberList = chainMemberList;
    }

    public ChainMemberList getChainList() {
        return chainMemberList;
    }

    public LatticeEdge getCycleOrigin() {
           return cycleOrigin;
    }

    public LatticeEdge getCurrentEdge() {
        return currentEdge;
    }

    @Override
    public String getMessageType() {
        return "Verification";
    }

    private String getMessageInfo() {
        return super.messageInfo() + "\n"
        + "Root ID: " + this.chainMemberList.getRootID() + "\n" 
        + "Current Lattice Edge: " + this.currentEdge + "\n"
        + "Beginning Edge of Cycle: " + this.cycleOrigin;
    }

    @Override
    public String toString() {
        return getMessageInfo();
    }
}
