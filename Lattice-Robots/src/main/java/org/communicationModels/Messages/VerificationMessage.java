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

    * Root that initiated verification.
        */
        private final int rootID;

    /**

    * The root edge whose cycle is being checked.
        */
        private final LatticeEdge cycleOrigin;

    /**

    * The edge expected to be occupied by the recipient.
        */
        private final LatticeEdge currentEdge;

    public VerificationMessage(int senderId, int recipient, int rootID, LatticeEdge cycleOrigin, LatticeEdge currentEdge) {
        super(senderId, recipient);

        this.rootID = rootID;
        this.cycleOrigin = cycleOrigin;
        this.currentEdge = currentEdge;
    }

    public int getRootID() {
        return rootID;
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
        + "Root ID: " + this.rootID + "\n" 
        + "Current Lattice Edge: " + this.currentEdge + "\n"
        + "Beginning Edge of Cycle: " + this.cycleOrigin;
    }

    @Override
    public String toString() {
        return getMessageInfo();
    }
}
