    package org.communicationModels.Messages;

    import org.graphs.LatticeEdge;

    /**

    * Returned to the root after a verification attempt.
    */
    public class VerificationResponseMessage extends AbstractMessage {

    /**

    * Root that initiated the verification.
        */
        private final int rootID;

    /**

    * Which cycle was checked.
        */
        private final LatticeEdge cycleOrigin;

    /**

    * Result of the verification.
        */
        private final boolean successful;

    public VerificationResponseMessage(int senderId, int recipient, int rootID, LatticeEdge cycleOrigin, boolean successful) {
        super(senderId, recipient);

        this.rootID = rootID;
        this.cycleOrigin = cycleOrigin;
        this.successful = successful;
    }

    public int getRootID() {
        return rootID;
    }

    public LatticeEdge getCycleOrigin() {
        return cycleOrigin;
    }

    public boolean isSuccessful() {
        return successful;
    }

    @Override
    public String getMessageType() {
        return "VerificationResponse";
    }

    public int getPriority() {
        return 2;
    }

    private String getMessageInfo() {
        return super.messageInfo() + "\n"
            + "Root ID: " + rootID + "\n"
            + "Beginning Edge of the Cycle: " + cycleOrigin + "\n"
            + "Verified: " + successful;
    }

    @Override
    public String toString() {
        return getMessageInfo();
    }
    
}
