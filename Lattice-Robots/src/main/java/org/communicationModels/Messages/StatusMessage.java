package org.communicationModels.Messages;

import org.graphs.LatticeEdge;

public class StatusMessage extends AbstractMessage {
    private boolean isSuccessful;
    private LatticeEdge originEdge;

    public StatusMessage(int senderId, int recipient, boolean isSuccessful, LatticeEdge originEdge) {
        super(senderId, recipient);
        this.isSuccessful = isSuccessful;
        this.originEdge = originEdge;
    }

    public boolean isSuccessful() {
        return isSuccessful;
    }

    public LatticeEdge getCycleOrigin() {
        return originEdge;
    }
    /**{@inheritDoc}*/
    public String getMessageType() {
        return "Status";
    }

    @Override
    public String toString() {
        return super.toString() + "\n"
            + "Status: " + (isSuccessful ? "Success" : "Failure") + "\n"
            + "Beginning Edge of Cycle: " + originEdge;
    
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
            && originEdge.equals(other.originEdge);
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(super.hashCode(), isSuccessful, originEdge);
    }
}
