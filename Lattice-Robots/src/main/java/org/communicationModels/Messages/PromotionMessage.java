package org.communicationModels.Messages;

import org.graphs.LatticeEdge;

public class PromotionMessage extends AbstractMessage {
    
    private LatticeEdge currentEdge;
    public PromotionMessage(int senderId, int recipient, LatticeEdge currentEdge) {
        super(senderId, recipient);
        this.currentEdge = currentEdge;
    }

    /**{@inheritDoc}*/
    public String getMessageType() {
        return "Promotion";
    }

    public int getPriority() {
        return 4;
    }

    public LatticeEdge getCurrentEdge() {
        return currentEdge;
    }

    /**
     * Provides details of the message
     * @return a string with message details
     */
    private String getMessageInfo() {
        return super.messageInfo() + "\n"
        + "Lattice Edge: " + this.currentEdge;
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

        return this.getCurrentEdge().equals(other.getCurrentEdge());
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(super.hashCode(), currentEdge);
    }

}