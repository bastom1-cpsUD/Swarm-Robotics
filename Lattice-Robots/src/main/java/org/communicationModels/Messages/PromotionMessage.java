package org.communicationModels.Messages;

public class PromotionMessage extends AbstractMessage {
    
    public PromotionMessage(int senderId, int recipient) {
        super(senderId, recipient);

    }

    /**{@inheritDoc}*/
    public String getMessageType() {
        return "Promotion";
    }

    public int getPriority() {
        return 4;
    }
}