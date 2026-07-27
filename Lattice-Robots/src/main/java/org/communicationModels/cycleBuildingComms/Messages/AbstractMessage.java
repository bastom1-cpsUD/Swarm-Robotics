package org.communicationModels.cycleBuildingComms.Messages;

/**
 * An abstract message template for messages that will be used within the lattice formation problem.
 */
public abstract class AbstractMessage implements Message, Comparable<AbstractMessage> {
    /**
     * The sender ID of a message
     */
    protected int senderId;
    /**
     * The recipient ID of a message
     */
    protected int recipient;

    public AbstractMessage(int senderId, int recipient) {
        this.senderId = senderId;
        this.recipient = recipient;
    }

    /**{@inheritDoc}*/
    public int getSenderId() {
        return senderId;
    }

    /**{@inheritDoc} */
    public int getRecipient() {
        return recipient;
    }

    /**{@inheritDoc} */
    abstract public String getMessageType();

    abstract public int getPriority();

    @Override
    public String toString() {
        return this.messageInfo();
    }

    @Override 
    public boolean equals(Object o) {
        if(o == this) {
            return true;
        } else if (!(o instanceof AbstractMessage)) {
            return false;
        }
        AbstractMessage other = (AbstractMessage) o;

        return this.getSenderId() == other.getSenderId() 
            && this.getRecipient() == other.getRecipient()
            && this.getMessageType().equals(other.getMessageType());
    }

    @Override 
    public int hashCode() {
        return java.util.Objects.hash(senderId, recipient, this.getMessageType());
    }

    @Override
    public int compareTo(AbstractMessage msg) {
        return Integer.compare(this.getPriority(), msg.getPriority());
    }


}
