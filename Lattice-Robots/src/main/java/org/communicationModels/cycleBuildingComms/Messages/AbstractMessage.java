package org.communicationModels.cycleBuildingComms.Messages;

/**
 * An abstract message template for messages that will be used within the lattice formation problem.
 */
public abstract class AbstractMessage implements Message, Comparable<AbstractMessage> {
    /**
     * Recipient value marking a message as a broadcast: emitted once and heard by every
     * robot in range, rather than addressed to a single robot. No robot ever carries this
     * as an id, so it can never collide with a real recipient.
     */
    public static final int BROADCAST = -1;

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
