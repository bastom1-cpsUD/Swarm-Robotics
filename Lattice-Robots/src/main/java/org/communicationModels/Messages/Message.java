package org.communicationModels.Messages;
/**
 * An interface that defines the basic functionality of a message used within the lattice formation problem.
 */
public interface Message {
    /**
     * Retrieves the sender ID
     * @return the sender ID of the message
     */
    int getSenderId();

    /**
     * Retrieves the recipient
     * @return the recipient of the message
     */
    int getRecipient();

    /**
     * Returns the message's functionality type
     * @return the type of message
     */
    String getMessageType();

    /**
     * Provides details of the message
     * @return a string with message details
     */
    default String messageInfo() {
        return "Message Type: " + getMessageType() + "\n"
                + "Sender ID: " + getSenderId() + "\n"
                + "Recipient ID: " + getRecipient();
    }
}

