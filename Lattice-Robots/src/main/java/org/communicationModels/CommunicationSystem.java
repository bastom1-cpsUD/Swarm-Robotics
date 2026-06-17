package org.communicationModels;

import java.util.LinkedList;
import java.util.Queue;

import org.communicationModels.Messages.AbstractMessage;

public abstract class CommunicationSystem {
    protected Queue<AbstractMessage> incomingMessages;

    public CommunicationSystem() {
        this(new LinkedList<>());
    }
    public CommunicationSystem(Queue<AbstractMessage> incomingMessages) {
        this.incomingMessages = incomingMessages;
    }
    /**
     * Enqueues a message onto the communication system
     * @param msg
     */
    public void enqueueMessage(AbstractMessage msg) {
        incomingMessages.add(msg);
    }

    /**
     * Processes the message queue within the communication system, updating internal state and generating new actions as necessary.
     */
    public abstract void processMessages();

}
