package org.communicationModels.SongOkaneComms;

import java.util.LinkedList;
import java.util.Queue;

/**
 * An abstract class outline the commonalities and essential functions of any communication system
 */
public abstract class CommunicationSystem {
    protected Queue<Message> incomingMessages;
    protected Queue<Action> actionQueue;


    public CommunicationSystem() {
        this(new LinkedList<>(), new LinkedList<>());
    }
    public CommunicationSystem(Queue<Message> incomingMessages, Queue<Action> actionQueue) {
        this.incomingMessages = incomingMessages;
        this.actionQueue = actionQueue;
    }
    /**
     * Enqueues a message onto the communication system
     * @param msg
     */
    public void enqueueMessage(Message msg) {
        incomingMessages.add(msg);
    }

    /**
     * Returns the next action to be executed by the robot, which is removed from the action queue.
     * @return next action for execution
     */
    public Action getNextAction() {
        return actionQueue.remove();
    }

    /**
     * Processes the message queue within the communication system, updating internal state and generating new actions as necessary.
     */
    public abstract void processMessages();
}
