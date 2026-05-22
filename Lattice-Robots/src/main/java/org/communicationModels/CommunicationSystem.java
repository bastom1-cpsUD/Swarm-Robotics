package org.communicationModels;

import java.util.ArrayDeque;
import java.util.Queue;

public abstract class CommunicationSystem {
    protected Queue<Message> incomingMessages;
    protected Queue<Action> actionQueue;

    public CommunicationSystem() {
        incomingMessages = new ArrayDeque<>();
        actionQueue = new ArrayDeque<>();
    }

    public void enqueueMessage(Message msg) {
        incomingMessages.add(msg);
    }

    public Action getNextAction() {
        return actionQueue.remove();
    }

    public abstract void processMessages();
}
