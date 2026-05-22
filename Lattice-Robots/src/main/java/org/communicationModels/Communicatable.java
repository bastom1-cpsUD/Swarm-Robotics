package org.communicationModels;

public interface Communicatable {
    
    void enqueueMessage(Message msg);
    void processMessages();
}
