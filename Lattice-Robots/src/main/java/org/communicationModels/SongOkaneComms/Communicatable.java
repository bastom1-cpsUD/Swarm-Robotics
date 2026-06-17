package org.communicationModels.SongOkaneComms;

public interface Communicatable {
    
    /**
     * Enqueues a message onto the communication system
     * @param msg the message to be added to the communication system queue
     */
    void enqueueMessage(Message msg);
    
    /**
     * Processes the message queue within the communication system
     */
    void processMessages();
}
