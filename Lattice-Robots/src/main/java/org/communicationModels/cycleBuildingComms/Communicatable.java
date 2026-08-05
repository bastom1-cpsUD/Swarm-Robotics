package org.communicationModels.cycleBuildingComms;

import org.communicationModels.cycleBuildingComms.Messages.AbstractMessage;
import org.communicationModels.cycleBuildingComms.Messages.TargetClaimMessage;

public interface Communicatable {

    /**
     * Enqueues a message onto the communication system
     * @param msg the message to be added to the communication system queue
     */
    void enqueueMessage(AbstractMessage msg);

    /**
     * Receives a broadcast claim from a neighbour. Deliberately not routed through
     * {@link #enqueueMessage(AbstractMessage)}: a claim is soft state read as a
     * neighbour's latest known intent, not a protocol event to be processed in order.
     *
     * <p>The implementor stamps the claim with its <em>own</em> activation count on the
     * way in. Senders cannot do it -- robots activate independently and share no clock.
     *
     * @param claim the neighbour's declared target, in the neighbour's own frame
     */
    void receiveClaim(TargetClaimMessage claim);

    /**
     * Processes the message queue within the communication system
     */
    void processMessages();
}
