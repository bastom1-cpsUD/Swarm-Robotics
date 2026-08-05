package org.utils.logging;

import org.communicationModels.cycleBuildingComms.Messages.AbstractMessage;

/**
 * A single outgoing message captured for logging purposes: who it went to,
 * what kind it was, and a short human-readable summary.
 *
 * <p>Produced by {@code CyclebuilderComms.send(...)} every time a message is
 * enqueued to another robot, and surfaced per-tick via
 * {@code CyclebuilderComms.sentThisTick()}.</p>
 */
public record OutgoingMessageRecord(int recipientId, String messageType, String summary) {

    /**
     * Renders {@link AbstractMessage#BROADCAST} as a word rather than as {@code -1},
     * which reads as a missing recipient rather than an intentional one. A broadcast is
     * recorded once no matter how many neighbours heard it, since it is one transmission.
     */
    @Override
    public String toString() {
        String target = recipientId == AbstractMessage.BROADCAST
                ? "broadcast"
                : "robot " + recipientId;
        return messageType + " -> " + target;
    }
}
