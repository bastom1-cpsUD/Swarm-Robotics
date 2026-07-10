package org.utils.logging;

/**
 * A single outgoing message captured for logging purposes: who it went to,
 * what kind it was, and a short human-readable summary.
 *
 * <p>Produced by {@code CyclebuilderComms.send(...)} every time a message is
 * enqueued to another robot, and surfaced per-tick via
 * {@code CyclebuilderComms.sentThisTick()}.</p>
 */
public record OutgoingMessageRecord(int recipientId, String messageType, String summary) {
}
