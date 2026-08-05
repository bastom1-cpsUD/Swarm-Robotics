package org.communicationModels.cycleBuildingComms;

import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.ConcurrentHashMap;

import org.communicationModels.cycleBuildingComms.Messages.AbstractMessage;
import org.communicationModels.cycleBuildingComms.Messages.TargetClaimMessage;

/**
 * Base class for a robot's radio, carrying two independent inboxes for two genuinely
 * different classes of traffic.
 *
 * <p>{@link #incomingMessages} holds <strong>protocol unicast</strong> --
 * {@code PositioningMessage}, {@code StatusMessage}, {@code RejectAssignmentMessage} and
 * friends. Each is addressed to one robot, must be handled exactly once and in order, and
 * {@code CyclebuilderComms.processMessages} deliberately pops at most one per tick.
 *
 * <p>{@link #incomingClaims} holds <strong>broadcast beacons</strong>: a neighbour's
 * declaration of where it intends to go. These are soft state -- emitted every tick,
 * latest-wins per sender, unordered, and expiring on a TTL, so losing one costs nothing
 * because another follows.
 *
 * <p>They are kept apart deliberately. Feeding claims through the protocol queue would
 * let them starve real protocol messages, and would let the queue's own
 * {@code pendingChildID} gate delay a claim arbitrarily -- exactly the wrong latency for
 * a check whose whole purpose is to fire early. Real ad-hoc systems make the same split
 * (OLSR/BATMAN HELLO packets, or the 10 Hz SAE J2735 safety broadcasts that let vehicles
 * announce position and intent alongside ordinary addressed traffic).
 */
public abstract class CommunicationSystem {
    protected Queue<AbstractMessage> incomingMessages;

    /**
     * Latest claim heard from each neighbour, keyed by sender id.
     *
     * <p>Concurrent by necessity, not by caution: a neighbour deposits its beacon from
     * that neighbour's own executor thread while this robot reads the map on its own —
     * the same reason {@code CyclebuilderComms} uses a {@code ConcurrentLinkedQueue} for
     * {@link #incomingMessages}.
     */
    protected final Map<Integer, ClaimEntry> incomingClaims = new ConcurrentHashMap<>();

    /**
     * A received claim and how many of the receiver's own phases have elapsed since it
     * arrived.
     *
     * <p>An age rather than a timestamp, deliberately: no clock is involved on either
     * side. Robots activate independently and share no time reference, so the only thing
     * a receiver can meaningfully count is its own phases.
     */
    public record ClaimEntry(TargetClaimMessage claim, int age) {
        ClaimEntry aged() {
            return new ClaimEntry(claim, age + 1);
        }
    }

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
     * Receives a broadcast claim from a neighbour, replacing any previous claim from that
     * same sender. Never queued and never blocking -- a claim is only ever read as the
     * neighbour's latest known intent.
     *
     * @param claim the neighbour's declared target, in the neighbour's own frame
     */
    public void receiveClaim(TargetClaimMessage claim) {
        incomingClaims.put(claim.getSenderId(), new ClaimEntry(claim, 0));
    }

    /**
     * Ages every held claim by one phase and drops any that have reached {@code ttl}.
     * Call once at the start of each phase.
     *
     * <p>Expiry does double duty. It tolerates a dropped beacon, and -- more importantly
     * -- it retires the claim of a robot that has become unassigned and simply stopped
     * emitting. Without it that stale claim would keep provoking a phantom conflict over
     * a spot nobody is heading for any more.
     *
     * <p>Ages by replacement rather than mutation so {@link ClaimEntry} stays immutable:
     * neighbours deposit claims into this map from their own executor threads, and
     * {@code replaceAll} on a {@code ConcurrentHashMap} is atomic per entry.
     *
     * @param ttl how many phases a claim stays live after arriving
     */
    protected void ageClaims(int ttl) {
        incomingClaims.replaceAll((senderId, entry) -> entry.aged());
        incomingClaims.values().removeIf(entry -> entry.age() >= ttl);
    }

    /**
     * Processes the message queue within the communication system, updating internal state and generating new actions as necessary.
     */
    public abstract void processMessages();

}
