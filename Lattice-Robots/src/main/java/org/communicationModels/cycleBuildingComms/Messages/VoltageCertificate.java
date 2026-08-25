package org.communicationModels.cycleBuildingComms.Messages;

import org.graphs.util.RigidBodyTransformation;

/**
 * The evidence a walk carries with it, so that closure can be decided by whether the walk
 * came back rather than by who it happened to reach.
 *
 * <p>Three scalars travel, and all three are immutable in transit: the <em>initiator</em>
 * that minted the certificate and is the only robot permitted to evaluate it, the number
 * of hops taken so far, and the accumulated <em>measured</em> transform.
 *
 * <p><strong>No chain member list.</strong> The walk's topology now lives in the
 * communication tuples each robot holds for its own incident edges, so carrying a roster
 * of ids in every message would duplicate it -- at O(walk length) per message, growing
 * with the face. What the certificate carries is fixed-size regardless of how far it has
 * travelled.
 *
 * <p><strong>Measured, not ideal.</strong> Composing the graph's own voltages would prove
 * nothing: {@code VoltageGraphBuilder.build()} already throws unless every face's ideal
 * holonomy converges to identity, so that product is identity by construction. The content
 * lives in realized geometry -- each hop is the transform from the parent's <em>live</em>
 * pose to this robot's live pose, so accumulated placement error is what the initiator
 * ends up testing.
 *
 * <p><strong>Each robot composes the hop INTO itself</strong> -- {@code T(parent -> me)}
 * -- at the moment it acts on the obligation, never {@code T(me -> child)}. The child has
 * not moved yet when it is selected, so measuring outbound samples a pose that does not
 * exist; and on a symmetric lattice that mistake still lands near identity often enough to
 * pass square and hexagon runs unnoticed. Composing inbound means both endpoints are
 * settled: the parent relayed earlier, so it was in position then, and this robot is in
 * position now.
 *
 * <p>Arithmetic follows from that rule. The initiator sends {@code hops = 0} and an
 * identity product; each relayer adds one hop and one inbound transform; so a certificate
 * returning to its initiator carries {@code cycleLength - 1} hops, and the initiator
 * supplies the closing hop itself before testing.
 *
 * <p>A certificate is never stored in a field of its own. It lives in messages, rides back
 * unchanged on rejections and statuses, and stays queued in the inbox while a robot is in
 * transit.
 */
public class VoltageCertificate {


    private final int initiatorId;
    private final int hops;
    private final RigidBodyTransformation measuredVoltage;

    /**
     * A fresh certificate minted by an initiator. Hop count and accumulated transform both
     * start empty; the first relayer contributes the first measured hop.
     *
     * @param initiatorId the robot minting this certificate, and the only one that may
     *                    evaluate it when it returns
     */
    public VoltageCertificate(int initiatorId) {
        this(initiatorId, 0, RigidBodyTransformation.identity());
    }

    private VoltageCertificate(int initiatorId, int hops, RigidBodyTransformation measured) {
        this.initiatorId = initiatorId;
        this.hops = hops;
        this.measuredVoltage = measured;
    }

    /**
     * Extends this certificate by one measured hop, returning a new instance. The original
     * is left untouched, which is what lets the same certificate be re-offered to a
     * different candidate after a rejection with no extension to undo.
     *
     * @param inboundHop {@code T(parent -> me)}, measured from this robot's own observation
     *                   of the parent that relayed to it. Never {@code T(me -> child)};
     *                   see the class javadoc.
     * @return a new certificate one hop longer
     */
    public VoltageCertificate extend(RigidBodyTransformation inboundHop) {
        // Left-to-right, matching the order VoltageGraph.validateCycle composes a walk.
        return new VoltageCertificate(initiatorId, hops + 1,
                measuredVoltage.compose(inboundHop));
    }

    /**
     * The robot that minted this certificate, and the only one that may decide whether it
     * closes.
     */
    public int getInitiatorID() {
        return initiatorId;
    }

    /**
     * The number of measured hops accumulated so far. A certificate arriving back at its
     * initiator carries one fewer than the face's cycle length, because the closing hop is
     * the initiator's own to contribute.
     */
    public int getHops() {
        return hops;
    }

    /**
     * The accumulated measured transform, composed left-to-right in walk order. Only
     * meaningful as a closure test once the initiator has added the closing hop.
     */
    public RigidBodyTransformation getMeasuredVoltage() {
        return measuredVoltage;
    }

    @Override
    public String toString() {
        return "VoltageCertificate[initiator: " + initiatorId
                + " hops: " + hops
                + " measured: " + measuredVoltage.asPose() + "]";
    }

    /**
     * Identity and length only. The measured transform is deliberately excluded: it is a
     * float-valued measurement, so two certificates for the same walk would almost never
     * compare equal on it, and every call site -- message equality in tests, walk
     * comparisons in {@code CyclebuilderComms} -- is asking about the walk, not the
     * geometry.
     */
    @Override
    public boolean equals(Object o) {
        if(this == o) {
            return true;
        } else if(!(o instanceof VoltageCertificate)) {
            return false;
        }

        VoltageCertificate other = (VoltageCertificate) o;
        return this.initiatorId == other.initiatorId && this.hops == other.hops;
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(initiatorId, hops);
    }
}