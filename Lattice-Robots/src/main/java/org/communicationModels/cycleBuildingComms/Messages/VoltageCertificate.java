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
 * <p><strong>Closure is decided by identity and length, not by geometry.</strong> A
 * certificate closes a face when it comes back to the robot that minted it, having taken
 * exactly {@code cycleLength} hops to do so. Those are the two discrete facts carried
 * here, and they are sufficient.
 *
 * <p><strong>Why the accumulated transform decides nothing.</strong> Each relayer composes
 * the hop into itself, {@code T(parent -> me) = P_{i-1}^-1 . P_i}, so a walk composes to
 * <pre>  P_0^-1.P_1 . P_1^-1.P_2 . ... . P_{k-1}^-1.P_k  =  P_0^-1 . P_k</pre>
 * -- every intermediate pose cancels -- and the initiator's own closing hop
 * {@code P_k^-1 . P_0} makes the product <em>exactly</em> the identity. That holds for any
 * poses whatsoever. It was measured, not assumed: displacing a robot 150 units off its
 * lattice site leaves the closed product at 0.000000000000, and so does a walk over four
 * arbitrary poses that form no face at all.
 *
 * <p>Accumulating the per-hop residual against the ideal voltage does not rescue it. The
 * measured walk is a closed loop, so its displacements sum to zero; the ideal walk closes
 * by construction, so its voltages sum to zero; and while every shipped lattice declares
 * its roles at orientation 0, every voltage is a pure translation and the difference of
 * two zeros is zero. Nothing accumulated around a walk that physically returns to its
 * starting robot can carry information -- the robot is where it is.
 *
 * <p><strong>So the transform is kept as a consistency invariant, not a tolerance test.</strong>
 * At closure it should be exactly the identity. A non-identity product means the walk did
 * not come back to the same physical robot, or that a robot moved mid-walk -- both
 * genuinely anomalous, and both worth surfacing in the tick log. It is deliberately
 * <em>not</em> gated on an epsilon: there is no drift budget to spend, so there is no
 * tolerance to name, and the two {@code MathUtils} constants that once existed for this
 * were removed rather than left to imply otherwise.
 *
 * <p>Hop arithmetic: the initiator sends {@code hops = 0}; each relayer adds one; so a
 * certificate returning to its initiator carries {@code cycleLength - 1}, and the
 * initiator supplies the closing hop itself. Pinned by
 * {@code VoltageCertificateFlowTest} on both a 3-cycle and a 4-cycle.
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
     *                   of the parent that relayed to it -- never {@code T(me -> child)},
     *                   which would sample a pose the child has not reached yet. The
     *                   product of these telescopes, so at closure it is the identity by
     *                   construction; see the class javadoc for why that is an invariant to
     *                   check rather than a geometric test to tune.
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
     * The accumulated measured transform, composed left-to-right in walk order.
     *
     * <p>Once the initiator has added its closing hop this is the identity, exactly, for a
     * walk that returned to the robot that minted it -- not approximately, and not within
     * any tolerance. Read it as a consistency check: anything other than zero displacement
     * and zero rotation means the walk ended somewhere other than where it began, or a
     * robot moved while the certificate was in flight. Do not read it as a measure of
     * accumulated placement error; it cannot be one. See the class javadoc.
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