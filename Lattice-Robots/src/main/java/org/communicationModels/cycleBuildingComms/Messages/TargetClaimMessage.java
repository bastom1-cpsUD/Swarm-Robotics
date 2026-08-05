package org.communicationModels.cycleBuildingComms.Messages;

import org.graphs.util.OrientedPoint;

/**
 * A robot's declaration of where it intends to go, broadcast to every neighbour in
 * range.
 *
 * <p>This exists because <em>intent is not observable</em>. A robot can sense a
 * neighbour's pose -- a camera, lidar, or UWB tag gives relative bearing and range with
 * no cooperation from the target -- but no sensor reveals where that neighbour is
 * <em>headed</em>. Without a declaration the only alternative is to infer motion from
 * successive observations and extrapolate, which is both far more fragile and, worse,
 * asymmetric: two robots inferring each other's trajectories test them against
 * <em>different</em> targets, so they can both yield or neither can. A declaration is
 * symmetric -- both compare the same pair of points -- which is what lets an id
 * tie-break pick exactly one winner.
 *
 * <p>The claim is carried in the <em>sender's own frame</em>. That is the only form
 * transmittable in a decentralized swarm: a global coordinate would presume a shared
 * origin no robot has. A receiver recovers it in its own frame by composing with its
 * observation of the sender -- see
 * {@code CyclebuilderComms.detectAssignmentContention()}.
 *
 * <p>Unlike the protocol messages this class sits beside, a claim is <em>soft state</em>:
 * broadcast every tick, latest-wins, and expiring on a TTL. It never enters
 * {@code incomingMessages} and is never popped by {@code processMessages}, so
 * {@link #getPriority()} is not meaningful for it.
 */
public class TargetClaimMessage extends AbstractMessage {

    private final OrientedPoint claimInSenderFrame;

    public TargetClaimMessage(int senderId, OrientedPoint claimInSenderFrame) {
        super(senderId, BROADCAST);
        this.claimInSenderFrame = claimInSenderFrame;
    }

    /**
     * The pose the sender intends to occupy, expressed in the sender's own local frame.
     * @return the claimed pose, relative to the sender
     */
    public OrientedPoint getClaimInSenderFrame() {
        return claimInSenderFrame;
    }

    /**{@inheritDoc}*/
    public String getMessageType() {
        return "TargetClaim";
    }

    /**
     * Unused. Claims are held in their own latest-wins inbox rather than the priority
     * queue, so nothing ever orders them.
     */
    public int getPriority() {
        return Integer.MAX_VALUE;
    }

    @Override
    public String toString() {
        return super.toString() + "\n"
            + "Claimed Pose (sender frame): " + claimInSenderFrame;
    }

    @Override
    public boolean equals(Object o) {
        if(this == o) {
            return true;
        }
        if(!super.equals(o)) {
            return false;
        }
        if(!(o instanceof TargetClaimMessage)) {
            return false;
        }

        TargetClaimMessage other = (TargetClaimMessage) o;

        return java.util.Objects.equals(claimInSenderFrame, other.getClaimInSenderFrame());
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(super.hashCode(), claimInSenderFrame);
    }
}
