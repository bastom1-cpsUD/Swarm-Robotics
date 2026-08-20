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

    /** {@link #getStandAsideId()} value meaning "not asking anyone to move". */
    public static final int NO_REQUEST = -1;

    private final OrientedPoint claimInSenderFrame;
    private final int targetRoleID;
    private final int standAsideId;

    public TargetClaimMessage(int senderId, OrientedPoint claimInSenderFrame, int targetRoleID) {
        this(senderId, claimInSenderFrame, targetRoleID, NO_REQUEST);
    }

    public TargetClaimMessage(int senderId, OrientedPoint claimInSenderFrame, int targetRoleID,
                              int standAsideId) {
        super(senderId, BROADCAST);
        this.claimInSenderFrame = claimInSenderFrame;
        this.targetRoleID = targetRoleID;
        this.standAsideId = standAsideId;
    }

    /**
     * The neighbour this sender is asking to move out of its path, or {@link #NO_REQUEST}.
     *
     * <p>The directive rides the claim rather than travelling as a message of its own
     * because the claim already carries everything one needs: who is asking
     * ({@code senderId}), and where they are trying to get to
     * ({@link #getClaimInSenderFrame()}, which is exactly the far end of the corridor to
     * be cleared). The addressee was the only missing piece, so it is the only thing
     * added. The marginal cost is zero — this beacon was going out regardless.
     *
     * <p>Reusing the claim also gets the lifetime right for free. Stop being blocked and
     * you stop setting the field; the next beacon overwrites it, latest-wins. Go out of
     * range or stop emitting and the whole entry expires on the claim TTL. A directive
     * therefore cannot outlive the intent behind it.
     *
     * <p>One consequence worth knowing: only a {@code cycleBuilder} broadcasts claims at
     * all, so only a cycleBuilder can ask. That is the right emitter set rather than a
     * limitation — it is the one role that both has somewhere to be and can be blocked
     * getting there.
     *
     * @return the id of the robot being asked to stand aside, or {@link #NO_REQUEST}
     */
    public int getStandAsideId() {
        return standAsideId;
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

    public int getTargetRoleID() {
        return targetRoleID;
    }

    @Override
    public String toString() {
        return super.toString() + "\n"
            + "Claimed Pose (sender frame): " + claimInSenderFrame
            + "\nClaimed Target Role ID: " + targetRoleID
            + (standAsideId == NO_REQUEST ? "" : "\nStand aside: robot " + standAsideId);
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

        return java.util.Objects.equals(claimInSenderFrame, other.getClaimInSenderFrame()) &&
                targetRoleID == other.getTargetRoleID() &&
                standAsideId == other.getStandAsideId();
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(super.hashCode(), claimInSenderFrame, targetRoleID, standAsideId);
    }
}
