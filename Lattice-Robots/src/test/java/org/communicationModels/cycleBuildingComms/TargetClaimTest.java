package org.communicationModels.cycleBuildingComms;

import org.communicationModels.cycleBuildingComms.Messages.AbstractMessage;
import org.communicationModels.cycleBuildingComms.Messages.TargetClaimMessage;
import org.graphs.util.OrientedPoint;
import org.graphs.util.RigidBodyTransformation;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Covers the claim-beacon half of assignment-contention detection: the frame arithmetic
 * that turns a neighbour's declared target into something comparable against this robot's
 * own, and the latest-wins/expiring inbox those declarations land in.
 *
 * <p>The frame arithmetic is the part worth pinning down. The approach this replaced
 * inferred a neighbour's intent by differencing two observations, and its frame change
 * was wrong in a way nothing caught -- it dropped the rotation between the two frames and
 * applied the translation in global axes, so it was only correct when every robot happened
 * to be sitting at heading zero. The tests below therefore always give both robots
 * non-zero and unequal headings, and
 * {@link #claimSurvivesRigidMotionOfTheWholeWorld()} pins the property that actually
 * matters: the result cannot depend on a global origin, because no robot has one.
 */
class TargetClaimTest {

    private static final double TOL = 1e-9;

    /** A concrete CommunicationSystem, since the class under test is abstract. */
    private static final class TestComms extends CommunicationSystem {
        @Override
        public void processMessages() {
            // Not exercised here; claims never touch the protocol queue.
        }

        void age(int ttl) {
            ageClaims(ttl);
        }

        int liveClaimCount() {
            return incomingClaims.size();
        }

        TargetClaimMessage claimFrom(int senderId) {
            ClaimEntry entry = incomingClaims.get(senderId);
            return entry == null ? null : entry.claim();
        }
    }

    /** The pose {@code target} occupies in {@code frame}'s local coordinates. */
    private static OrientedPoint inFrameOf(OrientedPoint frame, OrientedPoint target) {
        return new RigidBodyTransformation(frame).inverse().apply(target);
    }

    private static void assertPoseEquals(OrientedPoint expected, OrientedPoint actual) {
        assertEquals(expected.getX(), actual.getX(), TOL, "x");
        assertEquals(expected.getY(), actual.getY(), TOL, "y");
        assertEquals(0.0, org.utils.MathUtils.angleDifference(
                expected.getOrientation(), actual.getOrientation()), TOL, "orientation");
    }

    // ---------------------------------------------------------------------
    // Frame arithmetic
    // ---------------------------------------------------------------------

    @Test
    @DisplayName("A neighbour's claim reconstructs to the target's pose in the observer's frame")
    void claimReconstructsTargetInObserverFrame() {
        OrientedPoint observer = new OrientedPoint(10, -4, 0.7);
        OrientedPoint sender = new OrientedPoint(-22, 31, -2.1);
        OrientedPoint target = new OrientedPoint(55, 8, 1.3);

        // What the sender broadcasts, and what the observer sees of the sender.
        OrientedPoint claim = inFrameOf(sender, target);
        OrientedPoint senderSeen = inFrameOf(observer, sender);

        assertPoseEquals(inFrameOf(observer, target),
                CyclebuilderComms.claimInMyFrame(senderSeen, claim));
    }

    @Test
    @DisplayName("Reconstruction is unchanged by any rigid motion of the whole world")
    void claimSurvivesRigidMotionOfTheWholeWorld() {
        OrientedPoint observer = new OrientedPoint(3, 3, 0.25);
        OrientedPoint sender = new OrientedPoint(-9, 14, 2.9);
        OrientedPoint target = new OrientedPoint(40, -12, -0.6);

        OrientedPoint before = CyclebuilderComms.claimInMyFrame(
                inFrameOf(observer, sender), inFrameOf(sender, target));

        // Pick everything up and set it down somewhere else, rotated. Nothing about the
        // robots' relationship to each other changed, so nothing about the reconstruction
        // may change either -- if a global origin leaked into the math, this fails.
        RigidBodyTransformation worldShift =
                new RigidBodyTransformation(new OrientedPoint(-137, 88, 1.95));
        OrientedPoint after = CyclebuilderComms.claimInMyFrame(
                inFrameOf(worldShift.apply(observer), worldShift.apply(sender)),
                inFrameOf(worldShift.apply(sender), worldShift.apply(target)));

        assertPoseEquals(before, after);
    }

    @Test
    @DisplayName("Two robots heading for one spot each see the other's claim as their own")
    void bothRobotsAgreeWhenTheyContestOneSpot() {
        OrientedPoint contested = new OrientedPoint(17, 23, 0.4);
        OrientedPoint robotA = new OrientedPoint(-30, 5, 1.1);
        OrientedPoint robotB = new OrientedPoint(44, -18, -2.6);

        OrientedPoint claimFromA = inFrameOf(robotA, contested);
        OrientedPoint claimFromB = inFrameOf(robotB, contested);

        // Each reconstructs the other's claim and finds it on top of its own target. The
        // verdict is the same on both sides, which is what lets an id comparison pick
        // exactly one winner -- an inference-based test would have each robot checking a
        // different target, so both could yield or neither.
        assertPoseEquals(claimFromA,
                CyclebuilderComms.claimInMyFrame(inFrameOf(robotA, robotB), claimFromB));
        assertPoseEquals(claimFromB,
                CyclebuilderComms.claimInMyFrame(inFrameOf(robotB, robotA), claimFromA));
    }

    @Test
    @DisplayName("Robots heading for different spots reconstruct to different points")
    void distinctTargetsDoNotCollide() {
        OrientedPoint robotA = new OrientedPoint(0, 0, 0.3);
        OrientedPoint robotB = new OrientedPoint(50, 10, -1.4);
        OrientedPoint targetA = new OrientedPoint(20, 20, 0.0);
        OrientedPoint targetB = new OrientedPoint(80, -5, 0.0);

        OrientedPoint theirClaimInMyFrame = CyclebuilderComms.claimInMyFrame(
                inFrameOf(robotA, robotB), inFrameOf(robotB, targetB));

        double separation = inFrameOf(robotA, targetA).distance(theirClaimInMyFrame);
        assertTrue(separation > org.utils.MathUtils.REASSIGNMENT_POSITION_EPSILON,
                "distinct targets must stay outside the contention tolerance, was " + separation);
    }

    // ---------------------------------------------------------------------
    // The claim inbox
    // ---------------------------------------------------------------------

    @Test
    @DisplayName("A claim is addressed to no one in particular")
    void claimIsABroadcast() {
        TargetClaimMessage claim = new TargetClaimMessage(7, new OrientedPoint(1, 2, 0.5));
        assertEquals(AbstractMessage.BROADCAST, claim.getRecipient());
        assertEquals(7, claim.getSenderId());
    }

    @Test
    @DisplayName("A newer claim from the same sender replaces the older one")
    void latestClaimWins() {
        TestComms comms = new TestComms();
        comms.receiveClaim(new TargetClaimMessage(3, new OrientedPoint(1, 1, 0)));
        comms.receiveClaim(new TargetClaimMessage(3, new OrientedPoint(9, 9, 0)));

        assertEquals(1, comms.liveClaimCount(), "same sender must not accumulate entries");
        assertEquals(9, comms.claimFrom(3).getClaimInSenderFrame().getX(), TOL);
    }

    @Test
    @DisplayName("A claim survives one full time step, then expires")
    void claimExpiresAfterTwoPhases() {
        TestComms comms = new TestComms();
        comms.receiveClaim(new TargetClaimMessage(3, new OrientedPoint(1, 1, 0)));

        comms.age(2);
        assertEquals(1, comms.liveClaimCount(), "must survive the phase after it arrived");

        comms.age(2);
        assertEquals(0, comms.liveClaimCount(), "must be gone one full time step later");
    }

    @Test
    @DisplayName("Re-broadcasting keeps a claim alive indefinitely")
    void reBroadcastRefreshesTheClaim() {
        TestComms comms = new TestComms();

        // A robot that keeps emitting must never have its claim aged out from under it --
        // expiry is meant to retire robots that went quiet, not ones still talking.
        for (int phase = 0; phase < 10; phase++) {
            comms.age(2);
            comms.receiveClaim(new TargetClaimMessage(3, new OrientedPoint(1, 1, 0)));
        }

        assertEquals(1, comms.liveClaimCount());
    }
}
