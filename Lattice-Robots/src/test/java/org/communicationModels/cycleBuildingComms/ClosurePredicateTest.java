package org.communicationModels.cycleBuildingComms;

import org.graphs.util.OrientedPoint;
import org.graphs.util.RigidBodyTransformation;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * The closure rule on its own, away from any robot.
 *
 * <p>Three conjuncts decide whether a face is closed, and each one exists to catch a
 * different failure. Tested here rather than through the simulation because the interesting
 * cases are precisely the ones the simulation is built never to produce -- a certificate
 * judged by the wrong robot, or a walk that closes geometrically around the wrong face --
 * so reaching them through the protocol would mean first breaking the protocol.
 */
class ClosurePredicateTest {

    private static final int SELF = 7;

    private static RigidBodyTransformation identity() {
        return RigidBodyTransformation.identity();
    }

    private static RigidBodyTransformation displacedBy(double dx, double dy) {
        return new RigidBodyTransformation(new OrientedPoint(dx, dy, 0));
    }

    @Test
    @DisplayName("a certificate that came home, at the right length, with an identity product closes")
    void allThreeConjunctsClose() {
        assertTrue(CyclebuilderComms.closesFace(SELF, SELF, 3, 4, identity()),
                "three hops plus this robot's own closing hop is a four-hop face");
    }

    /**
     * The conjunct that fixes defect 2. Under the old rule this was a closure, because the
     * robot judging it was a root standing at the right pose.
     */
    @Test
    @DisplayName("a certificate minted by someone else never closes, however good it looks")
    void anotherRobotsCertificateNeverCloses() {
        assertFalse(CyclebuilderComms.closesFace(3, SELF, 3, 4, identity()),
                "only the initiator may decide its own certificate");
    }

    /**
     * The conjunct that separates faces meeting at one corner -- a triangle from a square on
     * snub square, a square from an octagon on octagon-square. Geometry alone cannot do it:
     * the product telescopes to the identity for any walk that physically returned, so a
     * four-hop walk closing where a three-hop face was expected passes the transform test
     * and must be caught on length.
     */
    @Test
    @DisplayName("a walk of the wrong length is rejected even though its product is identity")
    void walkOfWrongLengthIsRejected() {
        assertFalse(CyclebuilderComms.closesFace(SELF, SELF, 3, 3, identity()),
                "four hops around a three-hop face is not that face");
        assertFalse(CyclebuilderComms.closesFace(SELF, SELF, 2, 4, identity()),
                "three hops around a four-hop face is the defect-2 path: short of the corner");
        assertFalse(CyclebuilderComms.closesFace(SELF, SELF, 7, 4, identity()),
                "a walk that lapped the face twice has not closed it once");
    }

    /**
     * Not a drift budget. The measured product telescopes, so a walk that genuinely returned
     * closes at zero whatever the placement error along the way -- which means a non-zero
     * product cannot be ordinary error and must not be tolerated as such. 150 units is the
     * displacement used in {@code VoltageCertificateFlowTest} to show the telescoping; here
     * a hundredth of a unit is already far outside what this conjunct accepts.
     */
    @Test
    @DisplayName("a non-identity product is rejected at a scale no lattice tolerance would catch")
    void nonIdentityProductIsRejected() {
        assertFalse(CyclebuilderComms.closesFace(SELF, SELF, 3, 4, displacedBy(0.01, 0)),
                "a hundredth of a unit is not drift -- there is no drift here to absorb");
        assertFalse(CyclebuilderComms.closesFace(SELF, SELF, 3, 4, displacedBy(0, 1e-6)),
                "if this ever passes, the tolerance has been loosened into a drift budget; "
                        + "read Finding 1 in the migration plan before changing it back");
        assertTrue(CyclebuilderComms.closesFace(SELF, SELF, 3, 4, displacedBy(1e-12, 1e-12)),
                "floating-point noise from the compositions themselves must still close");
    }

    @Test
    @DisplayName("a missing product is a non-closure, not a pass")
    void absentProductDoesNotClose() {
        assertFalse(CyclebuilderComms.closesFace(SELF, SELF, 3, 4, null),
                "an unmeasured closing hop means the walk was never verified");
    }

    /**
     * A single robot claiming to have closed a face by itself. Length catches it, which is
     * worth pinning because identity does not -- the robot really is the initiator -- and
     * neither does the product, which is trivially the identity over an empty walk.
     */
    @Test
    @DisplayName("a zero-hop certificate does not close any face")
    void aWalkThatNeverLeftDoesNotClose() {
        assertFalse(CyclebuilderComms.closesFace(SELF, SELF, 0, 4, identity()));
        assertFalse(CyclebuilderComms.closesFace(SELF, SELF, 0, 3, identity()));
    }
}