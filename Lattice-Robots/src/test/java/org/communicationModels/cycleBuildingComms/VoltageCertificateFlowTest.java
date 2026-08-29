package org.communicationModels.cycleBuildingComms;

import java.util.ArrayList;
import java.util.List;

import org.communicationModels.cycleBuildingComms.Messages.AbstractMessage;
import org.communicationModels.cycleBuildingComms.Messages.PositioningMessage;
import org.communicationModels.cycleBuildingComms.Messages.VoltageCertificate;
import org.graphs.util.OrientedPoint;
import org.graphs.voltage.Face;
import org.graphs.voltage.HalfEdge;
import org.graphs.voltage.VoltageGraph;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.robots.GeometricCycleLatticeRobot;
import org.utils.logging.TickRecord;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Verifies the one thing a certificate actually carries that decides closure: the hop
 * count.
 *
 * <p>The accumulated transform does not decide anything, and cannot. Each relayer composes
 * {@code T(parent -> me) = P_{i-1}^-1 . P_i}, so a walk composes to
 * {@code P_0^-1 . P_1 . P_1^-1 . P_2 . ... = P_0^-1 . P_k} -- every intermediate pose
 * cancels -- and the initiator's closing hop {@code P_k^-1 . P_0} makes the product exactly
 * the identity for <em>any</em> poses whatsoever. Displacing a robot 150 units off its site
 * changes nothing; neither does a walk over four arbitrary poses that form no face at all.
 * Accumulating the per-hop residual against the ideal voltage does not rescue it either:
 * the measured walk is a closed loop so its displacements sum to zero, the ideal walk
 * closes so its voltages sum to zero, and on lattices whose voltages are pure translations
 * the difference of two zeros is zero.
 *
 * <p>That leaves closure resting on two discrete facts, both already carried: the
 * certificate returned to the robot that <em>minted</em> it, and it took the right number
 * of hops to get there. {@link #hopCountArrivesAsCycleLengthMinusOne} pins the second, and
 * it is load-bearing -- a walk of the wrong length that happens to end on the initiator is
 * otherwise indistinguishable from a real face.
 */
class VoltageCertificateFlowTest {

    /** Every certificate that reached {@code robotId}, read out of its inbox snapshots. */
    private static List<VoltageCertificate> certificatesDeliveredTo(List<TickRecord> records, int robotId) {
        List<VoltageCertificate> seen = new ArrayList<>();
        for (TickRecord record : records) {
            if (record.robotId() != robotId) {
                continue;
            }
            for (AbstractMessage message : record.before().queueInOrder()) {
                if (message instanceof PositioningMessage pm && pm.getRecipient() == robotId) {
                    seen.add(pm.getCertificate());
                }
            }
        }
        return seen;
    }

    private static void runFace(VoltageGraph graph, int expectedCycleLength) {
        Face face = null;
        for (Face candidate : graph.getFaces()) {
            if (candidate.getCycleLength() == expectedCycleLength) {
                face = candidate;
                break;
            }
        }
        assertNotNull(face, "lattice has no face of length " + expectedCycleLength);

        HalfEdge start = face.getBoundary().get(0);
        List<GeometricCycleLatticeRobot> robots =
                LatticeHarness.placeOnFace(graph, start, new OrientedPoint(0, 0, 0));

        // One robot is seeded as a root; it is not the only root for the whole run. This face
        // is all the swarm has, so the seed closes one corner and writes the other three off
        // for want of candidates -- and a part-failed root promotes the neighbours on the
        // corners it did close. Those become roots, mint certificates of their own, and offer
        // them straight back here. So "every certificate arriving at robot 0 was minted by
        // robot 0" is false, and relaying somebody else's walk is exactly what Phase 5 made a
        // root do (see CycleClosureCharacterizationTest#pathBetweenTwoRootsIsRelayedNotCertified).
        // The hop arithmetic under test is about the seed's OWN certificate, so select for it.
        GeometricCycleLatticeRobot initiator = robots.get(0);
        initiator.promoteToPrimaryRoot();

        List<TickRecord> records = LatticeHarness.tick(robots, 40);

        List<VoltageCertificate> returned = certificatesDeliveredTo(records, initiator.getRobotId());
        List<VoltageCertificate> ownWalk = returned.stream()
                .filter(certificate -> certificate.getInitiatorID() == initiator.getRobotId())
                .toList();
        assertFalse(ownWalk.isEmpty(),
                "no certificate minted by the initiator came back to it on a " + expectedCycleLength
                        + "-cycle, so this test is not exercising a full walk. It did receive "
                        + returned.size() + " certificate(s), minted by "
                        + returned.stream().map(VoltageCertificate::getInitiatorID).distinct().toList());

        boolean sawFullWalk = false;
        for (VoltageCertificate certificate : ownWalk) {
            if (certificate.getHops() == expectedCycleLength - 1) {
                sawFullWalk = true;
            }
        }

        assertTrue(sawFullWalk,
                "expected a certificate back at the initiator carrying "
                        + (expectedCycleLength - 1) + " hops on a " + expectedCycleLength
                        + "-cycle; the initiator contributes the closing hop itself. Saw: "
                        + ownWalk.stream().map(VoltageCertificate::getHops).toList());
    }

    @Test
    @DisplayName("a certificate returns to its initiator carrying cycleLength - 1 hops")
    void hopCountArrivesAsCycleLengthMinusOne() {
        runFace(org.graphs.voltage.SquareVoltageGraph.build(), 4);
    }

    @Test
    @DisplayName("the hop arithmetic holds on a three-cycle as well as a four-cycle")
    void hopCountHoldsOnATriangularFace() {
        runFace(org.graphs.voltage.TriangleVoltageGraph.build(), 3);
    }

    /**
     * The measured product is a consistency invariant, not a tolerance test: a walk that
     * returns to the robot that minted it closes to the identity <em>exactly</em>, because
     * the hops telescope. Pinned at 1e-12 rather than at a protocol epsilon, since there is
     * no drift budget for it to spend.
     *
     * <p>Written so that if the accumulation is ever changed to something that does not
     * telescope -- per-hop residuals, ideal voltages, outbound hops -- this fails and says
     * so, rather than the change silently producing a number nobody reads.
     */
    @Test
    @DisplayName("a closed walk's measured product is exactly the identity")
    void closedWalkMeasuresExactlyIdentity() {
        List<OrientedPoint> poses = List.of(
                new OrientedPoint(0, 0, 0),
                new OrientedPoint(70, 0, 0),
                new OrientedPoint(70, 70, 0),
                new OrientedPoint(0, 70, 0));

        // Relayers extend with T(parent -> me); the initiator adds the closing hop itself.
        VoltageCertificate certificate = new VoltageCertificate(0);
        for (int i = 1; i < poses.size(); i++) {
            certificate = certificate.extend(
                    new org.graphs.util.RigidBodyTransformation(poses.get(i - 1), poses.get(i)));
        }
        VoltageCertificate closed = certificate.extend(
                new org.graphs.util.RigidBodyTransformation(poses.get(poses.size() - 1), poses.get(0)));

        assertEquals(poses.size() - 1, certificate.getHops(), "hops before the closing hop");
        assertTrue(closed.getMeasuredVoltage().isApproximatelyIdentity(1e-12, 1e-12),
                "a returned certificate must close to the identity exactly, got "
                        + closed.getMeasuredVoltage().asPose());

        // And the reason it is only an invariant: displacing a robot far off its site does
        // not change the result, so this quantity can never gate closure on geometry.
        List<OrientedPoint> displaced = new ArrayList<>(poses);
        displaced.set(2, new OrientedPoint(-137.4, 88.1, 1.05));

        VoltageCertificate perturbed = new VoltageCertificate(0);
        for (int i = 1; i < displaced.size(); i++) {
            perturbed = perturbed.extend(
                    new org.graphs.util.RigidBodyTransformation(displaced.get(i - 1), displaced.get(i)));
        }
        perturbed = perturbed.extend(new org.graphs.util.RigidBodyTransformation(
                displaced.get(displaced.size() - 1), displaced.get(0)));

        assertTrue(perturbed.getMeasuredVoltage().isApproximatelyIdentity(1e-12, 1e-12),
                "the product telescopes, so it closes regardless of where the robots are. If "
                        + "this now fails, the accumulation has changed and the class javadoc "
                        + "needs revisiting -- closure may no longer rest on identity alone.");
    }

    /**
     * The initiator is minted with an empty walk. If this ever starts at one, every length
     * comparison in the closure predicate is off by one -- and it would still close faces,
     * just the wrong ones.
     */
    @Test
    @DisplayName("a freshly minted certificate has zero hops and names its initiator")
    void freshCertificateStartsEmpty() {
        VoltageCertificate fresh = new VoltageCertificate(42);

        assertEquals(42, fresh.getInitiatorID());
        assertEquals(0, fresh.getHops());
    }
}
