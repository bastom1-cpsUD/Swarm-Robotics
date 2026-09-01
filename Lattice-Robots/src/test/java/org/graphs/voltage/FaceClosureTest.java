package org.graphs.voltage;

import org.graphs.util.OrientedPoint;
import org.graphs.util.RigidBodyTransformation;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.robots.GeometricCycleLatticeRobot;
import org.utils.MathUtils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Phase 1 of the communication-tuple migration: the geometry a face certificate will be
 * judged by, tested before anything depends on it.
 *
 * <p>Several of these are permanent traps rather than checks of new behaviour. The
 * edge-length bound guards an invariant later phases spend: that a robot's parent is
 * always still in radio range. The pure-translation canary guards the opposite kind of
 * thing -- a hazard that is currently inert and will not stay that way.
 */
class FaceClosureTest {

    /**
     * Tight enough to be an equality assertion in all but name. These are exact lattice
     * constructions, not measurements, so anything above floating-point noise here is a
     * real defect rather than accumulated drift.
     */
    private static final double EXACT_POSITION = 1e-9;
    private static final double EXACT_ANGLE = 1e-9;

    /**
     * Illustrative tolerances for exercising the two-tolerance overload -- a
     * position-sized one and an angle-sized one, far enough apart that a single scalar
     * cannot serve both.
     *
     * <p>Local to this file on purpose. They were briefly {@code MathUtils} constants
     * intended to gate face closure, until the certificate's accumulated product turned
     * out to telescope: each relayer composes {@code T(parent -> me)}, so a walk composes
     * to {@code P_0^-1 . P_k} and the initiator's closing hop makes it exactly the
     * identity for any poses at all. There is no drift budget to spend, so there is no
     * protocol tolerance to name -- closure rests on the initiator's identity and the hop
     * count. These stay as test fixtures for the primitive, which is still correct and
     * still worth pinning.
     */
    private static final double POSITION_TOLERANCE = 1.0;
    private static final double ANGLE_TOLERANCE = 1e-1;

    /** Every shipped lattice, named so a failure says which one broke. */
    static Stream<Arguments> lattices() {
        return Stream.of(
                Arguments.of("Square", SquareVoltageGraph.build()),
                Arguments.of("Triangle", TriangleVoltageGraph.build()),
                Arguments.of("Hexagon", HexagonVoltageGraph.build()),
                Arguments.of("OctagonSquare", OctagonSquareVoltageGraph.build()),
                Arguments.of("SnubSquare", SnubSquareVoltageGraph.build()),
                Arguments.of("SnubHexagon", SnubHexagonVoltageGraph.build()),
                Arguments.of("HexagonTriangle", HexagonTriangleVoltageGraph.build()),
                Arguments.of("HexagonSquareTriangle", HexagonSquareTriangleVoltageGraph.build()),
                Arguments.of("DodecagonTriangle", DodecagonTriangleVoltageGraph.build()),
                Arguments.of("DodecagonHexagonSquare", DodecagonHexagonSquareVoltageGraph.build()),
                Arguments.of("ElongatedTriangular", ElongatedTriangularVoltageGraph.build()));
    }

    // --- composition order -------------------------------------------------------

    /**
     * A certificate accumulates one hop at a time as it travels, never as a batch. This
     * asserts the two agree, and that both agree with the holonomy the builder captured
     * independently at construction.
     */
    @ParameterizedTest(name = "{0}")
    @MethodSource("lattices")
    @DisplayName("incremental compose in walk order equals the stored holonomy")
    void incrementalComposeMatchesStoredHolonomy(String name, VoltageGraph graph) {
        for (Face face : graph.getFaces()) {
            List<HalfEdge> boundary = face.getBoundary();

            RigidBodyTransformation running = RigidBodyTransformation.identity();
            for (HalfEdge h : boundary) {
                running = running.compose(h.getVoltage());
            }

            assertPosesMatch(name + " face " + face.getId(), face.getHolonomy(), running);
            assertTrue(running.isApproximatelyIdentity(EXACT_POSITION, EXACT_ANGLE),
                    name + " face " + face.getId() + ": walk-order product is not identity, got "
                            + running.asPose());
            assertTrue(graph.validateCycle(boundary, EXACT_POSITION, EXACT_ANGLE),
                    name + " face " + face.getId() + ": validateCycle rejected its own boundary");
        }
    }

    /**
     * The ordering trap, inverted into a canary, because there is currently nothing to
     * trap.
     *
     * <p>Every role in every shipped lattice is declared at orientation 0, so every
     * voltage is a pure translation -- and translations commute. Composition order
     * therefore cannot affect closure on any face of any current lattice. This was
     * measured, not assumed: reversing octagon-square's 8-cycle, the face the DCEL plan
     * names as the most likely to catch an ordering bug, still lands on the identity to
     * within 1e-14.
     *
     * <p>Two things follow, and both stop holding the moment a single role is declared
     * with a nonzero orientation:
     * <ul>
     *   <li>a reversal test would assert a falsehood, so none is written here;</li>
     *   <li>the angular half of any closure test has no discriminating power over
     *       ideal voltages -- the whole content of a closure test lives in the position
     *       term.</li>
     * </ul>
     *
     * <p>So the precondition is asserted instead. <strong>If this test fails, the
     * ordering hazard has just become real</strong> -- the fix is to write a reversal
     * trap on the longest face that mixes edge types and to re-examine the angular half
     * of the closure predicate, not to delete this test.
     */
    @Test
    @DisplayName("canary: all voltages are pure translations, so order cannot matter yet")
    void voltagesArePureTranslations_whichIsWhyNoReversalTrapExists() {
        for (Arguments arg : lattices().toList()) {
            String name = (String) arg.get()[0];
            VoltageGraph graph = (VoltageGraph) arg.get()[1];

            for (Role role : graph.getRoles()) {
                for (HalfEdge h : graph.getOutgoingHalfEdges(role)) {
                    assertEquals(0.0, h.getVoltage().getRotation(), 1e-12,
                            name + " half-edge " + h.getId() + " carries a rotation, so the "
                                    + "lattice's voltages no longer commute. Composition order "
                                    + "now affects closure: add a reversal trap on the longest "
                                    + "mixed face, and re-check the angular half of the closure "
                                    + "predicate, which until now has been testing nothing.");
                }
            }
        }
    }

    /**
     * The consequence of the canary above, stated where it can be seen rather than left
     * implicit. Kept as a separate test so that when the canary fires this one fails
     * alongside it and points at the same conclusion.
     */
    @Test
    @DisplayName("OctagonSquare: the 8-cycle is currently order-insensitive")
    void reversedOctagonBoundaryStillClosesWhileVoltagesCommute() {
        VoltageGraph graph = OctagonSquareVoltageGraph.build();

        Face octagon = graph.getFaces().stream()
                .filter(f -> f.getCycleLength() == 8)
                .findFirst()
                .orElseThrow(() -> new AssertionError("OctagonSquare has no 8-cycle face"));

        List<HalfEdge> reversed = new ArrayList<>(octagon.getBoundary());
        Collections.reverse(reversed);

        RigidBodyTransformation running = RigidBodyTransformation.identity();
        for (HalfEdge h : reversed) {
            running = running.compose(h.getVoltage());
        }

        assertTrue(running.isApproximatelyIdentity(EXACT_POSITION, EXACT_ANGLE),
                "The reversed 8-cycle no longer closes, which means the voltages have stopped "
                        + "commuting. Composition order is now load-bearing: this test should "
                        + "become an assertFalse, and the certificate's accumulation order needs "
                        + "auditing end to end. Got " + running.asPose());
    }

    // --- boundary structure ------------------------------------------------------

    /**
     * One obligation per edge assumes a face boundary visits each half-edge once.
     * {@code VoltageGraphBuilder.discoverFaces} tolerates {@code laps > 1}, where that
     * would not hold, so it is asserted rather than assumed.
     */
    @ParameterizedTest(name = "{0}")
    @MethodSource("lattices")
    @DisplayName("every face boundary visits distinct half-edges")
    void everyFaceBoundaryHasDistinctHalfEdges(String name, VoltageGraph graph) {
        for (Face face : graph.getFaces()) {
            List<HalfEdge> boundary = face.getBoundary();
            Set<HalfEdge> distinct = new HashSet<>(boundary);

            assertEquals(boundary.size(), distinct.size(),
                    name + " face " + face.getId() + " repeats a half-edge on its boundary; "
                            + "one obligation per edge cannot key a walk that visits an edge twice");
        }
    }

    // --- the invariant later phases spend ----------------------------------------

    /**
     * Every parent in a communication tuple sits exactly one lattice edge from its child.
     * Phase 5 drops the timeout backstop on {@code CertificateLostMessage} because that
     * parent is therefore always reachable; if this bound ever fails, that recovery path
     * fails with it and a root can hang permanently.
     */
    @ParameterizedTest(name = "{0}")
    @MethodSource("lattices")
    @DisplayName("every edge is shorter than COMM_RANGE")
    void everyLatticeEdgeIsWithinCommRange(String name, VoltageGraph graph) {
        for (Role role : graph.getRoles()) {
            for (HalfEdge h : graph.getOutgoingHalfEdges(role)) {
                OrientedPoint offset = h.getVoltage().asPose();
                double length = Math.hypot(offset.getX(), offset.getY());

                assertTrue(length < GeometricCycleLatticeRobot.COMM_RANGE,
                        name + " half-edge " + h.getId() + " is " + length + " units long, at or "
                                + "beyond COMM_RANGE " + GeometricCycleLatticeRobot.COMM_RANGE
                                + ". A parent one edge away would drop out of radio range, so a "
                                + "lost-certificate report could not be delivered and a root "
                                + "waiting on that child would never make progress.");
            }
        }
    }

    // --- the new predicates ------------------------------------------------------

    @ParameterizedTest(name = "{0}")
    @MethodSource("lattices")
    @DisplayName("maxCycleLength is the longest face, and every face length is recognized")
    void cycleLengthPredicatesAgreeWithTheFaces(String name, VoltageGraph graph) {
        int longest = graph.getFaces().stream().mapToInt(Face::getCycleLength).max().orElseThrow();
        assertEquals(longest, graph.maxCycleLength(), name);

        for (Face face : graph.getFaces()) {
            assertTrue(graph.isFaceCycleLength(face.getCycleLength()), name);
        }
        assertFalse(graph.isFaceCycleLength(longest + 1), name + ": accepted an over-long walk");
        assertFalse(graph.isFaceCycleLength(0), name + ": accepted an empty walk");
    }

    /**
     * Why the two-tolerance overload exists. The single-scalar form compares raw matrix
     * entries, so one number has to serve both dimensionless sines and translations in
     * lattice units, and no value gets both right.
     */
    @Test
    @DisplayName("one scalar cannot bound both rotation and translation")
    void twoToleranceOverloadSeparatesUnitsTheSingleScalarConflates() {
        // A half-radian rotation about the origin: no translation at all, but wildly wrong.
        RigidBodyTransformation spun = new RigidBodyTransformation(new OrientedPoint(0, 0, 0.5));
        assertTrue(spun.isApproximatelyIdentity(1.0),
                "precondition: a position-sized scalar admits this rotation");
        assertFalse(spun.isApproximatelyIdentity(1.0, ANGLE_TOLERANCE),
                "the overload must reject a rotation no position-sized tolerance can catch");

        // Half a unit of drift, perfectly aligned: acceptable for a closure, but rejected
        // outright by any scalar tight enough to have pinned the rotation above.
        RigidBodyTransformation drifted = new RigidBodyTransformation(new OrientedPoint(0.5, 0, 0));
        assertFalse(drifted.isApproximatelyIdentity(ANGLE_TOLERANCE),
                "precondition: an angle-sized scalar rejects this translation");
        assertTrue(drifted.isApproximatelyIdentity(
                        POSITION_TOLERANCE, ANGLE_TOLERANCE),
                "the overload must admit drift well inside the closure budget");
    }

    @Test
    @DisplayName("validateCycle rejects an empty walk")
    void emptyWalkIsNotACycle() {
        VoltageGraph graph = SquareVoltageGraph.build();
        assertFalse(graph.validateCycle(List.of(),
                POSITION_TOLERANCE, ANGLE_TOLERANCE));
    }

    // --- helpers -----------------------------------------------------------------

    private static void assertPosesMatch(String context,
                                         RigidBodyTransformation expected,
                                         RigidBodyTransformation actual) {
        OrientedPoint e = expected.asPose();
        OrientedPoint a = actual.asPose();
        assertEquals(e.getX(), a.getX(), EXACT_POSITION, context + " x");
        assertEquals(e.getY(), a.getY(), EXACT_POSITION, context + " y");
        assertEquals(0.0, MathUtils.angleDifference(e.getOrientation(), a.getOrientation()),
                EXACT_ANGLE, context + " orientation");
    }
}
