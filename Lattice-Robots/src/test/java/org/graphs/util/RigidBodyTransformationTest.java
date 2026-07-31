package org.graphs.util;

import Jama.Matrix;

import org.graphs.lattice.HexagonLattice;
import org.graphs.lattice.LatticeEdge;
import org.graphs.lattice.SquareLattice;
import org.graphs.lattice.Vertex;
import org.graphs.voltage.DodecagonHexagonSquareVoltageGraph;
import org.graphs.voltage.DodecagonTriangleVoltageGraph;
import org.graphs.voltage.ElongatedTriangularVoltageGraph;
import org.graphs.voltage.HalfEdge;
import org.graphs.voltage.HexagonSquareTriangleVoltageGraph;
import org.graphs.voltage.HexagonTriangleVoltageGraph;
import org.graphs.voltage.HexagonVoltageGraph;
import org.graphs.voltage.OctagonSquareVoltageGraph;
import org.graphs.voltage.Role;
import org.graphs.voltage.SnubHexagonVoltageGraph;
import org.graphs.voltage.SnubSquareVoltageGraph;
import org.graphs.voltage.SquareVoltageGraph;
import org.graphs.voltage.VoltageGraph;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.utils.MathUtils;

import java.util.List;
import java.util.function.Supplier;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Covers the angle-preservation change to {@link RigidBodyTransformation}.
 *
 * <p>The change had two parts: {@code apply()} now composes the input pose's own
 * orientation instead of discarding it, and the two-argument constructor now rotates its
 * translation into {@code from}'s frame. Both are no-ops over the inputs the codebase
 * actually reaches today, which is what {@link #apply_isBitIdenticalForZeroOrientationInputs()}
 * and {@link #twoArgCtor_isBitIdenticalWhenFromHasZeroOrientation()} assert -- they
 * compare against verbatim copies of the old formulas. The remaining tests pin the
 * behaviour that was previously wrong.
 */
class RigidBodyTransformationTest {

    private static final double TOL = 1e-9;

    // ---------------------------------------------------------------------
    // Verbatim copies of the pre-change formulas, for differential testing.
    // ---------------------------------------------------------------------

    /** The old apply(): returned the transform's own rotation, ignoring point.orientation. */
    private static OrientedPoint legacyApply(RigidBodyTransformation t, OrientedPoint point) {
        Matrix pointMatrix = new Matrix(new double[][] {{point.x}, {point.y}, {1}});
        Matrix result = t.matrix.times(pointMatrix);
        double x = result.get(0, 0);
        double y = result.get(1, 0);
        double orientation = Math.atan2(t.matrix.get(1, 0), t.matrix.get(0, 0));
        return new OrientedPoint(x, y, orientation);
    }

    /** The old two-arg constructor: translation was the raw, unrotated global delta. */
    private static Matrix legacyTwoArg(OrientedPoint from, OrientedPoint to) {
        OrientedPoint delta = new OrientedPoint(to.x - from.x, to.y - from.y,
                to.getOrientation() - from.getOrientation());
        double cosTheta = Math.cos(delta.getOrientation());
        double sinTheta = Math.sin(delta.getOrientation());
        return new Matrix(new double[][] {
            {cosTheta, -sinTheta, delta.x},
            {sinTheta, cosTheta, delta.y},
            {0, 0, 1}
        });
    }

    private static void assertMatricesBitIdentical(Matrix expected, Matrix actual) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                assertEquals(expected.get(i, j), actual.get(i, j),
                        "matrix entry (" + i + "," + j + ")");
            }
        }
    }

    // ---------------------------------------------------------------------
    // 1. Differential no-op proof
    // ---------------------------------------------------------------------

    @Test
    @DisplayName("apply() is bit-identical to the old formula for zero-orientation inputs")
    void apply_isBitIdenticalForZeroOrientationInputs() {
        // Every apply() call site on the CyclebuilderComms path passes either the origin
        // pose or a pose whose orientation is zero, so this is the domain that must not
        // move.
        //
        // Position is asserted bit-exact -- it feeds every distance comparison in the
        // algorithm, so a single changed bit there could change a decision.
        //
        // Orientation is asserted as an angle, not as a bit pattern, because the new
        // apply() deliberately normalizes into (-pi, pi] while the old one returned
        // atan2's raw [-pi, pi]. The two therefore disagree at exactly -pi, where the
        // new code reports +pi: the same heading in the documented canonical form.
        // Nothing compares orientations for exact equality, so this is a difference in
        // spelling rather than in value.
        for (double theta = -Math.PI; theta <= Math.PI; theta += Math.PI / 8) {
            for (double tx = -70; tx <= 70; tx += 35) {
                for (double ty = -70; ty <= 70; ty += 35) {
                    RigidBodyTransformation t =
                            new RigidBodyTransformation(new OrientedPoint(tx, ty, theta));

                    for (double px = -50; px <= 50; px += 25) {
                        for (double py = -50; py <= 50; py += 25) {
                            OrientedPoint in = new OrientedPoint(px, py, 0);
                            OrientedPoint expected = legacyApply(t, in);
                            OrientedPoint actual = t.apply(in);

                            assertEquals(expected.x, actual.x, "x");
                            assertEquals(expected.y, actual.y, "y");
                            assertEquals(0.0,
                                    MathUtils.angleDifference(expected.getOrientation(),
                                            actual.getOrientation()),
                                    0.0,
                                    "orientation differs by more than a (-pi, pi] wrap");
                        }
                    }
                }
            }
        }
    }

    @Test
    @DisplayName("apply() and the old formula agree exactly except at the -pi wrap")
    void apply_differsFromLegacyOnlyAtTheNegativePiWrap() {
        // Pins the one input where the two spellings diverge, so the exemption above is
        // a known, bounded case rather than a blanket loosening of the comparison.
        RigidBodyTransformation halfTurn =
                new RigidBodyTransformation(new OrientedPoint(0, 0, -Math.PI));

        double legacy = legacyApply(halfTurn, new OrientedPoint(0, 0, 0)).getOrientation();
        double current = halfTurn.apply(new OrientedPoint(0, 0, 0)).getOrientation();

        assertEquals(-Math.PI, legacy, TOL);
        assertEquals(Math.PI, current, TOL);
        assertEquals(0.0, MathUtils.angleDifference(legacy, current), TOL);
    }

    @Test
    @DisplayName("two-arg ctor is bit-identical to the old formula when from's orientation is zero")
    void twoArgCtor_isBitIdenticalWhenFromHasZeroOrientation() {
        // Every Role pose and every Vertex pose in the repo has orientation zero, so
        // R(-theta_from) is exactly the identity and the new translation reduces to the
        // old one bit-for-bit.
        for (double fx = -70; fx <= 70; fx += 35) {
            for (double fy = -70; fy <= 70; fy += 35) {
                OrientedPoint from = new OrientedPoint(fx, fy, 0);

                for (double tx = -70; tx <= 70; tx += 35) {
                    for (double ty = -70; ty <= 70; ty += 35) {
                        for (double tth = -Math.PI; tth <= Math.PI; tth += Math.PI / 4) {
                            OrientedPoint to = new OrientedPoint(tx, ty, tth);
                            assertMatricesBitIdentical(
                                    legacyTwoArg(from, to),
                                    new RigidBodyTransformation(from, to).matrix);
                        }
                    }
                }
            }
        }
    }

    // ---------------------------------------------------------------------
    // 2. New-behaviour gate — each of these failed before the change
    // ---------------------------------------------------------------------

    @Test
    @DisplayName("self, expressed in self's own frame, is exactly the origin")
    void selfInOwnFrameIsExactlyTheOrigin() {
        OrientedPoint self = new OrientedPoint(17, -5, 2.3);

        OrientedPoint origin = new RigidBodyTransformation(self).inverse().apply(self);

        assertEquals(0.0, origin.x, TOL);
        assertEquals(0.0, origin.y, TOL);
        assertEquals(0.0, origin.getOrientation(), TOL); // was -2.3
    }

    @Test
    @DisplayName("apply() composes the pose's own orientation with the transform's rotation")
    void apply_composesOrientations() {
        RigidBodyTransformation t =
                new RigidBodyTransformation(new OrientedPoint(3, 4, Math.PI / 2));

        OrientedPoint result = t.apply(new OrientedPoint(1, 0, Math.PI / 4));

        assertEquals(3.0, result.x, TOL); // R(90) * (1,0) = (0,1), plus (3,4)
        assertEquals(5.0, result.y, TOL);
        assertEquals(3 * Math.PI / 4, result.getOrientation(), TOL);
    }

    @Test
    @DisplayName("apply() normalizes the composed orientation into (-pi, pi]")
    void apply_normalizesComposedOrientation() {
        RigidBodyTransformation t =
                new RigidBodyTransformation(new OrientedPoint(0, 0, 3 * Math.PI / 4));

        OrientedPoint result = t.apply(new OrientedPoint(0, 0, 3 * Math.PI / 4));

        assertEquals(-Math.PI / 2, result.getOrientation(), TOL); // 3pi/2 wrapped
    }

    @Test
    @DisplayName("asPose() equals apply() on the origin pose")
    void asPose_matchesApplyOnOrigin() {
        RigidBodyTransformation t =
                new RigidBodyTransformation(new OrientedPoint(11, -3, 0.77));

        OrientedPoint viaApply = t.apply(new OrientedPoint(0, 0, 0));
        OrientedPoint viaAsPose = t.asPose();

        assertEquals(viaApply.x, viaAsPose.x);
        assertEquals(viaApply.y, viaAsPose.y);
        assertEquals(viaApply.getOrientation(), viaAsPose.getOrientation());
    }

    @Test
    @DisplayName("two-arg ctor rotates the translation into from's frame")
    void twoArgCtor_expressesToInFromsFrame() {
        OrientedPoint from = new OrientedPoint(10, 0, Math.PI / 2);
        OrientedPoint to = new OrientedPoint(10, 7, Math.PI);

        OrientedPoint relative = new RigidBodyTransformation(from, to).asPose();

        assertEquals(7.0, relative.x, TOL); // R(-90) * (0,7) = (7,0)
        assertEquals(0.0, relative.y, TOL);
        assertEquals(Math.PI / 2, relative.getOrientation(), TOL);
    }

    @Test
    @DisplayName("T(from).compose(T(from,to)) equals T(to)")
    void twoArgCtor_satisfiesTheCompositionLaw() {
        OrientedPoint from = new OrientedPoint(10, 0, Math.PI / 2);
        OrientedPoint to = new OrientedPoint(10, 7, Math.PI);

        RigidBodyTransformation composed = new RigidBodyTransformation(from)
                .compose(new RigidBodyTransformation(from, to));

        assertMatricesApproximatelyEqual(new RigidBodyTransformation(to).matrix, composed.matrix);
    }

    @Test
    @DisplayName("T(a,b) and T(b,a) are mutual inverses even under rotation")
    void twoArgCtor_roundTrips() {
        OrientedPoint a = new OrientedPoint(1, 2, 0.4);
        OrientedPoint b = new OrientedPoint(-3, 5, -1.1);

        assertTrue(new RigidBodyTransformation(a, b)
                .isInverse(new RigidBodyTransformation(b, a)));
    }

    @Test
    @DisplayName("getRotation() recovers the transform's own rotation")
    void getRotation_recoversTheRotation() {
        for (double theta = -3.0; theta <= 3.0; theta += 0.25) {
            RigidBodyTransformation t =
                    new RigidBodyTransformation(new OrientedPoint(5, -2, theta));
            assertEquals(theta, t.getRotation(), TOL);
        }
    }

    private static void assertMatricesApproximatelyEqual(Matrix expected, Matrix actual) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                assertEquals(expected.get(i, j), actual.get(i, j), TOL,
                        "matrix entry (" + i + "," + j + ")");
            }
        }
    }

    // ---------------------------------------------------------------------
    // 3. Tier 2 gate — the already-relative double-subtraction
    // ---------------------------------------------------------------------

    @Test
    @DisplayName("HexagonLattice's v1-side edges match their declared offsets")
    void hexagonLattice_v1EdgeTransformsMatchDeclaredOffsets() {
        // v1 sits at (25, 43.3), so before the fix LatticeEdge subtracted that pose from
        // an offset already relative to v1: e5 ("Right", (50, 0)) became (25, -43.3).
        HexagonLattice lattice = new HexagonLattice();

        Vertex v1 = lattice.getVertexByID(2);
        assertNotNull(v1, "v1 not found in HexagonLattice");

        for (LatticeEdge edge : lattice.getOutgoingEdges(v1)) {
            OrientedPoint declared = edge.getToPos();
            OrientedPoint actual = edge.getEdgeTransformation().asPose();

            assertEquals(declared.x, actual.x, TOL, "edge " + edge.getId() + " x");
            assertEquals(declared.y, actual.y, TOL, "edge " + edge.getId() + " y");
        }
    }

    // ---------------------------------------------------------------------
    // 4. Lattice sweep
    // ---------------------------------------------------------------------

    @Test
    @DisplayName("every lattice still builds, and every voltage is unchanged and rotation-free")
    void allLatticesStillBuildWithUnchangedVoltages() {
        List<Supplier<VoltageGraph>> factories = List.of(
                SquareVoltageGraph::build,
                HexagonVoltageGraph::build,
                OctagonSquareVoltageGraph::build,
                SnubSquareVoltageGraph::build,
                SnubHexagonVoltageGraph::build,
                HexagonTriangleVoltageGraph::build,
                HexagonSquareTriangleVoltageGraph::build,
                DodecagonTriangleVoltageGraph::build,
                DodecagonHexagonSquareVoltageGraph::build,
                ElongatedTriangularVoltageGraph::build);

        for (Supplier<VoltageGraph> factory : factories) {
            // build() throws if any face's holonomy fails to converge to identity.
            VoltageGraph graph = factory.get();

            assertFalse(graph.getFaces().isEmpty(), "no faces discovered");

            for (Role role : graph.getRoles()) {
                for (HalfEdge h : graph.getOutgoingHalfEdges(role)) {
                    // Every declared lattice is built from pure translations; if that
                    // ever stops being true the assumption behind the no-op claim is
                    // gone and this fires.
                    assertEquals(0.0, h.getVoltage().getRotation(), TOL);

                    // asPose() must agree with the apply(origin) idiom it replaced.
                    OrientedPoint viaApply = h.getVoltage().apply(new OrientedPoint(0, 0, 0));
                    OrientedPoint viaAsPose = h.getVoltage().asPose();
                    assertEquals(viaApply.x, viaAsPose.x);
                    assertEquals(viaApply.y, viaAsPose.y);
                    assertEquals(viaApply.getOrientation(), viaAsPose.getOrientation());
                }
            }
        }
    }

    @Test
    @DisplayName("SquareLattice edge transforms match their declared offsets")
    void squareLattice_edgeTransformsMatchDeclaredOffsets() {
        SquareLattice lattice = new SquareLattice();

        for (Vertex v : lattice.getVertices()) {
            for (LatticeEdge edge : lattice.getOutgoingEdges(v)) {
                OrientedPoint declared = edge.getToPos();
                OrientedPoint actual = edge.getEdgeTransformation().asPose();

                assertEquals(declared.x, actual.x, TOL);
                assertEquals(declared.y, actual.y, TOL);
            }
        }
    }
}
