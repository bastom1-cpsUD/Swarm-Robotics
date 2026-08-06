package org.utils;

import org.graphs.util.OrientedPoint;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Covers {@link Vec2} and the collinearity-tolerance change in {@link MathUtils}.
 *
 * <p>{@code Vec2} replaced a set of {@code MathUtils} helpers that represented vectors as
 * {@link OrientedPoint}s with orientation hardcoded to zero. The equivalence tests below
 * compare each new method against a verbatim copy of the helper it replaced, bit-exact,
 * so the migration is provably arithmetic-preserving.
 */
class Vec2Test {

    // ---------------------------------------------------------------------
    // Verbatim copies of the deleted MathUtils helpers.
    // ---------------------------------------------------------------------

    private static OrientedPoint legacyVectorSum(OrientedPoint v1, OrientedPoint v2) {
        return new OrientedPoint(v1.getX() + v2.getX(), v1.getY() + v2.getY(), 0);
    }

    private static OrientedPoint legacyVectorBetween(OrientedPoint p1, OrientedPoint p2) {
        return new OrientedPoint(p2.getX() - p1.getX(), p2.getY() - p1.getY(), 0);
    }

    private static double legacyDotProduct(OrientedPoint v1, OrientedPoint v2) {
        return v1.getX() * v2.getX() + v1.getY() * v2.getY();
    }

    private static double legacyCrossProduct(OrientedPoint v1, OrientedPoint v2) {
        return v1.getX() * v2.getY() - v1.getY() * v2.getX();
    }

    private static double legacyMagnitude(OrientedPoint v1) {
        return Math.sqrt(v1.getX() * v1.getX() + v1.getY() * v1.getY());
    }

    private static double legacyAngleBetween(OrientedPoint v1, OrientedPoint v2) {
        return MathUtils.normalizeAngle(
                Math.atan2(legacyCrossProduct(v1, v2), legacyDotProduct(v1, v2)));
    }

    // ---------------------------------------------------------------------
    // 5. Vec2 equivalence — bit-exact against the helpers it replaced
    // ---------------------------------------------------------------------

    @Test
    @DisplayName("every Vec2 operation reproduces its old MathUtils helper bit-for-bit")
    void vec2_matchesLegacyHelpersBitForBit() {
        for (double ax = -70; ax <= 70; ax += 17.5) {
            for (double ay = -70; ay <= 70; ay += 17.5) {
                for (double bx = -70; bx <= 70; bx += 17.5) {
                    for (double by = -70; by <= 70; by += 17.5) {
                        OrientedPoint pa = new OrientedPoint(ax, ay, 0);
                        OrientedPoint pb = new OrientedPoint(bx, by, 0);
                        Vec2 a = new Vec2(ax, ay);
                        Vec2 b = new Vec2(bx, by);

                        assertEquals(legacyVectorSum(pa, pb).getX(), a.plus(b).x);
                        assertEquals(legacyVectorSum(pa, pb).getY(), a.plus(b).y);

                        assertEquals(legacyVectorBetween(pa, pb).getX(), Vec2.between(pa, pb).x);
                        assertEquals(legacyVectorBetween(pa, pb).getY(), Vec2.between(pa, pb).y);

                        assertEquals(legacyDotProduct(pa, pb), a.dot(b));
                        assertEquals(legacyCrossProduct(pa, pb), a.cross(b));
                        assertEquals(legacyMagnitude(pa), a.magnitude());

                        if (a.magnitude() != 0.0 && b.magnitude() != 0.0) {
                            assertEquals(legacyAngleBetween(pa, pb), a.angleTo(b));
                        }
                    }
                }
            }
        }
    }

    @Test
    @DisplayName("of() and asOrientedPoint() round-trip, dropping any heading")
    void vec2_ofAndAsOrientedPointRoundTrip() {
        OrientedPoint p = new OrientedPoint(3.5, -2.25, 1.1);

        OrientedPoint roundTripped = Vec2.of(p).asOrientedPoint();

        assertEquals(3.5, roundTripped.x);
        assertEquals(-2.25, roundTripped.y);
        assertEquals(0.0, roundTripped.getOrientation(), "a vector carries no heading");
    }

    // ---------------------------------------------------------------------
    // 7. Collinearity epsilon — signs preserved, degeneracies now caught
    // ---------------------------------------------------------------------

    @Test
    @DisplayName("clearly-turning triples keep their old +1/-1 verdict")
    void ccwTest_preservesSignsForClearTurns() {
        OrientedPoint p1 = new OrientedPoint(0, 0, 0);
        OrientedPoint p2 = new OrientedPoint(2, 5, 0);
        OrientedPoint p3 = new OrientedPoint(5, 2, 0);
        OrientedPoint p4 = new OrientedPoint(-1, 5, 0);

        assertEquals(1, MathUtils.threePointClockwiseCounterClockwiseTest(p1, p2, p3));
        assertEquals(-1, MathUtils.threePointClockwiseCounterClockwiseTest(p1, p2, p4));
    }

    @Test
    @DisplayName("clearly-turning triples at lattice scale keep their verdict")
    void ccwTest_preservesSignsAtLatticeScale() {
        // sigma here is order EDGE_LENGTH^2 (~4900), nine orders above the tolerance.
        OrientedPoint origin = new OrientedPoint(0, 0, 0);
        OrientedPoint right = new OrientedPoint(70, 0, 0);
        OrientedPoint up = new OrientedPoint(70, 70, 0);
        OrientedPoint down = new OrientedPoint(70, -70, 0);

        assertEquals(-1, MathUtils.threePointClockwiseCounterClockwiseTest(origin, right, up));
        assertEquals(1, MathUtils.threePointClockwiseCounterClockwiseTest(origin, right, down));
    }

    @Test
    @DisplayName("exactly collinear triples report collinear")
    void ccwTest_detectsExactCollinearity() {
        OrientedPoint p1 = new OrientedPoint(0, 0, 0);
        OrientedPoint p2 = new OrientedPoint(2, 5, 0);
        OrientedPoint p5 = new OrientedPoint(-2, -5, 0);

        assertEquals(0, MathUtils.threePointClockwiseCounterClockwiseTest(p1, p2, p5));
    }

    @Test
    @DisplayName("near-collinear triples report collinear instead of the sign of the noise")
    void ccwTest_detectsNearCollinearity() {
        // Perturbed off the line by 1e-9: sigma is then order 1e-7, inside the 1e-6
        // tolerance. Before the change this returned +1 or -1 depending on which way the
        // rounding happened to fall.
        OrientedPoint p1 = new OrientedPoint(0, 0, 0);
        OrientedPoint p2 = new OrientedPoint(70, 0, 0);
        OrientedPoint p3 = new OrientedPoint(140, 1e-9, 0);

        assertEquals(0, MathUtils.threePointClockwiseCounterClockwiseTest(p1, p2, p3));
    }
}