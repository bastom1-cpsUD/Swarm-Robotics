package org.utils;

import org.graphs.util.OrientedPoint;

/**
 * Stateless planar-obstacle geometry for the physical collision-avoidance layer.
 *
 * <p>Deliberately separate from {@link MathUtils}, which declares itself the single
 * source of truth for <em>tolerances and angular helpers</em>. These are domain routines
 * about bodies and paths, not about "are these effectively equal?", so they live apart —
 * but they build on {@code MathUtils} rather than restating any of its tolerances.
 *
 * <p>Nothing here knows about robots, comms, or the lattice. Every method is a pure
 * function of the points handed to it, which is what makes the whole set unit-testable
 * without standing up a simulation.
 */
public final class AvoidanceGeometry {

    private AvoidanceGeometry() {
        // Utility class
    }

    /**
     * Whether a candidate step may be committed, with respect to one obstacle.
     *
     * <p>Rejects a step only when it would <strong>both</strong> end inside the obstacle's
     * keep-out radius <strong>and</strong> reduce the separation from it. The second
     * conjunct is what makes an existing overlap escapable rather than a permanent freeze:
     * robots can start overlapping (the shipped 100-robot dataset contains one such pair,
     * robots 20 and 58 at 29.07 apart) or be dragged into overlap with the mouse, and a
     * rule that vetoed every step ending under {@code keepOut} would trap them there for
     * the rest of the run.
     *
     * <p>The {@link MathUtils#EPSILON} slack on the comparison is load-bearing, not
     * cosmetic. A robot in {@code ROTATE_TO_POINT} or {@code ROTATE_TO_FINAL} translates
     * zero distance, so {@code after == before} and floating-point noise alone could read
     * a pure rotation as an approach — freezing a robot that is only turning. Rotation
     * cannot move a circle, so it must never be vetoed.
     *
     * @param from     the pose before the step
     * @param to       the candidate pose after the step
     * @param obstacle the other body's centre
     * @param keepOut  centre-to-centre separation below which the two bodies collide
     * @return true if the step may be committed
     */
    public static boolean permitsStep(OrientedPoint from, OrientedPoint to,
                                      OrientedPoint obstacle, double keepOut) {
        double after = to.distance(obstacle);
        if (after >= keepOut) {
            return true;
        }
        return after >= from.distance(obstacle) - MathUtils.EPSILON;
    }

    /**
     * Shortest distance from a point to the <em>segment</em> {@code [a, b]} — not to the
     * infinite line through them, and not to a ray.
     *
     * <p>The segment/ray distinction is the whole point: an obstacle sitting behind a
     * robot, or beyond its target, is not in its way, and treating the path as a ray would
     * report it as blocking forever.
     *
     * @param a segment start
     * @param b segment end
     * @param p the point to measure from
     * @return the distance from {@code p} to the nearest point of {@code [a, b]}
     */
    public static double distancePointToSegment(OrientedPoint a, OrientedPoint b, OrientedPoint p) {
        double abx = b.x - a.x;
        double aby = b.y - a.y;
        double lengthSquared = abx * abx + aby * aby;

        // Degenerate segment: both endpoints coincide, so the "segment" is a point.
        // Guarded exactly rather than by tolerance -- only a true zero divides badly here,
        // and for a merely tiny length the clamp below already yields the right answer.
        if (lengthSquared <= 0.0) {
            return p.distance(a);
        }

        double t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / lengthSquared;
        t = Math.max(0.0, Math.min(1.0, t));

        double closestX = a.x + t * abx;
        double closestY = a.y + t * aby;

        return Math.hypot(p.x - closestX, p.y - closestY);
    }
}
