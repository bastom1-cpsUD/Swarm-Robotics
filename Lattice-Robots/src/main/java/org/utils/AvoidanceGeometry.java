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
        OrientedPoint closest = closestPointOnSegment(a, b, p);
        return Math.hypot(p.x - closest.x, p.y - closest.y);
    }

    /**
     * The point of segment {@code [a, b]} nearest to {@code p}.
     *
     * <p>The returned orientation is {@code a}'s and carries no meaning — this is a
     * position query, and {@link OrientedPoint} is simply the project's point type.
     */
    public static OrientedPoint closestPointOnSegment(OrientedPoint a, OrientedPoint b, OrientedPoint p) {
        double abx = b.x - a.x;
        double aby = b.y - a.y;
        double lengthSquared = abx * abx + aby * aby;

        // Degenerate segment: both endpoints coincide, so the "segment" is a point.
        // Guarded exactly rather than by tolerance -- only a true zero divides badly here,
        // and for a merely tiny length the clamp below already yields the right answer.
        if (lengthSquared <= 0.0) {
            return new OrientedPoint(a.x, a.y, a.orientation);
        }

        double t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / lengthSquared;
        t = Math.max(0.0, Math.min(1.0, t));

        return new OrientedPoint(a.x + t * abx, a.y + t * aby, a.orientation);
    }

    /**
     * The point {@code clearance} from a corridor, directly <em>outward</em> from the
     * corridor's nearest point to {@code from}.
     *
     * <p>This is the escape a robot should take when asked to get out of someone's way:
     * it always increases the distance to every point of the corridor, so it can never
     * move the evader toward the robot that asked, and it never crosses the corridor to
     * reach the other side.
     *
     * <p>Offsetting from the nearest corridor point rather than perpendicularly from the
     * foot is what makes that true at the ends. When {@code from} lies beyond an endpoint
     * the foot clamps to it, and a perpendicular offset from that clamped foot carries a
     * large along-corridor component — which points straight back at the requester when
     * the evader is past the far end.
     *
     * @return the escape point, or null when {@code from} is already at least
     *         {@code clearance} away (nothing to do), or is exactly on the corridor so
     *         that no outward direction is defined — the caller must pick a side itself
     */
    public static OrientedPoint escapeFromCorridor(OrientedPoint from, OrientedPoint corridorStart,
                                                   OrientedPoint corridorEnd, double clearance) {
        OrientedPoint nearest = closestPointOnSegment(corridorStart, corridorEnd, from);

        double dx = from.x - nearest.x;
        double dy = from.y - nearest.y;
        double distance = Math.hypot(dx, dy);

        if (distance <= 0.0 || distance >= clearance) {
            return null;
        }

        double ux = dx / distance;
        double uy = dy / distance;
        return new OrientedPoint(nearest.x + ux * clearance,
                                 nearest.y + uy * clearance,
                                 Math.atan2(uy, ux));
    }

    /**
     * A point {@code clearance} to one side of a corridor, level with wherever
     * {@code from} currently stands along it.
     *
     * <p>Used by a robot asked to get out of another's way. Stepping <em>perpendicular to
     * the corridor</em> rather than merely away from the requester is the whole point:
     * moving away from the requester is monotone but frequently useless, because it can
     * push the evader further along the very line the requester wants to travel.
     *
     * @param sign +1 for the left-hand normal of {@code corridorStart -> corridorEnd},
     *             -1 for the right
     */
    public static OrientedPoint sidestepOffCorridor(OrientedPoint from, OrientedPoint corridorStart,
                                                    OrientedPoint corridorEnd, double clearance, int sign) {
        OrientedPoint foot = closestPointOnSegment(corridorStart, corridorEnd, from);

        double dx = corridorEnd.x - corridorStart.x;
        double dy = corridorEnd.y - corridorStart.y;
        double length = Math.hypot(dx, dy);

        double nx, ny;
        if (length <= 0.0) {
            // Degenerate corridor -- the requester declares it is already where it wants
            // to be, so there is no direction to be perpendicular to. Back straight off it.
            double ax = from.x - corridorStart.x;
            double ay = from.y - corridorStart.y;
            double away = Math.hypot(ax, ay);
            if (away <= 0.0) {
                nx = 1.0;
                ny = 0.0;
            } else {
                nx = ax / away;
                ny = ay / away;
            }
        } else {
            nx = -dy / length;
            ny =  dx / length;
        }

        return new OrientedPoint(foot.x + sign * nx * clearance,
                                 foot.y + sign * ny * clearance,
                                 Math.atan2(sign * ny, sign * nx));
    }

    /**
     * Whether an obstacle stands in the way of travelling from {@code from} to
     * {@code target}.
     *
     * @param from     where the robot is now
     * @param target   where it is trying to get to
     * @param obstacle the other body's centre
     * @param keepOut  centre-to-centre separation below which the two bodies collide
     * @return true if the obstacle intrudes on the straight path
     */
    public static boolean blocks(OrientedPoint from, OrientedPoint target,
                                 OrientedPoint obstacle, double keepOut) {
        return distancePointToSegment(from, target, obstacle) < keepOut;
    }

    /**
     * A one-tick aim point on the tangent ray from {@code from} to the keep-out circle
     * around {@code obstacle}.
     *
     * <p>Aiming along the <em>tangent ray</em> rather than stepping onto the circle is
     * what makes this safe for any {@code reach}: every point on a tangent ray is at
     * least the radius from the centre, so the returned waypoint is provably no closer
     * than {@code keepOut} to the obstacle and can never itself be a collision. The hard
     * guard can therefore only ever veto transient frames, never the destination.
     *
     * <p>Re-planning each tick with {@code reach} of one tick's travel walks the robot
     * along successive tangent rays, grazing the circle — "slide along the bubble" is an
     * emergent consequence rather than something programmed. Because no path is stored,
     * drift or a vetoed frame is simply absorbed by the next tick's recomputation.
     *
     * <p>Callers must route the {@code dist <= keepOut} case to
     * {@link #radialEscape} instead: there the arcsine saturates and both candidates
     * degenerate into pure sidesteps, which do not increase separation and so would be
     * refused by the guard.
     *
     * @param sign +1 to pass counter-clockwise about the obstacle, -1 for clockwise
     */
    public static OrientedPoint tangentWaypoint(OrientedPoint from, OrientedPoint obstacle,
                                                double keepOut, double reach, int sign) {
        double vx = obstacle.x - from.x;
        double vy = obstacle.y - from.y;
        double dist = Math.hypot(vx, vy);

        double half = (dist <= 0.0)
                ? Math.PI / 2.0
                : Math.asin(Math.min(1.0, keepOut / dist));
        double theta = Math.atan2(vy, vx) + sign * half;

        return new OrientedPoint(from.x + reach * Math.cos(theta),
                                 from.y + reach * Math.sin(theta),
                                 theta);
    }

    /**
     * A straight-away step directly out of an obstacle's keep-out circle, for the case
     * where the robot is already inside it.
     *
     * <p>This is what makes the escape argument constructive. The hard guard only ever
     * <em>permits</em> a separating step; something has to actually <em>choose</em> one,
     * and a robot aiming at its lattice target may well be aiming straight into the
     * obstacle. Because this strictly increases separation it can never be vetoed.
     *
     * @param robotId used only in the exactly-coincident case, to pick a direction from
     *                the golden angle so that two stacked robots diverge deterministically
     *                rather than both choosing the same way out
     */
    public static OrientedPoint radialEscape(OrientedPoint from, OrientedPoint obstacle,
                                             double reach, int robotId) {
        double dx = from.x - obstacle.x;
        double dy = from.y - obstacle.y;
        double dist = Math.hypot(dx, dy);

        double ux, uy;
        if (dist <= 0.0) {
            double angle = robotId * 2.39996323;   // golden angle, radians
            ux = Math.cos(angle);
            uy = Math.sin(angle);
        } else {
            ux = dx / dist;
            uy = dy / dist;
        }

        return new OrientedPoint(from.x + reach * ux,
                                 from.y + reach * uy,
                                 Math.atan2(uy, ux));
    }

    /**
     * Whether a waypoint keeps the robot inside its parent's communication range.
     *
     * <p>No per-tick margin is needed and none is applied: the disk of radius
     * {@code tetherRadius} about the parent is convex and a tick's motion is a straight
     * segment, so both endpoints being inside implies the whole path is. The only motion
     * that would need a margin is the parent's own, and a cycleBuilder's parent is
     * provably parked at the moment it hands out an assignment.
     *
     * @param parent the parent's pose, or null when there is no parent to stay near —
     *               a root, a stable, or an unassigned robot, none of which have a link
     *               to preserve
     * @return true if the waypoint is acceptable
     */
    public static boolean withinTether(OrientedPoint waypoint, OrientedPoint parent,
                                       double tetherRadius) {
        if (parent == null) {
            return true;
        }
        return waypoint.distance(parent) <= tetherRadius;
    }
}
