package org.utils;

import java.awt.geom.Point2D;

import org.graphs.util.OrientedPoint;

/**
 * An immutable free 2D vector: a direction and magnitude, with no position and no
 * heading.
 *
 * <p>This exists to separate two things the codebase used to conflate. An
 * {@link OrientedPoint} is a <em>pose</em> -- a location plus a heading -- and is what
 * a rigid body transformation acts on. A vector is neither: it has no location to
 * translate and no heading to compose. Representing one as an {@code OrientedPoint}
 * with orientation hardcoded to {@code 0}, as the old {@code MathUtils} vector helpers
 * did, makes that distinction invisible and invites passing a direction into an API
 * that will translate it.
 */
public final class Vec2 {

    public final double x;
    public final double y;

    public Vec2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * The displacement from {@code from} to {@code to}.
     * @param from the tail of the vector
     * @param to the head of the vector
     * @return the vector pointing from `from` to `to`
     */
    public static Vec2 between(Point2D from, Point2D to) {
        return new Vec2(to.getX() - from.getX(), to.getY() - from.getY());
    }

    /**
     * Reads a point's coordinates as a vector from the origin. Any heading the point
     * carries is dropped, since a vector has none.
     * @param point the point to read
     * @return the vector from the origin to `point`
     */
    public static Vec2 of(Point2D point) {
        return new Vec2(point.getX(), point.getY());
    }

    /**
     * This vector's coordinates as a point, with zero heading — the inverse of
     * {@link #of(Point2D)}.
     * @return an OrientedPoint at this vector's coordinates
     */
    public OrientedPoint asOrientedPoint() {
        return new OrientedPoint(x, y, 0);
    }

    public Vec2 plus(Vec2 other) {
        return new Vec2(x + other.x, y + other.y);
    }

    public Vec2 minus(Vec2 other) {
        return new Vec2(x - other.x, y - other.y);
    }

    public double dot(Vec2 other) {
        return x * other.x + y * other.y;
    }

    /**
     * The 2D cross product (the z-component of the 3D cross product). Signed: positive
     * when `other` is counterclockwise from this. Note the result is an <em>area</em>,
     * so it scales as the square of the input lengths -- any tolerance compared against
     * it must be sized accordingly (see {@link MathUtils#COLLINEARITY_EPSILON}).
     * @param other the second vector
     * @return the signed cross product
     */
    public double cross(Vec2 other) {
        return x * other.y - y * other.x;
    }

    /** Uses sqrt rather than hypot to stay bit-identical to the helper this replaced. */
    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * The signed angle from this vector to {@code other}, wrapped into (-pi, pi].
     * @param other the vector to measure to
     * @return the signed angle in radians
     */
    public double angleTo(Vec2 other) {
        return MathUtils.normalizeAngle(Math.atan2(cross(other), dot(other)));
    }

    @Override
    public String toString() {
        return "Vec2[x: " + x + ", y: " + y + "]";
    }
}
