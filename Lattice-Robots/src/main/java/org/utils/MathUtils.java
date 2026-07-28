package org.utils;

import org.graphs.util.OrientedPoint;

/**
 * Shared numeric and angular tolerance helpers.
 *
 * <p>This is the single source of truth for "are these effectively equal?" decisions
 * across the project. {@link #EPSILON} is the one convergence tolerance: the motion
 * models (deciding when a move is complete), the arrival gate in
 * {@code GeometricCycleLatticeRobot} (deciding when a robot is at its assigned pose),
 * and the root-closing check in {@code CyclebuilderComms} all compare against a
 * provably-stable reference (a static root/stable anchor, or a parent that has itself
 * already frozen to its exact ideal target) and so may safely share this tight
 * tolerance without risk of chasing a moving target forever.
 *
 * <p>{@link #REASSIGNMENT_POSITION_EPSILON} / {@link #REASSIGNMENT_ANGLE_EPSILON} are
 * a deliberately separate, looser pair, used only by
 * {@code CyclebuilderComms.checkAssignmentForCurrentPosition}. That check validates an
 * incoming assignment against a robot's own *possibly still in-transit* current pose
 * (a robot can be mid-transit toward its previous target when a new
 * {@code PositioningMessage} arrives — {@code hasBeenAssigned} is never cleared by
 * {@code reset()}), so it is not a convergence check and must not be tightened to
 * {@link #EPSILON}: doing so risks spuriously rejecting legitimate re-assignments
 * before the robot's own motion has caught up. See DCEL-Implementation-Plan.md-style
 * reasoning in {@code CyclebuilderComms} for the full argument.
 */
public final class MathUtils {

    /**
     * Tolerance below which a scalar — a distance, or an angular difference in radians —
     * is treated as zero. Shared convergence epsilon for motion completion, at-position
     * detection, and cycle-closing detection, all of which compare against a
     * provably-stable reference. See class javadoc.
     */
    public static final double EPSILON = 1e-3;

    /**
     * Linear-distance tolerance for validating a re-assignment against a robot's own
     * current, possibly still-converging position. Not a convergence tolerance — see
     * class javadoc. Deliberately looser than {@link #EPSILON}.
     */
    public static final double REASSIGNMENT_POSITION_EPSILON = 1e-1;

    /**
     * Angular tolerance, in radians, for the same re-assignment check. Sized
     * independently from {@link #REASSIGNMENT_POSITION_EPSILON} — that constant is a
     * distance tolerance, this one an angle tolerance, and the two units should never
     * be conflated under one raw value. {@code 0.1} rad (~5.7°) is roughly one
     * motion-tick's worth of rotation at {@code TimeStepDiffDrive.MAX_ANGULAR_SPEED};
     * re-check this against actual tick timing if that constant changes.
     */
    public static final double REASSIGNMENT_ANGLE_EPSILON = 1e-1;

    private MathUtils() {
        // Utility class
    }

    /**
     * Determines whether a value is within {@link #EPSILON} of zero.
     * @param value the value to test (a distance, or an angular difference in radians)
     * @return true if |value| &lt; EPSILON
     */
    public static boolean isZero(double value) {
        return isZero(value, EPSILON);
    }

    /**
     * Determines whether a value is within a caller-supplied tolerance of zero. Provided
     * for callers that need a finer or coarser tolerance than the shared {@link #EPSILON}
     * (for example, a continuous-time motion model running at 1e-9).
     * @param value the value to test
     * @param epsilon the tolerance to compare against
     * @return true if |value| &lt; epsilon
     */
    public static boolean isZero(double value, double epsilon) {
        return Math.abs(value) < epsilon;
    }

    /**
     * Determines whether two scalars are equal within {@link #EPSILON}.
     * @param a the first value
     * @param b the second value
     * @return true if the two values are within EPSILON of each other
     */
    public static boolean approxEquals(double a, double b) {
        return isZero(a - b);
    }

    public static boolean approxEquals(double a, double b, double epsilon) {
        return isZero(a - b, epsilon);
    }

    /**
     * Normalizes an angle to the range (-pi, pi].
     * @param angle the angle in radians
     * @return the equivalent angle wrapped into (-pi, pi]
     */
    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle <= -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Returns the smallest signed angular difference (a - b), wrapped into (-pi, pi].
     * Use this instead of a raw subtraction so headings on either side of the +/-pi
     * boundary are compared correctly.
     * @param a the first angle in radians
     * @param b the second angle in radians
     * @return the wrapped difference a - b in (-pi, pi]
     */
    public static double angleDifference(double a, double b) {
        return normalizeAngle(a - b);
    }

    /**
     * Determines whether two angles represent effectively the same heading, accounting
     * for wraparound and using the shared {@link #EPSILON} tolerance.
     * @param a the first angle in radians
     * @param b the second angle in radians
     * @return true if the angles are within EPSILON after wrapping
     */
    public static boolean anglesEqual(double a, double b) {
        return isZero(angleDifference(a, b));
    }

    /**
     * Determines if an ordered sequence of three points make a turn to the right (clockwise direction)
     * @param p1 the first point
     * @param p2 the second point
     * @param p3 the third point
     * @return negative if counterclockwise, 0 if co-linear, positive if clockwise
     */
    public static int threePointClockwiseCounterClockwiseTest(OrientedPoint p1, OrientedPoint p2, OrientedPoint p3) {
        OrientedPoint v1 = new OrientedPoint(p2.getX() - p1.getX(), p2.getY() - p1.getY(), 0);
        OrientedPoint v2 = new OrientedPoint(p3.getX() - p2.getX(), p3.getY() - p2.getY(), 0);

        double sigma = -1 * crossProduct(v1, v2);

        if(sigma == 0.0) {
            return 0;
        }

        return sigma > 0.0 ? 1 : -1;
    }

    public static double angleBetween(OrientedPoint v1, OrientedPoint v2) {
        return normalizeAngle(Math.atan2(crossProduct(v1, v2), dotProduct(v1, v2)));
    }

    public static OrientedPoint vectorSum(OrientedPoint v1, OrientedPoint v2) {
        return new OrientedPoint(v1.getX() + v2.getX(), v1.getY() + v2.getY(), 0);
    }

    public static OrientedPoint vectorBetween(OrientedPoint p1, OrientedPoint p2) {
        return new OrientedPoint(p2.getX() - p1.getX(), p2.getY() - p1.getY(), 0);
    }

    public static double dotProduct(OrientedPoint v1, OrientedPoint v2) {
        return v1.getX() * v2.getX() + v1.getY() * v2.getY();
    }

    public static double crossProduct(OrientedPoint v1, OrientedPoint v2) {
        return v1.getX() * v2.getY() - v1.getY() * v2.getX();
    }

    public static double magnitude(OrientedPoint v1) {
        return Math.sqrt(v1.getX() * v1.getX() + v1.getY() * v1.getY());
    }

    public static void main(String[] args) {
        OrientedPoint p1 = new OrientedPoint(0,0,0);
        OrientedPoint p2 = new OrientedPoint(2, 5, 0);
        OrientedPoint p3 = new OrientedPoint(5, 2, 0);
        OrientedPoint p4 = new OrientedPoint(-1, 5, 0);
        OrientedPoint p5 = new OrientedPoint(-2, -5, 0);
        
        //Check if positive for clockwise turn    
        System.out.println(threePointClockwiseCounterClockwiseTest(p1, p2, p3));

        //Check if negative for counterclockwise turn
        System.out.println(threePointClockwiseCounterClockwiseTest(p1, p2, p4));

        //Check if false for colinear
        System.out.println(threePointClockwiseCounterClockwiseTest(p1, p2, p5));
    }
}