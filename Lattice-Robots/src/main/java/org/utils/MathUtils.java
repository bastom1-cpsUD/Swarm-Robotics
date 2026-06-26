package org.utils;

/**
 * Shared numeric and angular tolerance helpers.
 *
 * <p>This is the single source of truth for "are these effectively equal?" decisions
 * across the project. Both the motion models (deciding when a move is complete) and
 * {@code GeometricCycleLatticeRobot} (deciding when a robot is at its assigned pose)
 * must route their tolerance checks through here so the two definitions never drift
 * apart. A mismatch between the motion model's convergence tolerance and the broadcast
 * gate's "at position" check is precisely what caused robots to physically stop while
 * never being marked as arrived.
 */
public final class MathUtils {

    /**
     * Tolerance below which a scalar — a distance, or an angular difference in radians —
     * is treated as zero. Shared convergence epsilon for both motion completion and
     * at-position detection.
     */
    public static final double EPSILON = 1e-3;

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
}