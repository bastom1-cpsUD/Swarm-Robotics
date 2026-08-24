package org.robots;

/**
 * What the collision-avoidance layer is currently doing to a robot's motion.
 *
 * <p>Reported for the simulation overlay and the tick log; nothing in the algorithm
 * branches on it. Safety never depends on this value — the hard guard in
 * {@code GeometricCycleLatticeRobot.move(double)} runs at motion rate and classifies
 * nothing.
 *
 * <p>Only {@link #CLEAR} and {@link #BLOCKED} are reachable today. The remaining three
 * are declared now so that the overlay and log columns do not have to churn when the
 * tick-rate policy lands.
 */
public enum AvoidanceState {

    /** Moving toward the true target with nothing in the way. */
    CLEAR,

    /** Deliberately stationary, letting a lower-id robot pass. */
    HOLDING,

    /** Routing around a stationary obstacle rather than through it. */
    DETOURING,

    /** Stepping out of another robot's path at its request. */
    EVADING,

    /** A step was vetoed by the hard guard: the path is physically obstructed. */
    BLOCKED
}
