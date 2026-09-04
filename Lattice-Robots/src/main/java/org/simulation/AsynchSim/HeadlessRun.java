package org.simulation.AsynchSim;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.graphs.voltage.VoltageGraph;
import org.robots.GeometricCycleLatticeRobot;

/**
 * A simulation with no window, so a run can be repeated, measured and compared.
 *
 * <p>Two jobs. It is what {@code DeterminismTest} drives to assert that the same input twice gives
 * the same output, which is not a claim the panel can make about itself. And it is a batch harness:
 * convergence probes over a few hundred logical seconds run in a fraction of a second here, where in
 * the panel they run in real time and cannot be scripted.
 *
 * <p>Deliberately not a subclass or a stripped copy of {@link AsyncRobotPanel}. It shares the part
 * that decides what happens -- {@link SimSchedule} and the three event bodies -- and shares nothing
 * of the part that draws it. A headless mode built by disabling rendering inside the panel would
 * keep the panel's lifecycle, its lock and its logger in the loop, and each of those is a way for
 * the two to drift apart without anything noticing.
 */
public final class HeadlessRun implements SimSchedule.Handler {

    private final Map<Integer, GeometricCycleLatticeRobot> robots = new LinkedHashMap<>();
    private final SimSchedule schedule;
    private final Map<Integer, Integer> tickCounts = new LinkedHashMap<>();
    private final double commRange;

    /**
     * @param tickPeriodMs the activation period, in the same units the panel's speed slider sets.
     *                     Part of the run's identity: the protocol converts seconds to activations
     *                     through {@code GeometricCycleLatticeRobot.setTickRate}, so two runs at
     *                     different periods are different runs and are not expected to agree.
     */
    public HeadlessRun(List<GeometricCycleLatticeRobot> swarm, long tickPeriodMs,
                       long proximityPeriodMs, long motionPeriodMs) {
        for (GeometricCycleLatticeRobot robot : swarm) {
            robots.put(robot.getRobotId(), robot);
            tickCounts.put(robot.getRobotId(), 0);
        }
        this.commRange = GeometricCycleLatticeRobot.COMM_RANGE;
        this.schedule = new SimSchedule(this, tickPeriodMs, proximityPeriodMs, motionPeriodMs);
        this.schedule.arm(new ArrayList<>(robots.keySet()), tickPeriodMs);
    }

    /** Runs forward by {@code millis} of logical time. */
    public void advance(long millis) {
        schedule.advanceTo(schedule.nowMs() + millis);
    }

    public long nowMs() {
        return schedule.nowMs();
    }

    public List<GeometricCycleLatticeRobot> robots() {
        return new ArrayList<>(robots.values());
    }

    /** The whole swarm's state, for comparing one run against another. */
    public String digest() {
        return SwarmDigest.of(robots.values());
    }

    // ------------------------------------------------------------------
    // Event bodies. Same three actions as the panel's, minus logging and locking.
    // ------------------------------------------------------------------

    @Override
    public void proximity() {
        for (GeometricCycleLatticeRobot robot : robots.values()) {
            robot.clearNeighbors();
            for (GeometricCycleLatticeRobot other : robots.values()) {
                if (robot.getRobotId() == other.getRobotId()) continue;
                if (robot.getPosition().distance(other.getPosition()) <= commRange) {
                    robot.addNeighbor(other);
                }
            }
        }
    }

    @Override
    public void tick(int robotId) {
        GeometricCycleLatticeRobot robot = robots.get(robotId);
        if (robot == null) return;
        int tick = tickCounts.merge(robotId, 1, Integer::sum);
        robot.executeTimeStep(schedule.tickPeriodMs() / 1000.0, tick);
    }

    @Override
    public void motion(double dtSeconds) {
        for (GeometricCycleLatticeRobot robot : robots.values()) {
            robot.move(dtSeconds);
        }
    }

    /**
     * Lays out {@code count} robots on a square grid of the given pitch, ids ascending in row-major
     * order, with robot 0 promoted to the primary root.
     *
     * <p>A fixture rather than a scenario: it exists to be identical every time it is called, which
     * is all a reproducibility test needs from its starting conditions.
     */
    public static List<GeometricCycleLatticeRobot> scatteredGrid(VoltageGraph graph, int count,
                                                                 double pitch) {
        int perRow = (int) Math.ceil(Math.sqrt(count));
        List<GeometricCycleLatticeRobot> swarm = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            double x = (i % perRow) * pitch;
            double y = (i / perRow) * pitch;
            swarm.add(new GeometricCycleLatticeRobot(
                    i, new org.graphs.util.OrientedPoint(x, y, 0), graph));
        }
        swarm.get(0).promoteToPrimaryRoot();
        return swarm;
    }
}
