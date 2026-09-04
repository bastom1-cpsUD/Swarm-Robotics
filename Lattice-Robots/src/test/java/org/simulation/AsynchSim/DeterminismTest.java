package org.simulation.AsynchSim;

import java.util.List;

import org.graphs.voltage.SnubSquareVoltageGraph;
import org.graphs.voltage.VoltageGraph;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.robots.GeometricCycleLatticeRobot;

import static org.junit.jupiter.api.Assertions.*;

/**
 * The same input twice gives the same output.
 *
 * <p>This is the property the simulation did not have. There is no randomness anywhere in
 * {@code Lattice-Robots} -- no {@code Math.random}, no {@code Random}, ids sorted before scheduling,
 * insertion-ordered maps throughout -- and runs still diverged, because logical time was the wall
 * clock. Robot activations were initial delays on a thread pool sized by the machine's core count,
 * so their real order was the OS scheduler's to decide; and motion integrated {@code nanoTime}
 * deltas, so a GC pause changed how far every robot travelled. Both land on threshold tests --
 * nearest-candidate selection, and a position epsilon -- where a small difference is not a small
 * difference at all but a different robot chosen and a different formation built.
 *
 * <p>So the assertion is exact equality of full swarm state, not a tolerance. A tolerance would pass
 * precisely the runs worth failing: two that have already diverged in a decision and are still close
 * together in space.
 */
class DeterminismTest {

    private static final VoltageGraph GRAPH = SnubSquareVoltageGraph.build();

    private static final long TICK_PERIOD_MS      = 1000L;
    private static final long PROXIMITY_PERIOD_MS = 100L;
    private static final long MOTION_PERIOD_MS    = 33L;

    /** Enough robots and spacing for faces to actually form, so the run has decisions to make. */
    private static HeadlessRun run() {
        List<GeometricCycleLatticeRobot> swarm = HeadlessRun.scatteredGrid(GRAPH, 25, 45.0);
        return new HeadlessRun(swarm, TICK_PERIOD_MS, PROXIMITY_PERIOD_MS, MOTION_PERIOD_MS);
    }

    /**
     * Compared at several horizons rather than only at the end.
     *
     * <p>An early check catches a divergence in the opening moves, where the digest is small enough
     * to read; a late one catches a divergence that only becomes possible once the formation is
     * crowded and robots have several candidates to choose between. Checking only the end would
     * report the second kind as a wall of differences with no clue when it started.
     */
    @Test
    @DisplayName("two runs of the same swarm agree at every horizon")
    void twoRunsAgree() {
        HeadlessRun first = run();
        HeadlessRun second = run();

        for (long horizon : new long[] {1_000L, 10_000L, 60_000L, 300_000L}) {
            first.advance(horizon - first.nowMs());
            second.advance(horizon - second.nowMs());

            assertEquals(first.digest(), second.digest(),
                    "two runs of the same swarm diverged by logical t=" + horizon + "ms. Nothing in "
                            + "this simulation is random, so a difference here means state is being "
                            + "derived from something outside the model -- the wall clock, thread "
                            + "interleaving, or a collection's iteration order.");
        }
    }

    /**
     * Advancing in small steps and in one jump are the same run.
     *
     * <p>The property that lets the step button and the play loop share one code path: every event
     * executes at its own due time regardless of how coarsely the caller advances, so a stalled play
     * loop that catches up in a single long {@code advanceTo} produces the run it would have
     * produced smoothly. Without this, a slow machine would not merely look slow, it would simulate
     * something else.
     */
    @Test
    @DisplayName("advancing in small steps equals advancing in one jump")
    void stepGranularityDoesNotMatter() {
        HeadlessRun stepped = run();
        HeadlessRun jumped = run();

        for (int i = 0; i < 60; i++) {
            stepped.advance(TICK_PERIOD_MS);
        }
        jumped.advance(60 * TICK_PERIOD_MS);

        assertEquals(stepped.nowMs(), jumped.nowMs(), "scenario error: the two runs ended at "
                + "different logical times, so this is not comparing the same interval");
        assertEquals(jumped.digest(), stepped.digest(),
                "sixty one-period steps and one sixty-period jump produced different states. Events "
                        + "must run at their own due times, not batched at the target -- otherwise "
                        + "the step button and the play loop are simulating different systems, and a "
                        + "play loop that falls behind changes the run rather than just its pace.");
    }

    /**
     * The digest notices the thing that used to vary.
     *
     * <p>Without this the two tests above are unfalsifiable -- a run that would never have diverged
     * passes them whether or not the digest reads anything at all. So one run is given a motion
     * frame a few percent longer, which is exactly what the old
     * {@code dt = (nanoTime - last) / 1e9} produced whenever a GC pause or a repaint stretched a
     * frame. If the digest cannot see that, it cannot see the bug this work removed.
     *
     * <p>Note this is a coarser perturbation than a single bit, and deliberately so: robots snap
     * onto their target pose on arrival, which erases sub-ULP differences at every settled robot, so
     * a one-ULP test would be asserting something the model does not actually preserve. Frame-length
     * jitter is both the realistic failure and one that survives to be measured.
     */
    @Test
    @DisplayName("the digest is sensitive to motion-frame jitter")
    void digestNoticesFrameJitter() {
        List<GeometricCycleLatticeRobot> swarmA = HeadlessRun.scatteredGrid(GRAPH, 25, 45.0);
        List<GeometricCycleLatticeRobot> swarmB = HeadlessRun.scatteredGrid(GRAPH, 25, 45.0);

        HeadlessRun reference = new HeadlessRun(swarmA, TICK_PERIOD_MS, PROXIMITY_PERIOD_MS,
                MOTION_PERIOD_MS);
        HeadlessRun jittered = new HeadlessRun(swarmB, TICK_PERIOD_MS, PROXIMITY_PERIOD_MS,
                MOTION_PERIOD_MS + 1);

        reference.advance(60_000L);
        jittered.advance(60_000L);

        assertNotEquals(reference.digest(), jittered.digest(),
                "a longer motion frame left the digest identical, so the digest is not measuring "
                        + "what these tests claim it measures and both of them pass vacuously. "
                        + "Frame length is precisely what the old wall-clock dt varied.");
    }
}
