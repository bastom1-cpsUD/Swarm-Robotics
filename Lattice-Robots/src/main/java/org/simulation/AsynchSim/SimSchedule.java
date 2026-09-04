package org.simulation.AsynchSim;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

/**
 * The simulation's clock and running order, as an explicit discrete-event queue.
 *
 * <p><strong>This exists because the simulation used to be timed by the wall clock, and so could
 * not repeat a run.</strong> Three periodic tasks -- proximity checks, robot activations and motion
 * frames -- were handed to a {@code ScheduledThreadPoolExecutor} sized by the machine's core count,
 * and the order they actually interleaved in was whatever the OS scheduler produced. Motion made it
 * worse by integrating <em>measured</em> elapsed time ({@code System.nanoTime()} deltas), so a GC
 * pause or a slow repaint changed how far every robot travelled. Distance decides things here --
 * {@code findBestNeighborForEdge} picks the nearest candidate and
 * {@code checkAssignmentForCurrentPosition} tests against an epsilon -- so drift of a few units
 * changed which robot was chosen, and one different choice changes the formation.
 *
 * <p>So logical time is a number now, not a reading. Everything is scheduled against
 * {@link #nowMs()}, {@link #advanceTo(long)} runs events in a total order, and the wall clock is
 * left with the one job it can do reproducibly: deciding how fast a human watches. A run is a pure
 * function of its robots, its tick period, and the ordering rule below.
 *
 * <p><strong>The ordering rule is the load-bearing part.</strong> Events are totally ordered by
 * {@code (dueMs, kind, robotId, seq)}, and the order between the kinds is
 * {@code PROXIMITY < TICK < MOTION} -- neighbours refreshed, then robots decide, then robots move.
 * That is not a new invention: it is the order {@code AsyncRobotPanel.singleStep()} has always used,
 * promoted from an implementation detail of the step button into the rule the whole simulation runs
 * by. Ties between robot activations break by robot id, which is what the staggered start offsets
 * were always trying to express.
 *
 * <p>Periods and offsets are copied from the old scheduling verbatim, so the <em>character</em> of
 * the asynchrony is unchanged -- robots still activate at staggered phases of a shared period, and
 * still observe each other mid-flight. What changes is only that the stagger is now honoured exactly
 * instead of approximately.
 *
 * <p>Not thread-safe, deliberately. One mutator is the point: the races it removes -- robots ticking
 * concurrently under a shared read lock while appending to each other's inboxes -- were a source of
 * divergence in their own right. Callers that share state with a UI thread should guard
 * {@link #advanceTo(long)} externally.
 */
public final class SimSchedule {

    /**
     * What an event does, in the order simultaneous events do it.
     *
     * <p>Declaration order <em>is</em> the tie-break: {@link Enum#ordinal()} feeds the comparator.
     * Reordering these constants changes simulation results.
     */
    public enum Kind {
        /** Rebuild every robot's neighbour list from current positions. */
        PROXIMITY,
        /** One robot's activation: read a message, decide, broadcast. */
        TICK,
        /** One motion frame: every robot advances toward its assigned pose. */
        MOTION
    }

    /** What a schedule does when an event comes due. Implemented by the panel and by headless runs. */
    public interface Handler {
        void proximity();
        void tick(int robotId);
        /** @param dtSeconds a fixed step, never a measured elapsed time */
        void motion(double dtSeconds);
    }

    /**
     * {@code robotId} is {@code NO_ROBOT} for the two swarm-wide kinds. {@code seq} is a stable
     * final tie-break; with one event of each kind in flight at a time the triple ahead of it is
     * already unique, but a {@link PriorityQueue} is not a stable sort and a future fourth kind
     * should not be able to reintroduce ambiguity by accident.
     */
    private record Event(long dueMs, Kind kind, int robotId, long seq) {}

    private static final int NO_ROBOT = -1;

    private static final Comparator<Event> ORDER = Comparator
            .comparingLong(Event::dueMs)
            .thenComparing(Event::kind)
            .thenComparingInt(Event::robotId)
            .thenComparingLong(Event::seq);

    private final Handler handler;
    private final long proximityPeriodMs;
    private final long motionPeriodMs;
    private final double motionDtSeconds;

    private final PriorityQueue<Event> queue = new PriorityQueue<>(ORDER);
    private long nowMs;
    private long seq;
    private long tickPeriodMs;

    /**
     * @param tickPeriodMs      how often each robot activates. Part of a run's identity: the same
     *                          robots at a different period are a different run, because
     *                          {@code GeometricCycleLatticeRobot.setTickRate} derives from it.
     * @param motionPeriodMs    the motion frame period, which is also the fixed {@code dt} handed to
     *                          {@link Handler#motion(double)}
     */
    public SimSchedule(Handler handler, long tickPeriodMs, long proximityPeriodMs, long motionPeriodMs) {
        this.handler = handler;
        this.tickPeriodMs = tickPeriodMs;
        this.proximityPeriodMs = proximityPeriodMs;
        this.motionPeriodMs = motionPeriodMs;
        this.motionDtSeconds = motionPeriodMs / 1000.0;
    }

    /** Logical milliseconds since the run began. */
    public long nowMs() {
        return nowMs;
    }

    public long tickPeriodMs() {
        return tickPeriodMs;
    }

    /**
     * Seeds the queue for these robots, discarding whatever was pending.
     *
     * <p>Offsets reproduce the old {@code scheduleRobotTasks}: robot <em>i</em> of <em>n</em>, taken
     * in ascending id order, first activates at {@code period + (i/n) * period} and every
     * {@code period} after. The swarm-wide events start immediately, so the first thing any run does
     * is establish neighbours.
     *
     * <p>Logical time is <strong>not</strong> reset. Re-arming mid-run -- which is what changing the
     * speed slider does -- moves the stagger onto the new period from wherever the run has got to,
     * rather than pretending it started over.
     */
    public void arm(List<Integer> robotIds, long tickPeriodMs) {
        this.tickPeriodMs = tickPeriodMs;
        queue.clear();

        push(nowMs, Kind.PROXIMITY, NO_ROBOT);
        push(nowMs, Kind.MOTION, NO_ROBOT);

        List<Integer> ids = new ArrayList<>(robotIds);
        ids.sort(Integer::compare);
        int n = ids.size();
        for (int i = 0; i < n; i++) {
            long offset = tickPeriodMs + (long) ((double) i / n * tickPeriodMs);
            push(nowMs + offset, Kind.TICK, ids.get(i));
        }
    }

    /** Whether anything is scheduled. False before {@link #arm} and after arming an empty swarm. */
    public boolean isArmed() {
        return !queue.isEmpty();
    }

    /**
     * Empties the queue and returns logical time to zero.
     *
     * <p>For starting over, as distinct from {@link #arm}'s re-phasing of a run in progress. The
     * clock has to go back too: leaving it where it stopped would give the next run a different
     * stagger phase from the first, so the two would not be comparable even from identical robots.
     */
    public void reset() {
        queue.clear();
        nowMs = 0;
        seq = 0;
    }

    /**
     * Runs every event due at or before {@code targetMs}, then parks logical time there.
     *
     * <p>Each event is executed at exactly its own due time, so a caller that advances in large
     * jumps gets the same sequence as one that advances a millisecond at a time. That is what lets
     * the step button and the play loop share this method and agree: stepping is
     * {@code advanceTo(nowMs + tickPeriodMs)} and playing is the same call with a target read off
     * the wall clock.
     */
    public void advanceTo(long targetMs) {
        while (!queue.isEmpty() && queue.peek().dueMs() <= targetMs) {
            Event event = queue.poll();
            nowMs = event.dueMs();
            dispatch(event);
            push(event.dueMs() + periodOf(event.kind()), event.kind(), event.robotId());
        }
        if (targetMs > nowMs) {
            nowMs = targetMs;
        }
    }

    private void dispatch(Event event) {
        switch (event.kind()) {
            case PROXIMITY -> handler.proximity();
            case TICK      -> handler.tick(event.robotId());
            case MOTION    -> handler.motion(motionDtSeconds);
        }
    }

    private long periodOf(Kind kind) {
        return switch (kind) {
            case PROXIMITY -> proximityPeriodMs;
            case TICK      -> tickPeriodMs;
            case MOTION    -> motionPeriodMs;
        };
    }

    private void push(long dueMs, Kind kind, int robotId) {
        queue.add(new Event(dueMs, kind, robotId, seq++));
    }
}
