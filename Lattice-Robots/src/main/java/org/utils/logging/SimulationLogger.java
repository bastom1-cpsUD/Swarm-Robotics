package org.utils.logging;

import com.darcarms.htmllog.HtmlLog;

import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.graphs.util.OrientedPoint;

/**
 * Owns the {@link HtmlLog} instance for one simulation run and turns
 * {@link TickRecord}s into rich per-robot HTML log entries.
 *
 * <p><b>Thread safety:</b> {@code record(...)} may be called concurrently
 * from multiple executor threads (one per robot, in the async simulation
 * model). {@link HtmlLog} itself has no internal synchronization, so all
 * actual writes are serialized on {@link #logLock}. Trajectory bookkeeping
 * uses a {@link ConcurrentHashMap} plus small per-robot synchronized blocks
 * so appends never race even when the log write for that tick gets skipped.</p>
 */
public final class SimulationLogger implements AutoCloseable {

    private final HtmlLog log;
    private volatile boolean logUnchangedRobots;
    private final Map<Integer, List<OrientedPoint>> trajectories = new ConcurrentHashMap<>();
    private final Object logLock = new Object();

    public SimulationLogger(boolean logUnchangedRobots) {
        String stamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd'T'HH-mm-ss"));
        this.log = HtmlLog.create(Path.of("logs", "sim-" + stamp), true, false);
        this.logUnchangedRobots = logUnchangedRobots;
    }

    /** Toggles whether no-op ticks (see {@link TickRecord#changed()}) are still written. */
    public void setLogUnchangedRobots(boolean value) {
        this.logUnchangedRobots = value;
    }

    public boolean isLoggingUnchangedRobots() {
        return logUnchangedRobots;
    }

    /** Directory containing this run's {@code index.html} and image assets. */
    public Path directory() {
        return log.directory();
    }

    /**
     * Writes a line naming what this run <em>is</em>, so a log can be compared against another.
     *
     * <p>Two runs are only expected to match when they were started from the same swarm at the same
     * activation period -- the period feeds {@code GeometricCycleLatticeRobot.setTickRate}, which
     * the protocol's own second-to-activation conversions read, so it is part of the input and not a
     * display preference. Recording it turns "these two logs differ" into a question with an answer.
     */
    public void note(String line) {
        synchronized (logLock) {
            log.heading(line);
        }
    }

    /**
     * Records one robot's tick. Always extends that robot's trajectory
     * (so the trail drawn later has no gaps), but only writes an HTML entry
     * if the tick actually changed something, or if
     * {@link #setLogUnchangedRobots(boolean)} has been set to {@code true}.
     */
    public void record(TickRecord rec) {
        List<OrientedPoint> trail = trajectories.computeIfAbsent(rec.robotId(), k -> new ArrayList<>());
        List<OrientedPoint> trailSnapshot;
        synchronized (trail) {
            trail.add(rec.poseAfter());
            trailSnapshot = List.copyOf(trail);
        }

        if (!rec.changed() && !logUnchangedRobots) {
            return;
        }

        synchronized (logLock) {
            try (HtmlLog.Group ignored = log.grouped()) {
                log.show(new RobotFrameView(rec, trailSnapshot));
            }
        }
    }

    @Override
    public void close() {
        log.close();
    }
}
