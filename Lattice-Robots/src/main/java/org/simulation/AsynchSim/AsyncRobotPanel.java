package org.simulation.AsynchSim;

import org.communicationModels.cycleBuildingComms.CycleRole;
import org.graphs.util.OrientedPoint;
import org.utils.logging.SimulationLogger;
import org.utils.logging.TickRecord;
import org.robots.GeometricCycleLatticeRobot;
import org.utils.RobotDataIO;

import javax.swing.*;
import javax.swing.Timer;
import javax.swing.border.EmptyBorder;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.time.LocalDateTime;
import java.util.*;
import java.util.List;
import java.util.concurrent.*;
import java.util.concurrent.locks.ReentrantReadWriteLock;

/**
 * Asynchronous simulation panel for the Song & O'Kane cycle-building algorithm.
 *
 * <h3>Activation model</h3>
 * Each robot activates at a fixed period with a staggered offset:
 * <pre>
 *   delay_i = period + (sortedIndex_i * period) / robotCount
 * </pre>
 * This spaces activations evenly across one full period, giving genuine asynchrony without
 * randomness. {@link SimSchedule} owns that model and the clock behind it.
 *
 * <h3>Reproducibility</h3>
 * <strong>This claim used to be made here and was false.</strong> The stagger was expressed as
 * initial delays on a {@code ScheduledThreadPoolExecutor} sized by the machine's core count, so what
 * actually ran, and in what order, was the OS scheduler's decision; and motion integrated measured
 * elapsed time, so robots travelled different distances on every run. Both fed the nearest-candidate
 * and position-epsilon tests that decide whether a face gets built, and the same input produced
 * visibly different formations.
 *
 * <p>Logical time is now a counter rather than a reading, and the wall clock only paces playback --
 * see {@link SimSchedule}. A run is reproducible given the same robots and the same
 * <em>period</em>, which the speed slider still sets; moving it mid-run changes the run.
 *
 * <h3>Threading</h3>
 * One sim thread mutates; the EDT only reads. A {@link ReentrantReadWriteLock} keeps them apart:
 * <ul>
 *   <li><b>Write lock</b> — the sim thread across each {@code advanceTo}, plus mouse drag and map
 *       mutations on the EDT.</li>
 *   <li><b>Read lock</b>  — the EDT render loop and hit-testing.</li>
 * </ul>
 * Robots no longer tick concurrently, so {@code incomingMessages} being a
 * {@link java.util.concurrent.ConcurrentLinkedQueue} is now belt-and-braces rather than load-bearing.
 */
public class AsyncRobotPanel extends JPanel {

    // ------------------------------------------------------------------
    // Timing constants
    // ------------------------------------------------------------------
    private static final int  RENDER_FPS         = 30;
    private static final long RENDER_PERIOD_MS    = 1000L / RENDER_FPS;
    private static final long DEFAULT_PERIOD_MS   = GeometricCycleLatticeRobot.DEFAULT_TICK_RATE > 0.0
            ? (long) (1000.0 / GeometricCycleLatticeRobot.DEFAULT_TICK_RATE) : 1000L;
    private static final long PROXIMITY_PERIOD_MS = 100L;

    // ------------------------------------------------------------------
    // Bubble-overlay colours
    //
    // Shared by drawBubbles and drawBubbleLegend so the two can never disagree —
    // a legend that has drifted from what is on screen is worse than no legend.
    // The three severity tiers are a scale (clear / near / overlap); the keep-out
    // ring is deliberately outside that scale, in a neutral grey, because it is a
    // measurement aid rather than a fourth severity level.
    // ------------------------------------------------------------------
    private static final Color BUBBLE_CLEAR_FILL = new Color( 90, 180, 120,  45);
    private static final Color BUBBLE_CLEAR_EDGE = new Color( 60, 150,  90, 110);
    private static final Color BUBBLE_NEAR_FILL  = new Color(230, 160,  40,  70);
    private static final Color BUBBLE_NEAR_EDGE  = new Color(200, 130,  20, 160);
    private static final Color BUBBLE_OVER_FILL  = new Color(220,  60,  40,  90);
    private static final Color BUBBLE_OVER_EDGE  = new Color(200,  40,  20, 210);
    private static final Color KEEP_OUT_RING     = new Color( 70,  78,  90, 200);

    // ------------------------------------------------------------------
    // Role colours
    // ------------------------------------------------------------------
    private static final Map<CycleRole, Color> ROLE_COLORS = new EnumMap<>(CycleRole.class);
    static {
        ROLE_COLORS.put(CycleRole.root,        new Color(0xFF6B35));
        ROLE_COLORS.put(CycleRole.cycleBuilder, new Color(0x4A90E2));
        ROLE_COLORS.put(CycleRole.stable,      new Color(0x7ED321));
        ROLE_COLORS.put(CycleRole.unassigned,  new Color(0xBDBDBD));
    }

    // ------------------------------------------------------------------
    // Shared state — guarded by lock
    // ------------------------------------------------------------------
    private final Map<Integer, GeometricCycleLatticeRobot> robots = new LinkedHashMap<>();
    private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();

    // ------------------------------------------------------------------
    // Async execution
    //
    // One schedule, one thread. The staggered activation model is unchanged -- it lives in
    // SimSchedule now rather than in a pool's timing -- but the wall clock no longer decides
    // anything except how fast a human watches. See SimSchedule for why that had to change.
    // ------------------------------------------------------------------
    private final SimSchedule schedule = new SimSchedule(
            new PanelEvents(), DEFAULT_PERIOD_MS, PROXIMITY_PERIOD_MS, RENDER_PERIOD_MS);
    private Thread simThread;

    /**
     * The wall-clock instant that logical time zero corresponds to, moved forward across pauses.
     * Playback pacing only -- no simulation state is derived from it.
     */
    private volatile long wallAnchorMs = 0;

    private volatile boolean simRunning  = false;
    private volatile boolean simStarted  = false;
    private volatile long    periodMs    = DEFAULT_PERIOD_MS;

    private final SimulationLogger simLogger = new SimulationLogger(false);
    private int globalStepCount = 0;

    // ------------------------------------------------------------------
    // Telemetry — written by executor threads, read approx by EDT
    // ConcurrentHashMap because the increment below runs outside the read lock, on
    // several executor threads at once. Stale reads remain fine for display, but the
    // writes are real concurrent mutation, and merge() keeps them from being lost.
    // ------------------------------------------------------------------
    private final Map<Integer, Long> tickCounts      = new ConcurrentHashMap<>();
    private final Map<Integer, Long> lastActivatedMs = new ConcurrentHashMap<>();

    // ------------------------------------------------------------------
    // Interaction state (EDT-only)
    // ------------------------------------------------------------------
    private GeometricCycleLatticeRobot selectedRobot = null;
    private boolean dragging      = false;
    private double  offsetX, offsetY;
    private boolean showProximity = false;
    private boolean showStats     = false;
    private boolean showBubbles   = false;
    private long    simStartWallMs = 0;

    // Control widgets
    private JButton playPauseBtn;
    private JLabel  speedLabel;
    // ------------------------------------------------------------------
    // Constructor
    // ------------------------------------------------------------------
    public AsyncRobotPanel() {
        setLayout(new BorderLayout());
        setBackground(Color.WHITE);

        JPanel canvas = buildCanvas();
        add(canvas, BorderLayout.CENTER);
        add(buildControlStrip(), BorderLayout.SOUTH);

        new Timer((int) RENDER_PERIOD_MS, e -> canvas.repaint()).start();

        System.out.println("[Panel] HTML tick log: " + simLogger.directory().resolve("index.html"));
    }

    public void shutdown() {
        simRunning = false;
        joinSimThread();
        simLogger.close();
    }

    // ------------------------------------------------------------------
    // Canvas
    // ------------------------------------------------------------------
    private JPanel buildCanvas() {
        JPanel canvas = new JPanel() {
            @Override protected void paintComponent(Graphics g) {
                super.paintComponent(g);
                render((Graphics2D) g);
            }
        };
        canvas.setBackground(Color.WHITE);
        canvas.setPreferredSize(new Dimension(1000, 800));
        canvas.setFocusable(true);

        canvas.addMouseListener(new MouseAdapter() {
            @Override public void mousePressed(MouseEvent e) {
                canvas.requestFocusInWindow();
                lock.readLock().lock();
                try {
                    for (GeometricCycleLatticeRobot r : robots.values()) {
                        if (r.contains(e.getX(), e.getY())) {
                            selectedRobot = r;
                            offsetX = e.getX() - r.getPosition().x;
                            offsetY = e.getY() - r.getPosition().y;
                            dragging = true;
                            r.promoteToPrimaryRoot();
                            r.dataDump();
                            break;
                        }
                    }
                } finally { lock.readLock().unlock(); }
            }
            @Override public void mouseReleased(MouseEvent e) {
                dragging = false;
                selectedRobot = null;
            }
        });

        canvas.addMouseMotionListener(new MouseAdapter() {
            @Override public void mouseDragged(MouseEvent e) {
                if (selectedRobot == null || !dragging) return;
                lock.writeLock().lock();
                try {
                    selectedRobot.setPosition(new OrientedPoint(
                            e.getX() - offsetX,
                            e.getY() - offsetY,
                            selectedRobot.getPosition().getOrientation()));
                } finally { lock.writeLock().unlock(); }
            }
        });

        canvas.addKeyListener(new KeyAdapter() {
            @Override public void keyPressed(KeyEvent e) {
                switch (e.getKeyCode()) {
                    case KeyEvent.VK_SPACE -> togglePlayPause();
                    case KeyEvent.VK_RIGHT -> { if (!simRunning) singleStep(); }
                    case KeyEvent.VK_D     -> showProximity = !showProximity;
                    case KeyEvent.VK_T     -> showStats     = !showStats;
                    case KeyEvent.VK_B     -> showBubbles   = !showBubbles;
                    case KeyEvent.VK_S     -> savePNG(canvas);
                    case KeyEvent.VK_J     -> {
                        lock.readLock().lock();
                        try { RobotDataIO.exportToJSON(robots); }
                        finally { lock.readLock().unlock(); }
                    }
                    case KeyEvent.VK_K     -> loadFromJSON();
                }
            }
        });

        return canvas;
    }

    // ------------------------------------------------------------------
    // Control strip
    // ------------------------------------------------------------------
    private JPanel buildControlStrip() {
        JPanel strip = new JPanel(new FlowLayout(FlowLayout.LEFT, 8, 5));
        strip.setBackground(new Color(28, 28, 28));
        strip.setBorder(new EmptyBorder(4, 8, 4, 8));

        playPauseBtn = darkButton("▶  Start");
        playPauseBtn.addActionListener(e -> togglePlayPause());
        strip.add(playPauseBtn);

        JButton stepBtn = darkButton("⏭  Step");
        stepBtn.addActionListener(e -> singleStep());
        strip.add(stepBtn);

        JButton resetBtn = darkButton("⟳  Reset");
        resetBtn.addActionListener(e -> resetSimulation());
        strip.add(resetBtn);

        JButton loadBtn = darkButton("📂  Load JSON");
        loadBtn.addActionListener(e -> loadFromJSON());
        strip.add(loadBtn);

        JButton saveBtn = darkButton("💾  Save JSON");
        saveBtn.addActionListener(e -> {
            lock.readLock().lock();
            try { RobotDataIO.exportToJSON(robots); }
            finally { lock.readLock().unlock(); }
        });
        strip.add(saveBtn);

        strip.add(dimLabel("  Period:"));

        JSlider slider = new JSlider(500, 3000, (int) DEFAULT_PERIOD_MS);
        slider.setInverted(true); // left = faster
        slider.setBackground(new Color(28, 28, 28));
        slider.setPreferredSize(new Dimension(150, 26));
        speedLabel = dimLabel(DEFAULT_PERIOD_MS + " ms");
        slider.addChangeListener(e -> {
            periodMs = slider.getValue();
            speedLabel.setText(periodMs + " ms");
            if (slider.getValueIsAdjusting()) return;          // still dragging
            GeometricCycleLatticeRobot.setTickRate(1000.0 / periodMs);
            if (simRunning) rescheduleRobotTasks();
        });
        strip.add(slider);
        strip.add(speedLabel);

        strip.add(Box.createHorizontalStrut(10));

        JButton proxBtn = darkButton("⊙  Proximity");
        proxBtn.addActionListener(e -> showProximity = !showProximity);
        strip.add(proxBtn);

        JButton bubblesBtn = darkButton("⬤  Bubbles");
        bubblesBtn.addActionListener(e -> showBubbles = !showBubbles);
        strip.add(bubblesBtn);

        JButton statsBtn = darkButton("📊  Stats");
        statsBtn.addActionListener(e -> showStats = !showStats);
        strip.add(statsBtn);

        JCheckBox logUnchangedBox = new JCheckBox("Log unchanged");
        logUnchangedBox.setSelected(simLogger.isLoggingUnchangedRobots());
        logUnchangedBox.setBackground(new Color(28, 28, 28));
        logUnchangedBox.setForeground(new Color(180, 180, 180));
        logUnchangedBox.setFocusPainted(false);
       logUnchangedBox.addActionListener(e -> simLogger.setLogUnchangedRobots(logUnchangedBox.isSelected()));
        strip.add(logUnchangedBox);

        strip.add(dimLabel("   [Space] play/pause  [→] step  [K] load  [J] save  [D] proximity  [B] bubbles  [T] stats  [S] screenshot"));
        return strip;
    }

    // ------------------------------------------------------------------
    // Robot map management
    // ------------------------------------------------------------------
    public void addRobot(GeometricCycleLatticeRobot robot) {
        lock.writeLock().lock();
        try {
            robots.put(robot.getRobotId(), robot);
            tickCounts.put(robot.getRobotId(), 0L);
            lastActivatedMs.put(robot.getRobotId(), -1L);
        } finally { lock.writeLock().unlock(); }
    }

    private void loadFromJSON() {
        boolean wasRunning = simRunning;
        if (wasRunning) pauseSimulation();

        Map<Integer, GeometricCycleLatticeRobot> imported = RobotDataIO.importFromJSON();
        if (imported.isEmpty()) {
            System.err.println("[Panel] Import returned no robots.");
            if (wasRunning) resumeSimulation();
            return;
        }

        lock.writeLock().lock();
        try {
            robots.clear();
            tickCounts.clear();
            lastActivatedMs.clear();
            robots.putAll(imported);
            for (int id : robots.keySet()) {
                tickCounts.put(id, 0L);
                lastActivatedMs.put(id, -1L);
            }
        } finally { lock.writeLock().unlock(); }

        runProximityCheck();
        System.out.println("[Panel] Loaded " + imported.size() + " robots from JSON.");
        if (wasRunning) resumeSimulation();
    }

    // ------------------------------------------------------------------
    // Simulation lifecycle
    // ------------------------------------------------------------------
    private void togglePlayPause() {
        if (!simStarted)     startSimulation();
        else if (simRunning) pauseSimulation();
        else                 resumeSimulation();
    }

    private void startSimulation() {
        if (robots.isEmpty()) return;

        simStarted     = true;
        simStartWallMs = System.currentTimeMillis();
        armSchedule();
        String identity = "Run start: period=" + periodMs + "ms, robots=" + robots.size()
                + ", graph=" + GeometricCycleLatticeRobot.GRAPH.getClass().getSimpleName();
        simLogger.note(identity);
        System.out.println("[Panel] " + identity
                + " — reproducible from these; moving the speed slider changes the run.");
        resumeSimulation();
    }

    private void pauseSimulation() {
        simRunning = false;
        joinSimThread();
        SwingUtilities.invokeLater(() -> playPauseBtn.setText("▶  Resume"));
    }

    /**
     * Starts the loop that walks logical time forward, pinned to where the run left off.
     *
     * <p>Re-anchoring rather than resetting is what makes a pause invisible to the simulation: the
     * schedule keeps its own clock, and this only re-establishes which wall-clock instant that clock
     * is currently level with.
     */
    private void resumeSimulation() {
        simRunning = true;
        wallAnchorMs = System.currentTimeMillis() - schedule.nowMs();
        simThread = new Thread(this::runLoop, "sim-clock");
        simThread.setDaemon(true);
        simThread.start();
        SwingUtilities.invokeLater(() -> playPauseBtn.setText("⏸  Pause"));
    }

    private void resetSimulation() {
        if (simRunning) pauseSimulation();
        simStarted = false;
        lock.writeLock().lock();
        try {
            schedule.reset();
            for (GeometricCycleLatticeRobot r : robots.values()) {
                r.clearNeighbors();
                r.clearEdges();
            }
            tickCounts.replaceAll((id, v) -> 0L);
            lastActivatedMs.replaceAll((id, v) -> -1L);
        } catch (Throwable t) {
            System.out.println("Error with a robot");
            t.printStackTrace();
        } finally { lock.writeLock().unlock(); }
        SwingUtilities.invokeLater(() -> playPauseBtn.setText("▶  Start"));
    }

    // ------------------------------------------------------------------
    // Scheduling
    // ------------------------------------------------------------------

    /** Seeds the schedule for the current swarm and period. */
    private void armSchedule() {
        lock.writeLock().lock();
        try {
            schedule.arm(new ArrayList<>(robots.keySet()), periodMs);
        } finally { lock.writeLock().unlock(); }
    }

    /**
     * The play loop: walk logical time forward to wherever the wall clock says it should be.
     *
     * <p>All the wall clock does is set a target. Whether this thread gets there in one long
     * {@code advanceTo} after a stall or in a hundred short ones, the events run in the same order
     * with the same arguments, so a slow machine produces a slower-looking run and an identical one.
     *
     * <p>The write lock is held across each advance, which makes the sim the only mutator and leaves
     * the EDT's render and drag handlers a consistent state to read.
     */
    private void runLoop() {
        while (simRunning) {
            long targetMs = System.currentTimeMillis() - wallAnchorMs;
            lock.writeLock().lock();
            try {
                schedule.advanceTo(targetMs);
            } catch (Exception e) {
                System.out.println("\n + [Panel] Error advancing simulation");
                e.printStackTrace();
            } finally { lock.writeLock().unlock(); }

            try {
                Thread.sleep(1);
            } catch (InterruptedException ie) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

    private void joinSimThread() {
        Thread thread = simThread;
        if (thread == null) return;
        try {
            thread.join(1000);
        } catch (InterruptedException ie) {
            Thread.currentThread().interrupt();
        }
        simThread = null;
    }

    /**
     * Re-arms the schedule after the speed slider moves.
     *
     * <p>The slider sets the <em>logical</em> activation period, not the playback rate, so it is
     * part of what a run is -- {@code GeometricCycleLatticeRobot.setTickRate} derives from it and
     * the protocol's own timing conversions read that. Two runs are therefore only comparable at the
     * same setting, and a run whose slider moved partway through is not reproducible from its
     * starting parameters at all. That is worth saying out loud rather than leaving as a surprise.
     */
    private void rescheduleRobotTasks() {
        String note = "Period changed to " + periodMs + "ms at logical t=" + schedule.nowMs()
                + "ms — this run is no longer reproducible from its starting parameters";
        simLogger.note(note);
        System.out.println("[Panel] " + note);
        armSchedule();
    }

    // ------------------------------------------------------------------
    // Event bodies — invoked by SimSchedule, never directly
    // ------------------------------------------------------------------
    private final class PanelEvents implements SimSchedule.Handler {
        @Override public void proximity() {
            runProximityCheck();
        }

        @Override public void tick(int robotId) {
            tickRobot(robotId);
        }

        @Override public void motion(double dtSeconds) {
            for (GeometricCycleLatticeRobot robot : robots.values()) {
                robot.move(dtSeconds);
            }
        }
    }

    /**
     * One robot's activation.
     *
     * <p>No {@code simRunning} check any more: whether this should run is the schedule's decision,
     * and the step button drives the same schedule while the run is paused.
     */
    private void tickRobot(int id) {
        TickRecord rec = null;
        try {
            GeometricCycleLatticeRobot robot = robots.get(id);
            if (robot == null) return;
            double dt = periodMs / 1000.0;
            int tick = (int) (tickCounts.getOrDefault(id, 0L) + 1);
            rec = robot.executeTimeStep(dt, tick);
        } catch(Exception e) {
            System.out.println("\n + [Panel] Error ticking robot " + id);
            e.printStackTrace();
        }
        if(rec != null) simLogger.record(rec);

        tickCounts.merge(id, 1L, Long::sum);
        lastActivatedMs.put(id, System.currentTimeMillis());
    }

    // ------------------------------------------------------------------
    // Single step — called from EDT
    // ------------------------------------------------------------------
    /**
     * Advances one activation period, through the same schedule the play loop uses.
     *
     * <p>This used to be a second implementation of the activation model -- every robot ticked once
     * per press, each moving immediately after its own tick -- which is not what the running
     * simulation did. Stepping and playing therefore explored subtly different systems. Sharing
     * {@code advanceTo} makes them agree by construction rather than by inspection.
     */
    private void singleStep() {
        if (robots.isEmpty()) return;
        if (!schedule.isArmed()) armSchedule();

        lock.writeLock().lock();
        try {
            schedule.advanceTo(schedule.nowMs() + periodMs);
        } catch(Exception e) {
            e.printStackTrace();
        } finally { lock.writeLock().unlock(); }
    }

    // ------------------------------------------------------------------
    // Proximity check — sole writer during simulation
    // ------------------------------------------------------------------
    public void runProximityCheck() {
        lock.writeLock().lock();
        try {
            for (GeometricCycleLatticeRobot robot : robots.values()) {
                robot.clearNeighbors();
                for (GeometricCycleLatticeRobot other : robots.values()) {
                    if (robot.getRobotId() == other.getRobotId()) continue;
                    if (robot.getPosition().distance(other.getPosition())
                            <= GeometricCycleLatticeRobot.COMM_RANGE) {
                        robot.addNeighbor(other);
                    }
                }
            }
        } finally { lock.writeLock().unlock(); }
    }

    // ------------------------------------------------------------------
    // Rendering — EDT, read lock
    // ------------------------------------------------------------------
    private void render(Graphics2D g2) {
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);


        lock.readLock().lock();
        try {
            if (showProximity) drawProximity(g2);
            if (showBubbles) drawBubbles(g2);
            drawEdges(g2);
            drawRobots(g2);
            if (showStats) drawStatsOverlay(g2);
        } finally { lock.readLock().unlock(); }

        drawLegend(g2);
        if (showBubbles) drawBubbleLegend(g2);
    }

    private void drawProximity(Graphics2D g2) {
        g2.setStroke(new BasicStroke(1f));
        for (GeometricCycleLatticeRobot r : robots.values()) {
            double x = r.getPosition().x;
            double y = r.getPosition().y;
            double d = GeometricCycleLatticeRobot.COMM_RANGE;
            g2.setColor(new Color(200, 200, 255, 40));
            g2.fill(new Ellipse2D.Double(x - d, y - d, d * 2, d * 2));
            g2.setColor(new Color(100, 100, 220, 100));
            g2.draw(new Ellipse2D.Double(x - d, y - d, d * 2, d * 2));
        }
    }

    /**
     * Draws each robot's physical bubble, coloured by how close its nearest neighbour is:
     * green when clear, amber when within one tick of travel of contact, red on an actual
     * overlap. The dashed keep-out ring is drawn for the selected robot only — at n=100,
     * drawing it for everyone is unreadable.
     *
     * <p>A red bubble anywhere means the hard guard has been violated, which should be
     * impossible once a run is under way. The shipped dataset does start with one
     * overlapping pair, which should clear within a few seconds and never return.
     */
    private void drawBubbles(Graphics2D g2) {
        final double r       = GeometricCycleLatticeRobot.BODY_RADIUS;
        final double keepOut = GeometricCycleLatticeRobot.KEEP_OUT;
        final double warn    = keepOut + GeometricCycleLatticeRobot.tickTravel();

        g2.setStroke(new BasicStroke(1f));
        for (GeometricCycleLatticeRobot a : robots.values()) {
            double nearest = nearestNeighborDistance(a);

            Color fill, edge;
            if (nearest <= keepOut) {
                fill = BUBBLE_OVER_FILL;  edge = BUBBLE_OVER_EDGE;
            } else if (nearest < warn) {
                fill = BUBBLE_NEAR_FILL;  edge = BUBBLE_NEAR_EDGE;
            } else {
                fill = BUBBLE_CLEAR_FILL; edge = BUBBLE_CLEAR_EDGE;
            }

            double x = a.getPosition().x;
            double y = a.getPosition().y;
            g2.setColor(fill);
            g2.fill(new Ellipse2D.Double(x - r, y - r, r * 2, r * 2));
            g2.setColor(edge);
            g2.draw(new Ellipse2D.Double(x - r, y - r, r * 2, r * 2));
        }

        if (selectedRobot != null) {
            double x = selectedRobot.getPosition().x;
            double y = selectedRobot.getPosition().y;
            g2.setStroke(new BasicStroke(1.2f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER,
                    10f, new float[] { 6f, 5f }, 0f));
            g2.setColor(KEEP_OUT_RING);
            g2.draw(new Ellipse2D.Double(x - keepOut, y - keepOut, keepOut * 2, keepOut * 2));
            g2.setStroke(new BasicStroke(1f));
        }
    }

    /** Distance from {@code robot} to its closest peer, or +inf if it is alone. */
    private double nearestNeighborDistance(GeometricCycleLatticeRobot robot) {
        double nearest = Double.POSITIVE_INFINITY;
        for (GeometricCycleLatticeRobot other : robots.values()) {
            if (robot.getRobotId() == other.getRobotId()) continue;
            nearest = Math.min(nearest, robot.getPosition().distance(other.getPosition()));
        }
        return nearest;
    }

    /**
     * Pairs currently closer than {@code KEEP_OUT}. The acceptance criterion for the hard
     * guard, reduced to one number: it should reach zero shortly after a run starts and
     * stay there.
     */
    private int countOverlappingPairs() {
        List<GeometricCycleLatticeRobot> all = new ArrayList<>(robots.values());
        int overlaps = 0;
        for (int i = 0; i < all.size(); i++) {
            for (int j = i + 1; j < all.size(); j++) {
                if (all.get(i).getPosition().distance(all.get(j).getPosition())
                        <= GeometricCycleLatticeRobot.KEEP_OUT) {
                    overlaps++;
                }
            }
        }
        return overlaps;
    }

    private void drawEdges(Graphics2D g2) {
        for (GeometricCycleLatticeRobot from : robots.values()) {
            from.getEdges().forEach(edge -> {
                GeometricCycleLatticeRobot to = robots.get(edge.getToId());
                if (to != null) edge.draw(g2, from, to);
            });
        }
    }

    private void drawRobots(Graphics2D g2) {
        for (GeometricCycleLatticeRobot r : robots.values()) {
            Color roleColor = roleColor(r);
            //Change to make it half transparent if moving
            if(r.isMovingToAssignedPosition()) {
                roleColor = new Color(127, 0, 255, 128);
            }
            g2.setColor(roleColor);
            Shape shape = r.draw();

            g2.fill(shape);
            if (r == selectedRobot) {
                g2.setStroke(new BasicStroke(2f));
                g2.draw(shape);
            }
            g2.setColor(Color.BLACK);
            g2.setFont(new Font("SansSerif", Font.BOLD, 9));
            g2.drawString(String.valueOf(r.getRobotId()),
                    (float)(r.getPosition().x - 4), (float)(r.getPosition().y + 4));
        }
    }

    private Color roleColor(GeometricCycleLatticeRobot r) {
        return ROLE_COLORS.getOrDefault(r.getRole(), ROLE_COLORS.get(CycleRole.unassigned));
    }

    private void drawStatsOverlay(Graphics2D g2) {
        int x = 10, y = 10, rowH = 15, pad = 5;
        int[] colW = {32, 100, 46, 110};
        int tableW = Arrays.stream(colW).sum() + pad * 2;
        int tableH = rowH * (robots.size() + 2) + pad * 2;

        g2.setColor(new Color(0, 0, 0, 160));
        g2.fillRoundRect(x, y, tableW, tableH, 8, 8);

        long now     = System.currentTimeMillis();
        long elapsed = simStarted ? (now - simStartWallMs) : 0;

        int overlaps = countOverlappingPairs();

        g2.setColor(Color.WHITE);
        g2.setFont(new Font("Monospaced", Font.BOLD, 10));
        g2.drawString(String.format("t=%.1fs  n=%d  period=%dms",
                elapsed / 1000.0, robots.size(), periodMs), x + pad, y + rowH);
        // Red whenever any pair is inside KEEP_OUT -- the hard guard's acceptance criterion.
        g2.setColor(overlaps == 0 ? new Color(140, 220, 160) : new Color(255, 110, 90));
        g2.drawString(String.format("overlaps: %d", overlaps), x + pad + 200, y + rowH);

        int row = y + rowH + pad;
        g2.setFont(new Font("Monospaced", Font.BOLD, 9));
        g2.setColor(new Color(160, 210, 255));
        String[] headers = {"ID", "Role", "Ticks", "Last tick"};
        int cx = x + pad;
        for (int i = 0; i < headers.length; i++) {
            g2.drawString(headers[i], cx, row);
            cx += colW[i];
        }
        row += rowH;

        g2.setFont(new Font("Monospaced", Font.PLAIN, 9));
        for (Map.Entry<Integer, GeometricCycleLatticeRobot> entry : robots.entrySet()) {
            int id   = entry.getKey();
            GeometricCycleLatticeRobot r = entry.getValue();
            long ticks = tickCounts.getOrDefault(id, 0L);
            long last  = lastActivatedMs.getOrDefault(id, -1L);
            String lastStr = last < 0 ? "never" : String.format("%.1fs ago", (now - last) / 1000.0);
            String role    = r.getRole().name();

            g2.setColor(roleColor(r));
            cx = x + pad;
            g2.drawString(String.valueOf(id), cx, row); cx += colW[0];
            g2.drawString(role,               cx, row); cx += colW[1];
            g2.setColor(Color.WHITE);
            g2.drawString(String.valueOf(ticks), cx, row); cx += colW[2];
            g2.drawString(lastStr,               cx, row);
            row += rowH;
        }
    }

    private void drawLegend(Graphics2D g2) {
        int x = getWidth() - 135, y = 10, rowH = 15, pad = 4;
        int h = ROLE_COLORS.size() * rowH + pad * 2;
        g2.setColor(new Color(0, 0, 0, 150));
        g2.fillRoundRect(x - pad, y, 128, h, 8, 8);
        g2.setFont(new Font("SansSerif", Font.PLAIN, 9));
        int ry = y + pad + rowH / 2;
        for (Map.Entry<CycleRole, Color> e : ROLE_COLORS.entrySet()) {
            g2.setColor(e.getValue());
            g2.fillRect(x, ry - 6, 10, 10);
            g2.setColor(Color.WHITE);
            g2.drawString(e.getKey().name(), x + 14, ry + 4);
            ry += rowH;
        }
    }

    /**
     * Explains the bubble overlay, shown only while that overlay is on and sitting
     * directly under the role legend.
     *
     * <p>Every threshold is formatted from the live constants rather than written out, so
     * retuning {@code BODY_RADIUS} cannot leave the legend quietly lying about what the
     * colours mean. The swatches use the same shared colours the bubbles are drawn with,
     * for the same reason.
     */
    private void drawBubbleLegend(Graphics2D g2) {
        final double body    = GeometricCycleLatticeRobot.BODY_RADIUS;
        final double keepOut = GeometricCycleLatticeRobot.KEEP_OUT;
        final double warn    = keepOut + GeometricCycleLatticeRobot.tickTravel();

        final int w = 172, rowH = 15, pad = 6, titleH = 13;
        int roleLegendH = ROLE_COLORS.size() * rowH + 4 * 2;
        // Right edge flush with the role legend above (which ends at getWidth() - 11),
        // and stacked directly beneath it.
        int x = getWidth() - 11 + pad - w;
        int y = 10 + roleLegendH + 8;
        int h = titleH + rowH * 4 + titleH + pad * 2;

        g2.setColor(new Color(0, 0, 0, 150));
        g2.fillRoundRect(x - pad, y, w, h, 8, 8);

        g2.setFont(new Font("SansSerif", Font.BOLD, 9));
        g2.setColor(new Color(170, 195, 215));
        g2.drawString("BODY CLEARANCE", x, y + pad + 8);

        g2.setFont(new Font("SansSerif", Font.PLAIN, 9));
        int ry = y + pad + titleH + rowH - 3;
        ry = bubbleLegendRow(g2, x, ry, rowH, BUBBLE_CLEAR_FILL, BUBBLE_CLEAR_EDGE, false,
                String.format("clear (%.0f+)", warn));
        ry = bubbleLegendRow(g2, x, ry, rowH, BUBBLE_NEAR_FILL, BUBBLE_NEAR_EDGE, false,
                String.format("one tick away (%.0f-%.0f)", keepOut, warn));
        ry = bubbleLegendRow(g2, x, ry, rowH, BUBBLE_OVER_FILL, BUBBLE_OVER_EDGE, false,
                String.format("OVERLAP (≤%.0f)", keepOut));
        ry = bubbleLegendRow(g2, x, ry, rowH, null, KEEP_OUT_RING, true,
                "keep-out, selected");

        // Spells out what the two radii actually are, because the dashed ring is not a
        // bigger body -- it marks where another robot's CENTRE sits at first contact.
        g2.setColor(new Color(150, 158, 170));
        g2.drawString(String.format("body r %.0f  ·  keep-out %.0f", body, keepOut), x, ry + 2);
    }

    /** One swatch-and-label row of the bubble legend. Returns the next row's baseline. */
    private int bubbleLegendRow(Graphics2D g2, int x, int ry, int rowH,
                                Color fill, Color edge, boolean dashed, String label) {
        final int d = 11;
        int cy = ry - 8;

        // White chip behind the swatch: the bubble fills are deliberately low-alpha so
        // they can overlap on the canvas, and would be invisible on this dark panel.
        // Drawing them over white shows exactly what they look like in the simulation.
        g2.setColor(Color.WHITE);
        g2.fillRect(x - 1, cy - 1, d + 3, d + 3);

        Ellipse2D.Double swatch = new Ellipse2D.Double(x, cy, d, d);
        if (fill != null) {
            g2.setColor(fill);
            g2.fill(swatch);
        }
        g2.setStroke(dashed
                ? new BasicStroke(1.2f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER,
                        10f, new float[] { 3f, 2.5f }, 0f)
                : new BasicStroke(1f));
        g2.setColor(edge);
        g2.draw(swatch);
        g2.setStroke(new BasicStroke(1f));

        g2.setColor(Color.WHITE);
        g2.drawString(label, x + d + 7, ry);
        return ry + rowH;
    }

    // ------------------------------------------------------------------
    // Screenshot
    // ------------------------------------------------------------------
    private void savePNG(JPanel canvas) {
        BufferedImage img = new BufferedImage(canvas.getWidth(), canvas.getHeight(),
                BufferedImage.TYPE_INT_ARGB);
        Graphics2D g2 = img.createGraphics();
        canvas.paintAll(g2);
        g2.dispose();
        try {
            File dir = new File("output/robot_panel_images");
            dir.mkdirs();
            String ts = LocalDateTime.now()
                    .format(java.time.format.DateTimeFormatter.ofPattern("yyyy_MM_dd_HH_mm_ss"));
            File out = new File(dir, "async_panel_" + ts + ".png");
            if (javax.imageio.ImageIO.write(img, "png", out))
                System.out.println("[Panel] Screenshot: " + out.getPath());
        } catch (Exception ex) { ex.printStackTrace(); }
    }

    // ------------------------------------------------------------------
    // Widget helpers
    // ------------------------------------------------------------------
    private JButton darkButton(String text) {
        JButton b = new JButton(text);
        b.setBackground(new Color(55, 55, 55));
        b.setForeground(Color.WHITE);
        b.setFocusPainted(false);
        b.setBorderPainted(false);
        b.setFont(b.getFont().deriveFont(11f));
        return b;
    }

    private JLabel dimLabel(String text) {
        JLabel l = new JLabel(text);
        l.setForeground(new Color(180, 180, 180));
        l.setFont(l.getFont().deriveFont(10f));
        return l;
    }

    // ------------------------------------------------------------------
    // Entry point
    // ------------------------------------------------------------------
    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            AsyncRobotPanel panel = new AsyncRobotPanel();

            Map<Integer, GeometricCycleLatticeRobot> loaded = RobotDataIO.importFromJSON();
            if (!loaded.isEmpty()) {
                loaded.values().forEach(panel::addRobot);
                panel.runProximityCheck();
                System.out.println("[main] Loaded " + loaded.size() + " robots from JSON.");
            } else {
                System.out.println("[main] No JSON found — use Load JSON button.");
            }

            JFrame frame = new JFrame("Async Lattice Robot Simulation");
            frame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
            frame.addWindowListener(new WindowAdapter() {
                @Override public void windowClosing(WindowEvent e) {
                    panel.shutdown();
                    frame.dispose();
                    System.exit(0);
                }
            });
            frame.setContentPane(panel);
            frame.pack();
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);
        });
    }
}