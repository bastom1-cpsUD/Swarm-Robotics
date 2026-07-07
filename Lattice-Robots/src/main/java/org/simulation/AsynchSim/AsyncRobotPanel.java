package org.simulation.AsynchSim;

import org.communicationModels.CycleRole;
import org.graphs.OrientedPoint;
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
 * Each robot is scheduled on a shared {@link ScheduledThreadPoolExecutor} with a
 * fixed period and a staggered initial delay:
 * <pre>
 *   delay_i = (sortedIndex_i * period) / robotCount
 * </pre>
 * This spaces activations evenly across one full period from the first tick,
 * giving genuine asynchrony without randomness, and making runs reproducible.
 *
 * <h3>Threading</h3>
 * One {@link ReentrantReadWriteLock} governs all shared robot state:
 * <ul>
 *   <li><b>Write lock</b> — proximity check task (periodic), mouse drag, map mutations.</li>
 *   <li><b>Read lock</b>  — robot tick tasks (concurrent), EDT render loop.</li>
 * </ul>
 * NOTE: {@code incomingMessages} inside {@code CommunicationSystem} must be a
 * {@link java.util.concurrent.ConcurrentLinkedQueue} because multiple robots can
 * enqueue to each other's queues while all holding the read lock simultaneously.
 */
public class AsyncRobotPanel extends JPanel {

    // ------------------------------------------------------------------
    // Timing constants
    // ------------------------------------------------------------------
    private static final int  RENDER_FPS         = 30;
    private static final long RENDER_PERIOD_MS    = 1000L / RENDER_FPS;
    private static final long DEFAULT_PERIOD_MS   = 1000L;
    private static final long PROXIMITY_PERIOD_MS = 100L;

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
    // ------------------------------------------------------------------
    private ScheduledThreadPoolExecutor executor;
    private final List<ScheduledFuture<?>> robotFutures  = new ArrayList<>();
    private ScheduledFuture<?>            proximityFuture;
    private ScheduledFuture<?>            motionFuture;
    private volatile long                 lastMotionNanos = System.nanoTime();

    private volatile boolean simRunning  = false;
    private volatile boolean simStarted  = false;
    private volatile long    periodMs    = DEFAULT_PERIOD_MS;

    private final SimulationLogger simLogger = new SimulationLogger(false);
    private int globalStepCount = 0;

    // ------------------------------------------------------------------
    // Telemetry — written by executor threads, read approx by EDT
    // Plain map is fine: stale reads of longs are acceptable for display
    // ------------------------------------------------------------------
    private final Map<Integer, Long> tickCounts      = new LinkedHashMap<>();
    private final Map<Integer, Long> lastActivatedMs = new LinkedHashMap<>();

    // ------------------------------------------------------------------
    // Interaction state (EDT-only)
    // ------------------------------------------------------------------
    private GeometricCycleLatticeRobot selectedRobot = null;
    private boolean dragging      = false;
    private double  offsetX, offsetY;
    private boolean showProximity = false;
    private boolean showStats     = false;
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
            if (simRunning) rescheduleRobotTasks();
        });
        strip.add(slider);
        strip.add(speedLabel);

        strip.add(Box.createHorizontalStrut(10));

        JButton proxBtn = darkButton("⊙  Proximity");
        proxBtn.addActionListener(e -> showProximity = !showProximity);
        strip.add(proxBtn);

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

        strip.add(dimLabel("   [Space] play/pause  [→] step  [K] load  [J] save  [T] stats  [S] screenshot"));
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
        int threads = Runtime.getRuntime().availableProcessors() + 1;
        executor = new ScheduledThreadPoolExecutor(threads);
        executor.setRemoveOnCancelPolicy(true);
        simStarted     = true;
        simRunning     = true;
        simStartWallMs = System.currentTimeMillis();
        scheduleProximityTask();
        scheduleRobotTasks();
        scheduleMotionTask();
        SwingUtilities.invokeLater(() -> playPauseBtn.setText("⏸  Pause"));
    }

    private void pauseSimulation() {
        simRunning = false;
        cancelRobotFutures();
        if (proximityFuture != null) proximityFuture.cancel(false);
        if (motionFuture    != null) motionFuture.cancel(false);
        SwingUtilities.invokeLater(() -> playPauseBtn.setText("▶  Resume"));
    }

    private void resumeSimulation() {
        simRunning = true;
        scheduleProximityTask();
        scheduleRobotTasks();
        scheduleMotionTask();
        SwingUtilities.invokeLater(() -> playPauseBtn.setText("⏸  Pause"));
    }

    private void resetSimulation() {
        if (simRunning) pauseSimulation();
        simStarted = false;
        lock.writeLock().lock();
        try {
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
    private void scheduleProximityTask() {
        proximityFuture = executor.scheduleAtFixedRate(
                this::runProximityCheck,
                0L, PROXIMITY_PERIOD_MS, TimeUnit.MILLISECONDS);
    }

    private void scheduleRobotTasks() {
        List<Integer> ids;
        lock.readLock().lock();
        try { ids = new ArrayList<>(robots.keySet()); }
        finally { lock.readLock().unlock(); }
        Collections.sort(ids);

        int n = ids.size();
        for (int i = 0; i < n; i++) {
            int  id           = ids.get(i);
            long initialDelay = (long)((double) i / n * periodMs);
            ScheduledFuture<?> future = executor.scheduleAtFixedRate(
                    () -> tickRobot(id),
                    initialDelay, periodMs, TimeUnit.MILLISECONDS);
            robotFutures.add(future);
        }
    }

    private void rescheduleRobotTasks() {
        cancelRobotFutures();
        if (proximityFuture != null) proximityFuture.cancel(false);
        if (motionFuture    != null) motionFuture.cancel(false);
        scheduleProximityTask();
        scheduleRobotTasks();
        scheduleMotionTask();
    }

    private void cancelRobotFutures() {
        robotFutures.forEach(f -> f.cancel(false));
        robotFutures.clear();
    }

    // ------------------------------------------------------------------
    // Motion task — 30 fps, decoupled from logic tick rate
    // ------------------------------------------------------------------
    private void scheduleMotionTask() {
        lastMotionNanos = System.nanoTime();
        motionFuture = executor.scheduleAtFixedRate(() -> {
            if (!simRunning) return;
            long now = System.nanoTime();
            double dt = (now - lastMotionNanos) / 1_000_000_000.0;
            lastMotionNanos = now;
            lock.readLock().lock();
            try {
                for (GeometricCycleLatticeRobot robot : robots.values()) {
                    robot.move(dt);
                }
            } finally { lock.readLock().unlock(); }
        }, 0L, 1000L / RENDER_FPS, TimeUnit.MILLISECONDS);
    }

    // ------------------------------------------------------------------
    // Robot tick — runs on executor thread, holds read lock
    // ------------------------------------------------------------------
    private void tickRobot(int id) {
        if (!simRunning) return;
        TickRecord rec = null;
        lock.readLock().lock();
        try {
            GeometricCycleLatticeRobot robot = robots.get(id);
            if (robot == null) return;
            double dt = periodMs / 1000.0;
            int tick = (int) (tickCounts.getOrDefault(id, 0L) + 1);
            rec = robot.executeTimeStep(dt, tick);
        } catch(Exception e) {
            e.printStackTrace();
        } finally { lock.readLock().unlock(); }

        if(rec != null) simLogger.record(rec);

        // Telemetry updated outside the lock — EDT reads are display-only
        tickCounts.put(id, tickCounts.getOrDefault(id, 0L) + 1);
        lastActivatedMs.put(id, System.currentTimeMillis());
    }

    // ------------------------------------------------------------------
    // Single step — called from EDT, safe to run synchronously
    // ------------------------------------------------------------------
    private void singleStep() {
        if (robots.isEmpty()) return;
        runProximityCheck();
        lock.readLock().lock();
        try {
            double dt = periodMs / 1000.0;
            for (Map.Entry<Integer, GeometricCycleLatticeRobot> e : robots.entrySet()) {
                int tick = (int) (tickCounts.getOrDefault(e.getKey(), 0L) + 1);
                TickRecord rec = e.getValue().executeTimeStep(dt, tick);
                e.getValue().move(dt);
                simLogger.record(rec);
                tickCounts.put(e.getKey(), tickCounts.getOrDefault(e.getKey(), 0L) + 1);
                lastActivatedMs.put(e.getKey(), System.currentTimeMillis());
            }
        } catch(Exception e) {
            e.printStackTrace();
        } finally { lock.readLock().unlock(); }
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
            drawEdges(g2);
            drawRobots(g2);
            if (showStats) drawStatsOverlay(g2);
        } finally { lock.readLock().unlock(); }

        drawLegend(g2);
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
            g2.setColor(roleColor(r));
            Shape shape = r.draw();

            g2.fill(shape);
            if (r == selectedRobot) {
                g2.setColor(Color.YELLOW);
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

        g2.setColor(Color.WHITE);
        g2.setFont(new Font("Monospaced", Font.BOLD, 10));
        g2.drawString(String.format("t=%.1fs  n=%d  period=%dms",
                elapsed / 1000.0, robots.size(), periodMs), x + pad, y + rowH);

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