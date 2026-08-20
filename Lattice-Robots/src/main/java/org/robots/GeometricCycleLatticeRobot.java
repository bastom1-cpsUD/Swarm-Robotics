package org.robots;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

import org.communicationModels.TrustLevel;
import org.communicationModels.cycleBuildingComms.Communicatable;
import org.communicationModels.cycleBuildingComms.CycleRole;
import org.communicationModels.cycleBuildingComms.CyclebuilderComms;
import org.communicationModels.cycleBuildingComms.Messages.AbstractMessage;
import org.communicationModels.cycleBuildingComms.Messages.TargetClaimMessage;
import org.drawingModels.TriangularModel;
import org.graphs.util.OrientedPoint;
import org.graphs.voltage.DodecagonHexagonSquareVoltageGraph;
import org.graphs.voltage.DodecagonTriangleVoltageGraph;
import org.graphs.voltage.ElongatedTriangularVoltageGraph;
import org.graphs.voltage.HexagonSquareTriangleVoltageGraph;
import org.graphs.voltage.HexagonTriangleVoltageGraph;
import org.graphs.voltage.HexagonVoltageGraph;
import org.graphs.voltage.OctagonSquareVoltageGraph;
import org.graphs.voltage.SnubHexagonVoltageGraph;
import org.graphs.voltage.SnubSquareVoltageGraph;
import org.graphs.voltage.SquareVoltageGraph;
import org.graphs.voltage.VoltageGraph;
import org.graphs.voltage.TriangleVoltageGraph;
import org.utils.logging.CommsSnapshot;
import org.utils.logging.TickRecord;
import org.motionModels.LatticeMotionModel;
import org.motionModels.TimeStepDiffDrive;
import org.simulation.Edge;
import org.utils.AvoidanceGeometry;
import org.utils.MathUtils;

/**
 * Robot participating in geometric cycle-building lattice formation.
 */
public class GeometricCycleLatticeRobot extends Robot implements Communicatable {

    // ------------------------------------------------------------
    // Constants
    // ------------------------------------------------------------
    public static final double COMM_RANGE = 75.0;
    /**
     * Robot activations per second. One activation is one <em>tick</em>.
     */
    public static final double TICK_RATE = 1.0;
    public static final VoltageGraph GRAPH = SnubSquareVoltageGraph.build();

    /**
     * Radius of this robot's physical bubble. Two robots are in collision when their
     * centres are closer than {@link #KEEP_OUT}.
     *
     * <p><b>INVARIANT:</b> {@code 2 * BODY_RADIUS} must be strictly less than the shortest
     * edge length of every lattice in {@code org.graphs.voltage} — currently 50.0
     * (Hexagon, Triangle, OctagonSquare, SnubHexagon, ElongatedTriangular,
     * HexagonTriangle, HexagonSquareTriangle and both Dodecagon variants; SnubSquare and
     * Square are 70.0). Violating it makes two adjacent lattice spots permanently
     * collide, so no formation could ever close.
     *
     * <p>This is a <em>clearance</em> radius, deliberately smaller than the triangle
     * {@code TriangularModel} draws, whose nose vertex reaches
     * {@code 1.2 * 30/sqrt(3) = 20.78}. Sizing the bubble to the drawn hull would put 19
     * of the 100 robots in the shipped dataset in overlap before the first tick, against
     * 1 at 15.0. The bubble overlay (B key) draws the true bubble, so the discrepancy
     * stays visible rather than hidden. Retune alongside
     * {@code TriangularModel.ROBOT_SIZE}.
     */
    public static final double BODY_RADIUS = 15.0;

    /**
     * Centre-to-centre separation below which two robots are in collision.
     *
     * <p>The hard guard in {@link #move(double)} enforces this with <em>zero</em> safety
     * margin, which is correct only while {@code AsyncRobotPanel}'s motion task moves all
     * robots sequentially on a single thread: each robot then tests against neighbour
     * positions that are fully committed, so no pair can slip past each other's checks.
     * If motion is ever parallelised across robots, each must instead be tested against a
     * <em>snapshot</em> of neighbour poses taken before the frame, and this constant must
     * grow by one frame's travel ({@code MAX_LINEAR_SPEED / RENDER_FPS}, about 0.33).
     */
    public static final double KEEP_OUT = 2.0 * BODY_RADIUS;
    // ------------------------------------------------------------
    // Fields
    // ------------------------------------------------------------
    private final CyclebuilderComms commsSystem;
    private final LatticeMotionModel latticeMotionModel;

    private CopyOnWriteArrayList<Edge> edges;
    private ArrayList<GeometricCycleLatticeRobot> neighbors;

    /**
     * The pose {@link #move(double)} drives at — the lattice target when the path is
     * clear, a detour waypoint or a hold when it is not.
     *
     * <p>Volatile because it is written by this robot's tick task and read by the shared
     * 30 fps motion task, which hold only the read lock and therefore genuinely run
     * concurrently. Reference assignment is atomic but carries no visibility guarantee
     * without this. Safe only because every write installs a fresh {@code OrientedPoint};
     * the pointed-to object must never be mutated in place.
     */
    private volatile OrientedPoint assignedPosition;
    private boolean isMovingToAssignedPosition = false;

    /**
     * Whether the last motion frame's step was refused by the hard guard. Written by the
     * motion task, read by the render thread, hence volatile.
     */
    private volatile boolean lastStepVetoed = false;

    // ------------------------------------------------------------
    // Constructor
    // ------------------------------------------------------------
    public GeometricCycleLatticeRobot(int id, OrientedPoint pose) {
        super(id, pose, new TimeStepDiffDrive(), new TriangularModel());

        this.latticeMotionModel = (LatticeMotionModel) motionModel;

        this.commsSystem = new CyclebuilderComms(this, GRAPH);

        this.edges = new CopyOnWriteArrayList<>();
        this.neighbors = new ArrayList<>();

        this.assignedPosition = new OrientedPoint(pose);
    }

    // ------------------------------------------------------------
    // Neighbor handling
    // ------------------------------------------------------------
    public void addNeighbor(GeometricCycleLatticeRobot other) {
        if (!neighbors.contains(other)) neighbors.add(other);
    }

    public void removeNeighbor(GeometricCycleLatticeRobot other) {
        neighbors.remove(other);
    }

    public void clearNeighbors() {
        neighbors.clear();
    }

    public ArrayList<GeometricCycleLatticeRobot> getNeighbors() {
        return new ArrayList<>(Collections.unmodifiableList(neighbors));
    }

    // ------------------------------------------------------------
    // Edge visualization
    // ------------------------------------------------------------
    public void addEdge(Edge e) { if(!edges.contains(e)) edges.add(e); }
    public void clearEdges() { edges.clear(); }
    public CopyOnWriteArrayList<Edge> getEdges() { return edges; }

    // ------------------------------------------------------------
    // Messaging interface
    // ------------------------------------------------------------
    @Override
    public void enqueueMessage(AbstractMessage msg) {
        commsSystem.enqueueMessage(msg);
    }

    @Override
    public void processMessages() {
        commsSystem.processMessages();
    }

    @Override
    public void receiveClaim(TargetClaimMessage claim) {
        commsSystem.receiveClaim(claim);
    }

    // ------------------------------------------------------------
    // Main time-step
    // ------------------------------------------------------------
    /**
     * Executes one activation — one <em>tick</em>, and returns a full record of what happened: state immediately
     * before, what message was processed, what action resulted, what was sent, and state
     * immediately after.
     *
     * <p>The role dispatch below intentionally stays here rather than in the simulation
     * panel, since root/stable and cycleBuilder/unassigned activate their comms calls in
     * different orders.
     *
     * @param dt   time step, in seconds, forwarded to downstream motion logic
     * @param tick a logical tick number for this robot's own activation
     *             timeline (e.g. the panel's per-robot tick counter, or a
     *             global step counter during single-step debugging). Purely
     *             informational — does not affect behavior.
     * @return a record of everything that happened this activation
     */
    public TickRecord executeTimeStep(double dt, int tick) {
        CommsSnapshot before = commsSystem.snapshot();
        OrientedPoint poseBefore = new OrientedPoint(pose);

        commsSystem.beginTick();
        commsSystem.expireStaleClaims();
        commsSystem.ageEvasion();

        String processed;
        String action;

        switch(getRole()) {
            case CycleRole.root, CycleRole.stable -> {
                commsSystem.makeObservations();
                processed = commsSystem.processMessages(tick);
                action = commsSystem.sendMessage(true, tick);
            }
            default -> {
                commsSystem.makeObservations();

                // Classify who is moving before anything reads that classification. Must
                // also run before next tick overwrites the observations it diffs against.
                commsSystem.observeMotion();

                // Before processMessages, so an assignment landing this tick cancels an
                // evasion rather than racing it.
                commsSystem.consumeStandAside();

                String contention = commsSystem.detectAssignmentContention();

                // 1. Process incoming messages
                processed = commsSystem.processMessages(tick);

                // 2. Ask comms system for current target, 3. broadcast
                action = commsSystem.sendMessage(updateAssignedPosition(), tick);

                action = contention != null ? contention + " | " + action: action;

                // Last, because it needs the true target that updateAssignedPosition just
                // installed, and because it replaces that target with a detour or a hold.
                applyAvoidanceWaypoint();
            }
        }
        //Broadcast claim after processing messages for contention processing
        commsSystem.broadcastTargetClaim();

        CommsSnapshot after = commsSystem.snapshot();
        OrientedPoint poseAfter = new OrientedPoint(pose);

        return new TickRecord(tick, robotId, before, poseBefore, processed, action,
        commsSystem.sentThisTick(), after, poseAfter);
    }

    /**
     * Points this robot at whatever its comms system currently considers its target, and
     * reports whether it is already there.
     * @return true if this robot is already at its assigned pose and heading
     */
    private boolean updateAssignedPosition() {
        OrientedPoint target = commsSystem.getAssignedGlobalPosition();
        boolean atPos = (target != null
                && pose.distance(target) < MathUtils.EPSILON)
                && MathUtils.anglesEqual(pose.orientation, target.orientation);

        if (atPos) {
            // Arrived on position AND heading: snap onto the ideal target, then hold that
            // pose. Holding a fixed pose rather than continuing to chase a recomputed target
            // is what makes us a static reference instead of a parent whose residual
            // rotation keeps sweeping the children's targets and driving the rotate-to-point
            // jitter -- but freezing wherever we happened to stop baked in up to EPSILON of
            // position error and EPSILON radians of heading error, permanently.
            //
            // That mattered because children derive their targets from this pose
            // (CyclebuilderComms.getAssignedGlobalPosition), so the error compounded down
            // the chain -- and the heading component compounded with a lever arm of one edge
            // length. 1e-3 rad across a 70-unit edge displaces a child's target by 0.07:
            // seventy times the tolerance the cycle-closing test in
            // CyclebuilderComms.findBestNeighborForEdge accepts. A root sitting physically
            // on the closing spot then failed that test and was passed over.
            //
            // The correction is bounded by the arrival gate just evaluated -- at most
            // EPSILON of translation and EPSILON radians of rotation -- and is the same snap
            // TimeStepDiffDrive.moveTo already performs in its DONE branch. It is repeated
            // here because the gate can fire on a tick where the motion model has not
            // reached DONE (the target keeps moving while a parent is still converging),
            // which is exactly the case that used to freeze off-ideal.
            pose.x = target.x;
            pose.y = target.y;
            pose.orientation = MathUtils.normalizeAngle(target.orientation);
            assignedPosition = new OrientedPoint(pose);
            isMovingToAssignedPosition = false;
        } else if (target != null) {
            assignedPosition = new OrientedPoint(target);
            isMovingToAssignedPosition = true;
        } else {
            // No live assignment (e.g. we just became unassigned because the target was
            // occupied). This is the one and only place an evasion target is consulted,
            // and it is deliberately not the same notion as an assignment: it is a motion
            // waypoint, never a lattice claim. getClaimedLocalTarget() short-circuits on
            // role != cycleBuilder, so an evading robot broadcasts nothing and cannot
            // enter contention; getAssignedGlobalPosition() still returns null here, so
            // the arrival gate, the occupancy check and any child's derived target are all
            // untouched.
            //
            // No snap either -- the snap above exists to hold ideal lattice poses exact so
            // children's targets do not drift, and a sidestep is not an ideal pose.
            OrientedPoint evade = commsSystem.getEvasionGlobalPosition();
            assignedPosition = new OrientedPoint(evade != null ? evade : pose);
            isMovingToAssignedPosition = (evade != null);
        }

        return atPos;
    }

    /**
     * Replaces the motion waypoint with whatever the collision policy decided — the true
     * target when the path is clear, a detour waypoint around a stationary body, or this
     * robot's own pose when it is yielding.
     *
     * <p>Writing into {@code assignedPosition} is what keeps the two systems apart. That
     * field has exactly one consumer, {@link #move(double)}; the protocol reads
     * {@code CyclebuilderComms.getAssignedGlobalPosition()}, which nothing here touches.
     * So a detour can never be mistaken for a lattice claim, reach the arrival gate, or
     * propagate into a child's derived target.
     */
    private void applyAvoidanceWaypoint() {
        OrientedPoint planned = commsSystem.planMotionWaypoint(assignedPosition);
        if (planned != null) {
            assignedPosition = planned;
        }
    }

    // ------------------------------------------------------------
    // Motion
    // ------------------------------------------------------------
    /**
     * Advances this robot one motion frame toward its current waypoint, refusing any step
     * that would drive a body into a neighbour.
     *
     * <p>This is the collision layer's <strong>guarantee</strong>, and it is deliberately
     * the only place one lives. It runs at motion rate (30 fps) rather than tick rate
     * (1 Hz) because a decision taken once per tick is open-loop for roughly thirty motion
     * frames and therefore cannot guarantee anything. It classifies nothing, sends nothing
     * and reads no protocol state — it is a proximity sensor plus a veto, and a real
     * platform's bumper likewise runs faster than its radio.
     *
     * <p>The step is <em>speculated on a copy</em> rather than taken and rolled back.
     * {@code TimeStepDiffDrive.moveTo} mutates the pose it is handed in place, and rebuilds
     * its {@code moveState} and wheel velocities from scratch on every call, so a discarded
     * candidate leaks nothing except an inflated {@code distTraveled} counter — telemetry
     * only, read by no decision.
     *
     * <p>One bypass exists and is tolerated: {@link #updateAssignedPosition()}'s arrival
     * branch writes {@code pose} directly, outside this method. That write is bounded by
     * the arrival gate to {@link MathUtils#EPSILON}, four orders of magnitude below
     * {@link #KEEP_OUT}, so it cannot produce a consequential overlap.
     */
    @Override
    public void move(double dt) {
        OrientedPoint waypoint = assignedPosition;
        if (waypoint == null) {
            return;
        }

        OrientedPoint candidate = new OrientedPoint(pose);
        latticeMotionModel.moveTo(candidate, waypoint, dt);

        if (!stepIsPermitted(candidate)) {
            lastStepVetoed = true;
            return;
        }

        lastStepVetoed = false;
        pose.x = candidate.x;
        pose.y = candidate.y;
        pose.orientation = candidate.orientation;
    }

    /**
     * Whether a candidate pose may be committed, tested against every neighbour.
     *
     * <p>Iterates the {@code neighbors} field directly rather than through
     * {@link #getNeighbors()}: this runs thirty times a second for every robot and the
     * accessor allocates a defensive copy per call. Doing so is safe without added
     * synchronization because the list is only ever restructured by
     * {@code AsyncRobotPanel.runProximityCheck()} under the <b>write</b> lock, which
     * excludes the motion task holding the read lock.
     *
     * @param candidate the pose the motion model produced for this frame
     * @return true if no neighbour vetoes the step
     */
    private boolean stepIsPermitted(OrientedPoint candidate) {
        for (GeometricCycleLatticeRobot other : neighbors) {
            if (!AvoidanceGeometry.permitsStep(pose, candidate, other.getPosition(), KEEP_OUT)) {
                return false;
            }
        }
        return true;
    }

    // ------------------------------------------------------------
    // Accessors
    // ------------------------------------------------------------
    public LatticeMotionModel getLatticeMotionModel() {
        return latticeMotionModel;
    }

    public CycleRole getRole() {
        return commsSystem.getRole();
    }

    public TrustLevel getTrustLevel() {
        return commsSystem.getTrustLevel();
    }

    public void setTrustLevel(TrustLevel trust) {
        commsSystem.setTrustLevel(trust);
    }

    public void promoteToPrimaryRoot() {
        commsSystem.promoteToPrimaryRoot();
    }

    public boolean isMovingToAssignedPosition() {
        return isMovingToAssignedPosition;
    }

    /** Whether the hard guard refused the most recent motion frame. Overlay/log only. */
    public boolean wasStepVetoed() {
        return lastStepVetoed;
    }

    /**
     * What the avoidance layer is doing to this robot's motion. Overlay and tick log only.
     *
     * <p>Combined from two sources rather than stored: the tick-rate policy says what was
     * <em>planned</em>, and the motion-rate guard says whether the last frame was actually
     * refused. A veto wins, because it is the more recent and more concrete fact — a robot
     * that planned a detour but is being physically stopped is blocked, not detouring.
     */
    public AvoidanceState getAvoidanceState() {
        if (lastStepVetoed) {
            return AvoidanceState.BLOCKED;
        }
        return commsSystem.getPlannedAvoidanceState();
    }

    /** The neighbour currently in this robot's way, or -1. Overlay and tick log only. */
    public int getBlockingObstacleId() {
        return commsSystem.getBlockingObstacleId();
    }

    /** One tick of travel at full speed — the quantity {@code CyclebuilderComms} calls gamma. */
    public static double tickTravel() {
        return TimeStepDiffDrive.MAX_LINEAR_SPEED / TICK_RATE;
    }

    public double getMaxSpeed() {
        return latticeMotionModel.getMaxSpeed();
    }

    // ------------------------------------------------------------
    // Debug
    // ------------------------------------------------------------
    public void dataDump() {
        System.out.printf(
            "[Robot %d] pose=%s target=%s%n",
            robotId,
            pose,
            assignedPosition
        );
    }

    // ------------------------------------------------------------
    // Comparable
    // ------------------------------------------------------------
    public int compareTo(GeometricCycleLatticeRobot other) {
        return Integer.compare(this.robotId, other.robotId);
    }
}
