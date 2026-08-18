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
     * Robot activations per second. One activation is one <em>tick</em>: a single phase
     * plus the motion that follows it. A full time step is two ticks -- phase one
     * (message passing) then phase two (assignment reconciliation).
     */
    public static final double TICK_RATE = 1.0;
    public static final double TIME_STEP = 0.5 * TICK_RATE;
    public static final VoltageGraph GRAPH = SnubSquareVoltageGraph.build();
    // ------------------------------------------------------------
    // Fields
    // ------------------------------------------------------------
    private final CyclebuilderComms commsSystem;
    private final LatticeMotionModel latticeMotionModel;

    private CopyOnWriteArrayList<Edge> edges;
    private ArrayList<GeometricCycleLatticeRobot> neighbors;

    private OrientedPoint assignedPosition;
    private boolean isMovingToAssignedPosition = false;

    /**
     * Which half of the time step the next activation runs. A time step is two ticks:
     * phase one passes protocol messages and hands out assignments, phase two reconciles
     * them against what neighbours have declared they are heading for.
     */
    private boolean inPhaseOne = true;

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
     * Executes one activation — one <em>tick</em>, which is one phase plus the motion
     * that follows it — and returns a full record of what happened: state immediately
     * before, what message was processed, what action resulted, what was sent, and state
     * immediately after.
     *
     * <p>Two ticks make a time step. <b>Phase one</b> is the protocol: process a message,
     * observe, hand out an assignment. <b>Phase two</b> reconciles those assignments — it
     * re-observes and checks whether any neighbour has declared it is heading for the
     * same spot this robot is. Splitting them keeps assignment and reconciliation from
     * racing inside a single activation.
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

        String processed;
        String action;

        if (inPhaseOne) {
            switch(getRole()) {
                case CycleRole.root -> {
                    commsSystem.makeFirstPhaseObservations();
                    processed = commsSystem.processMessages(tick);
                    action = commsSystem.sendMessage(true, tick);
                }
                case CycleRole.stable -> {
                    commsSystem.makeFirstPhaseObservations();
                    processed = commsSystem.processMessages(tick);
                    action = commsSystem.sendMessage(true, tick);
                }
                default -> {
                    commsSystem.detectAssignmentContention(commsSystem.makeFirstPhaseObservations());
                    
                    // 1. Process incoming messages
                    processed = commsSystem.processMessages(tick);

                    // 2. Ask comms system for current target, 3. broadcast
                    action = commsSystem.sendMessage(updateAssignedPosition(), tick);
                }
            }

            // Emitted every activation rather than only in phase two. Robots activate
            // staggered and asynchronously, so one robot's phase two routinely lands during a
            // neighbour's phase one; gating emission on phase would silently drop claims to
            // that drift.
            commsSystem.broadcastTargetClaim();
        } else {
            // PHASE TWO — reconcile. No assignments are handed out here; the only message
            // this can emit is a rejection to a parent when this robot yields a contested
            // spot, which is ordinary protocol traffic.
            processed = "N/A (phase two)";
            
            String contention = commsSystem.detectAssignmentContention(commsSystem.makeSecondPhaseObservations());

            // After contention, not before: a yield clears the assignment, and this is
            // what stops the robot from continuing toward a spot it just gave up.
            updateAssignedPosition();

            // Emitted every activation rather than only in phase two. Robots activate
            // staggered and asynchronously, so one robot's phase two routinely lands during a
            // neighbour's phase one; gating emission on phase would silently drop claims to
            // that drift.
            commsSystem.broadcastTargetClaim();

            action = contention != null ? contention : "N/A (no contention)";
        }

        inPhaseOne = !inPhaseOne;

        CommsSnapshot after = commsSystem.snapshot();
        OrientedPoint poseAfter = new OrientedPoint(pose);

        return new TickRecord(tick, robotId, before, poseBefore, processed, action,
                commsSystem.sentThisTick(), after, poseAfter);
    }

    /**
     * Points this robot at whatever its comms system currently considers its target, and
     * reports whether it is already there.
     *
     * <p>Called from both phases: phase one because a message may have just changed the
     * assignment, phase two because contention resolution may have just removed it.
     *
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
            //No live assignment (e.g. we just became unassigned due target being occupied)
            assignedPosition = new OrientedPoint(pose);
            isMovingToAssignedPosition = false;
        }

        return atPos;
    }

    // ------------------------------------------------------------
    // Motion
    // ------------------------------------------------------------
    @Override
    public void move(double dt) {
        if (assignedPosition != null) {
            latticeMotionModel.moveTo(pose, assignedPosition, dt);
        }
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
