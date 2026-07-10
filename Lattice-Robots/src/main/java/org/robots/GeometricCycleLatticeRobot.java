package org.robots;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

import org.communicationModels.Communicatable;
import org.communicationModels.CycleRole;
import org.communicationModels.CyclebuilderComms;
import org.communicationModels.TrustLevel;
import org.communicationModels.Messages.AbstractMessage;
import org.drawingModels.TriangularModel;
import org.graphs.HexagonLattice;
import org.graphs.LatticeGraph;
import org.graphs.OrientedPoint;
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

    // ------------------------------------------------------------
    // Fields
    // ------------------------------------------------------------
    private final CyclebuilderComms commsSystem;
    private final LatticeMotionModel latticeMotionModel;

    private CopyOnWriteArrayList<Edge> edges;
    private ArrayList<GeometricCycleLatticeRobot> neighbors;

    private OrientedPoint assignedPosition;
    private boolean isMovingToAssignedPosition = false;

    // ------------------------------------------------------------
    // Constructor
    // ------------------------------------------------------------
    public GeometricCycleLatticeRobot(int id, OrientedPoint pose) {
        this(id, pose, new HexagonLattice());
    }

    public GeometricCycleLatticeRobot(int id, OrientedPoint pose, LatticeGraph graph) {
        super(id, pose, new TimeStepDiffDrive(), new TriangularModel());

        this.latticeMotionModel = (LatticeMotionModel) motionModel;

        // NEW comms system (no graph passed in anymore)
        this.commsSystem = new CyclebuilderComms(this);

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
    public void addEdge(Edge e) { edges.add(e); }
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

    // ------------------------------------------------------------
    // Main time-step
    // ------------------------------------------------------------
    /**
     * Executes one activation of this robot's role-specific control logic
     * and returns a full record of what happened — state immediately before,
     * what message was processed, what action resulted, what was sent, and
     * state immediately after. The role dispatch below intentionally stays
     * here rather than in the simulation panel, since root/stable and
     * cycleBuilder/unassigned activate their comms calls in different
     * orders.
     *
     * @param dt   time step, in seconds, forwarded to downstream motion logic
     * @param tick a logical tick number for this robot's own activation
     *             timeline (e.g. the panel's per-robot tick counter, or a
     *             global step counter during single-step debugging). Purely
     *             informational — does not affect behavior.
     * @return a record of everything that happened this activation, or a
     *         "nothing happened" record if the robot is currently navigating
     *         a congested area (comms are skipped entirely in that case).
     */
    public TickRecord executeTimeStep(double dt, int tick) {
        CommsSnapshot before = commsSystem.snapshot();
        OrientedPoint poseBefore = new OrientedPoint(pose);

        commsSystem.beginTick();
        String processed;
        String action;

        switch(getRole()) {
            case CycleRole.root -> {
                commsSystem.makeObservations();
                processed = commsSystem.processMessages(tick);
                action = commsSystem.broadcastMessage(true, tick);
            }
            case CycleRole.stable -> {
                commsSystem.makeObservations();
                processed = commsSystem.processMessages(tick);
                action = commsSystem.broadcastMessage(true, tick);
            }
            default -> {
                // 1. Process incoming messages
                processed = commsSystem.processMessages(tick);

                // 2. Ask comms system for current target
                OrientedPoint target = commsSystem.getAssignedGlobalPosition();
                // 3. Broadcast logic (NEW API)
                boolean atPos = (target != null
                        && pose.distance(target) < MathUtils.POSITION_EPSILON)
                        && MathUtils.anglesEqual(pose.orientation, target.orientation);

                if (atPos) {
                    // Arrived on position AND heading: hold our exact pose. This makes us a static
                    // reference, instead of a parent whose residual rotation keeps sweeping the
                    // children's targets and driving the rotate-to-point jitter.
                    assignedPosition = new OrientedPoint(pose);
                    isMovingToAssignedPosition = false;
                } else if (target != null) {
                    assignedPosition = new OrientedPoint(target);
                    isMovingToAssignedPosition = true;
                }

                commsSystem.makeObservations();

                action = commsSystem.broadcastMessage(atPos, tick);
            }
        }

        CommsSnapshot after = commsSystem.snapshot();
        OrientedPoint poseAfter = new OrientedPoint(pose);

        return new TickRecord(tick, robotId, before, poseBefore, processed, action,
                commsSystem.sentThisTick(), after, poseAfter);
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
    // Visualization edges (simplified: no old parent logic)
    // ------------------------------------------------------------
    private void updateEdges() {
        clearEdges();

        // Optional: keep only debug visualization if comms exposes role
        // Example (if you add getRole()):
        //
        // if (commsSystem.getRole() == CycleRole.cycleBuilder) { ... }

        // Left intentionally minimal because new comms system
        // handles structure internally via messages
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

    public void promoteToRoot() {
        commsSystem.promoteToPriamaryRoot();
    }

    public void promoteToPrimaryRoot() {
        commsSystem.promoteToPriamaryRoot();
    }

    public boolean isMovingToAssignedPosition() {
        return isMovingToAssignedPosition;
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
