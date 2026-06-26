package org.robots;

import java.util.ArrayList;
import java.util.Collections;
import java.util.function.DoubleBinaryOperator;

import org.communicationModels.Communicatable;
import org.communicationModels.CycleRole;
import org.communicationModels.CyclebuilderComms;
import org.communicationModels.TrustLevel;
import org.communicationModels.Messages.AbstractMessage;
import org.drawingModels.TriangularModel;
import org.graphs.HexagonLattice;
import org.graphs.LatticeGraph;
import org.graphs.OrientedPoint;
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

    private ArrayList<Edge> edges;
    private ArrayList<GeometricCycleLatticeRobot> neighbors;

    private OrientedPoint assignedPosition;
    private boolean inCongestedArea;

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

        this.edges = new ArrayList<>();
        this.neighbors = new ArrayList<>();

        this.inCongestedArea = false;

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
    public ArrayList<Edge> getEdges() { return edges; }

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
    public void executeTimeStep(double dt) {
        if (inCongestedArea) return;

        switch(getRole()) {
            case CycleRole.root -> {
                commsSystem.makeObservations();
                processMessages();
                commsSystem.broadcastMessage(true);
            }
            case CycleRole.stable -> {
                commsSystem.makeObservations();
                processMessages();
                commsSystem.broadcastMessage(true);
            }
            default -> {
                // 1. Update observations + process incoming messages
                processMessages();

                // 2. Ask comms system for current target
                OrientedPoint target = commsSystem.getAssignedGlobalPosition();
                // 3. Broadcast logic (NEW API)
                boolean atPos = (target != null
                        && pose.distance(target) < MathUtils.EPSILON)
                        && MathUtils.anglesEqual(pose.orientation, target.orientation);
                
                if (atPos) {
                    // Arrived on position AND heading: hold our exact pose. This makes us a static
                    // reference, instead of a parent whose residual rotation keeps sweeping the
                    // children's targets and driving the rotate-to-point jitter.
                    assignedPosition = new OrientedPoint(pose);
                } else if (target != null) {
                    assignedPosition = new OrientedPoint(target);
                }


                commsSystem.makeObservations();

                commsSystem.broadcastMessage(atPos);
            }
        }

        
    }

    // ------------------------------------------------------------
    // Motion
    // ------------------------------------------------------------
    @Override
    public void move(double dt) {
        if (inCongestedArea) {
            boolean done = motionModel.move(pose, dt);
            if (done) {
                inCongestedArea = false;
                assignedPosition = new OrientedPoint(pose);
            }
            return;
        }

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