package org.robots;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import org.communicationModels.Communicatable;
import org.communicationModels.DecentralizedComms;
import org.communicationModels.Message;
import org.communicationModels.TrustLevel;
import org.drawingModels.TriangularModel;

import org.graphs.OrientedPoint;
import org.motionModels.LatticeMotionModel;
import org.motionModels.TimeStepDiffDrive;
import org.simulation.Edge;

public class LatticeRobot extends Robot implements Communicatable {

    //Local knowledge & edges
    private DecentralizedComms commsSystem;
    private Set<Edge> edges;
    private ArrayList<LatticeRobot> neighbors;

    private final LatticeMotionModel latticeMotionModel;
    public final static double COMM_RANGE = 75.0;
    private OrientedPoint assignedPosition;
    private boolean inCongestedArea;

    public LatticeRobot(int id, OrientedPoint pose) {
        super(id, pose, new TimeStepDiffDrive(), new TriangularModel());
        this.latticeMotionModel = (LatticeMotionModel) motionModel;
        this.commsSystem = new DecentralizedComms(id, this);
        this.edges = new HashSet<>();
        this.neighbors = new ArrayList<>();
        this.inCongestedArea = false;
    }

    public void setTrustLevel(TrustLevel trust) {
        commsSystem.setTrustLevel(trust);   
    }

    public void addNeighbor(LatticeRobot other) {
        this.neighbors.add(other);

    }

    public void removeNeighbor(LatticeRobot neighbor) {
        this.neighbors.remove(neighbor);

    }

    public void addEdge(Edge edge) {
        this.edges.add(edge);
    }

    public void clearEdges() {
        this.edges.clear();
    }

    public void clearNeighbors() {
        this.neighbors.clear();
    }

    public Set<Edge> getEdges() {
        return Collections.unmodifiableSet(edges);
    }

    public LatticeMotionModel getLatticeMotionModel() {
        return latticeMotionModel;
    }

    public TrustLevel getTrustLevel() {
        return commsSystem.getTrustLevel();
    }

    /** {@inheritDoc} */
    @Override
    public void enqueueMessage(Message msg) {
        commsSystem.enqueueMessage(msg);
    }

    /** {@inheritDoc} */
    @Override
    public void processMessages() {
        commsSystem.syncPeers(neighbors);
        commsSystem.processMessages();
    }
    
    /**
     * Executes a timestep of the robot's algorithmic behavior, which includes local communication, task assignment, the broadcasting of the assignment, and movement.
     * @param timeStep the duration of the time step to be executed
     */
    public void executeTimeStep(double timeStep) {
        if(!inCongestedArea) {
            processMessages();
            commsSystem.broadcastAssignment();
            //Retrieve assignment from previous time step
            OrientedPoint[] assignment = commsSystem.retrieveAssignmentLocation();
            OrientedPoint parentPose = assignment[0];
            OrientedPoint newAssignment = assignment[1];

            //If you are a root, stay in place via new assignment
            if(commsSystem.isRoot()) {
                assignedPosition = newAssignment;
                return;
            }
            //If you are unassigned, begin run away procedure
            if(!commsSystem.isAssigned()) {
                inCongestedArea = true;
                return;
            }

            //retrieve the intermediate point between target position and parent
            OrientedPoint newIntermediatePose = latticeMotionModel.getIntermediatePose(this.pose, parentPose, newAssignment, timeStep);

            //Adopt intermediate position if it is a significant difference away from 
            if(assignedPosition == null || newIntermediatePose.distanceTo(assignedPosition) > TimeStepDiffDrive.ASSIGNMENT_CHANGE_THRESHOLD) {
                assignedPosition = newIntermediatePose;
                motionModel.startMoving();
            } else {
                assignedPosition = newIntermediatePose; // update quietly without resetting motion
            }
            commsSystem.resetCommunicationState();
        }     
    }
    
    /** {@inheritDoc} */
    @Override
    public void move(double dt) {
        if(inCongestedArea == true) {
            boolean completedRepositionMovement = motionModel.move(pose,dt);
            if(completedRepositionMovement) {
                inCongestedArea = false;
                assignedPosition = this.pose;
                commsSystem.resetCommunicationState();
            }
            return;
        }
        latticeMotionModel.moveTo(pose, assignedPosition, dt);
        
    }
}