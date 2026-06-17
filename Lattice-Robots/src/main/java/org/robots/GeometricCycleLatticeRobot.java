package org.robots;

import java.util.ArrayList;
import java.util.Collections;

import org.communicationModels.Communicatable;
import org.communicationModels.CyclebuilderComms;
import org.communicationModels.TrustLevel;
import org.communicationModels.Messages.AbstractMessage;
import org.drawingModels.TriangularModel;
import org.graphs.OrientedPoint;
import org.motionModels.LatticeMotionModel;
import org.motionModels.TimeStepDiffDrive;
import org.simulation.Edge;

public class GeometricCycleLatticeRobot extends Robot implements Communicatable{

    //Local knowledge & edges
    private CyclebuilderComms commsSystem;
    private ArrayList<Edge> edges;
    private ArrayList<GeometricCycleLatticeRobot> neighbors;

    private final LatticeMotionModel latticeMotionModel;
    public final static double COMM_RANGE = 75.0;
    private OrientedPoint assignedPosition;
    private boolean inCongestedArea;

    
    public GeometricCycleLatticeRobot(int id, OrientedPoint pose) {
        super(id, pose, new TimeStepDiffDrive(), new TriangularModel());
        this.latticeMotionModel = (LatticeMotionModel) motionModel;
        this.commsSystem = null;
        this.edges = new ArrayList<>();
        this.neighbors = new ArrayList<>();
        this.inCongestedArea = false;
    }

    public void setTrustLevel(TrustLevel trust) {
        //MUST IMPLEMENT  
    }

    public void addNeighbor(GeometricCycleLatticeRobot other) {
        this.neighbors.add(other);

    }

    public void removeNeighbor(GeometricCycleLatticeRobot neighbor) {
        this.neighbors.remove(neighbor);

    }

    public ArrayList<GeometricCycleLatticeRobot> getNeighbors() {
        return new ArrayList<>(Collections.unmodifiableList(neighbors));
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

    public ArrayList<Edge> getEdges() {
        return edges;
    }

    public LatticeMotionModel getLatticeMotionModel() {
        return latticeMotionModel;
    }

    public TrustLevel getTrustLevel() {
        return null;
    }

    /** {@inheritDoc} */
    @Override
    public void enqueueMessage(AbstractMessage msg) {
    }

    /** {@inheritDoc} */
    @Override
    public void processMessages() {

    }
    
    /**
     * Executes a timestep of the robot's algorithmic behavior, which includes local communication, task assignment, the broadcasting of the assignment, and movement.
     * @param timeStep the duration of the time step to be executed
     */
    public void executeTimeStep(double timeStep) {
        
    }
    
    /** {@inheritDoc} */
    @Override
    public void move(double dt) {
       
    }

    public int compareTo(GeometricCycleLatticeRobot robot) {
        if(this.robotId < robot.getRobotId()) {
            return -1;
        } else if (this.robotId > robot.getRobotId()) {
            return 1;
        } else {
            return 0;
        }
    }

    public void dataDump() {
        
    }

    
}
