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

import org.motionModels.TimeStepDiffDrive;
import org.simulation.Edge;

public class LatticeRobot extends Robot implements Communicatable {

    //Local knowledge & edges
    private DecentralizedComms commsSystem;
    private Set<Edge> edges;
    private ArrayList<LatticeRobot> neighbors;

    public LatticeRobot(int id, OrientedPoint pose) {
        super(id, pose, new TimeStepDiffDrive(), new TriangularModel());
        this.commsSystem = new DecentralizedComms(id, this);
        this.edges = new HashSet<>();
        this.neighbors = new ArrayList<>();
    }

    public void setTrustLevel(TrustLevel trust) {
        commsSystem.setTrustLevel(trust);   
    }

    public void addNeighbor(LatticeRobot other) {
        //Check if edge already exists to prevent duplicates
        boolean edgeExists = this.edges.stream().anyMatch(edge -> edge.getToId() == other.getRobotId());

        if(!edgeExists) {
            this.edges.add(new Edge(this.getRobotId(), other.getRobotId()));
            other.edges.add(new Edge(other.getRobotId(), this.getRobotId()));
            this.neighbors.add(other);
            other.neighbors.add(this);
        }
    }

    public void removeNeighbor(LatticeRobot neighbor) {
        neighbor.edges.removeIf(edge -> edge.getToId() == this.getRobotId());
        this.edges.removeIf(edge -> edge.getToId() == neighbor.getRobotId());
        this.neighbors.remove(neighbor);
        neighbor.neighbors.remove(this);
    }

    public Set<Edge> getEdges() {
        return Collections.unmodifiableSet(edges);
    }

    public TrustLevel getTrustLevel() {
        return commsSystem.getTrustLevel();
    }

    @Override
    public void enqueueMessage(Message msg) {
        commsSystem.enqueueMessage(msg);
    }

    @Override
    public void processMessages() {
        commsSystem.processMessages();
    }
}


