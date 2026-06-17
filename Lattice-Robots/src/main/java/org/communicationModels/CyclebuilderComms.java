package org.communicationModels;

import java.util.ArrayList;
import java.util.HashMap;

import org.communicationModels.Messages.AbstractMessage;
import org.communicationModels.Messages.PositioningMessage;
import org.communicationModels.Messages.PromotionMessage;
import org.graphs.HexagonLattice;
import org.graphs.LatticeEdge;
import org.graphs.LatticeGraph;
import org.graphs.RigidBodyTransformation;
import org.robots.GeometricCycleLatticeRobot;

public class CyclebuilderComms extends CommunicationSystem {
    private TrustLevel trust;

    private int parentID;
    private int rootID;
    private ArrayList<Integer> expectingReponseFromID;

    private LatticeGraph graph = new HexagonLattice();

    private CycleRole role;
    private LatticeEdge assignedEdge;
    private GeometricCycleLatticeRobot self;
    private HashMap<Integer, Observation> observations;


    public CyclebuilderComms(GeometricCycleLatticeRobot self) {
        this.trust = TrustLevel.Friendly;
        this.parentID = -1;
        this.expectingReponseFromID = new ArrayList<>();
        this.role = CycleRole.unassigned;
        this.assignedEdge = new LatticeEdge();
        this.self = self;
        this.observations = new HashMap<>();
    }

    @Override
    public void processMessages() {
        if(incomingMessages.isEmpty()) {
            return;
        }
        switch(role) {
            case CycleRole.unassigned:
                    AbstractMessage next = incomingMessages.remove();
                    if(next instanceof PositioningMessage) {
                        role = CycleRole.cycleBuilder;
                        PositioningMessage message = (PositioningMessage) next;
                        parentID = message.getSenderId();
                        assignedEdge = message.getCurrentEdge();
                        rootID = message.getRootId();
                    }
                    if(next instanceof PromotionMessage) {
                        role = CycleRole.root;
                    }
                break;
            case CycleRole.cycleBuilder:
                    
                break;
            case CycleRole.root:
                
                break;
            case CycleRole.stable:
                break;
        }
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'processMessages'");
    }



    public void makeObservations() {
        ArrayList<GeometricCycleLatticeRobot> neighbors = self.getNeighbors();
        if(neighbors == null || neighbors.isEmpty()) {
            observations = new HashMap<>();
            return;
        }

        RigidBodyTransformation globalToLocal = new RigidBodyTransformation(self.getPosition()).inverse();

        for(GeometricCycleLatticeRobot neighbor : neighbors) {
            Observation obs = new Observation(neighbor, globalToLocal);
            observations.put(neighbor.getRobotId(), obs);
        }
    }
    
}
