package org.communicationModels;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

import org.communicationModels.Messages.AbstractMessage;
import org.communicationModels.Messages.PositioningMessage;
import org.communicationModels.Messages.PromotionMessage;
import org.communicationModels.Messages.StatusMessage;
import org.communicationModels.Messages.VerificationResponseMessage;
import org.graphs.HexagonLattice;
import org.graphs.LatticeEdge;
import org.graphs.LatticeGraph;
import org.graphs.RigidBodyTransformation;
import org.graphs.Vertex;
import org.robots.GeometricCycleLatticeRobot;

public class CyclebuilderComms extends CommunicationSystem {
    private TrustLevel trust;

    private int parentID;
    private int rootID;
    private int stableID;
    private int pendingChildID;
    private ArrayList<Integer> chainList;

    private HashMap<LatticeEdge, Boolean> completedCycles;
    private LatticeGraph graph = new HexagonLattice();

    private boolean isPrimaryRoot;
    private CycleRole role;
    private LatticeEdge assignedEdge;
    private GeometricCycleLatticeRobot self;
    private HashMap<Integer, Observation> observations;


    public CyclebuilderComms(GeometricCycleLatticeRobot self) {
        this.trust = TrustLevel.Friendly;
        this.parentID = -1;
        this.rootID = -1;
        this.stableID = -1;
        this.isPrimaryRoot = false;
        this.role = CycleRole.unassigned;
        this.completedCycles = new HashMap<>();
        this.assignedEdge = new LatticeEdge();
        this.self = self;
        this.observations = new HashMap<>();
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

    @Override
    public void processMessages() {
        if(incomingMessages.isEmpty()) {
            return;
        }
        AbstractMessage next = incomingMessages.poll();
        switch(role) {
            case CycleRole.unassigned:
                if(next instanceof PositioningMessage pm) {
                    role = CycleRole.cycleBuilder;
                    parentID = pm.getSenderId();
                    assignedEdge = pm.getCurrentEdge();
                    rootID = pm.getRootId();
                }
                break;
            case cycleBuilder:
                if (next instanceof StatusMessage sm && sm.isSuccessful()) {
                    pendingChildID = -1;
                    forwardSuccessToParent();
                } else if (next instanceof StatusMessage sm && !sm.isSuccessful()) {
                    pendingChildID = -1;
                    forwardFailureToParent();
                }  else if(next instanceof PromotionMessage pMessage) {
                    role = CycleRole.root;
                    stableID = pMessage.getSenderId();
                    initializeEdgeMap();
                }
                break;
            case CycleRole.root:
                if(next instanceof StatusMessage sm && sm.isSuccessful()) {
                    
                } else if(next instanceof StatusMessage sm && !sm.isSuccessful()) {
                    //Do nothing if failure
                } else if(next instanceof VerificationResponseMessage vrm && vrm.isSuccessful()) {

                }

                
                break;
            case CycleRole.stable:
                break;
        }
    }

    public void broadcastMessage(boolean alreadyInPosition) {
        switch (role) {
            case CycleRole.root:
                if(pendingChildID != -1) {
                    return;
                }

                Vertex myVertex = getCurrentVertex();

                
                break;
            case CycleRole.cycleBuilder:
                if(pendingChildID != -1) {
                    return;
                }        
                if(!alreadyInPosition) {
                    break;
                }

                 LatticeEdge targetEdge = inferNextEdge();
 
                if (targetEdge == null) {
                    // Traversal exhausted — this robot closes the cycle.
                    closeOrFailCycle();
                    return;
                }

                GeometricCycleLatticeRobot child = findBestNeighborForEdge(targetEdge);

                if(child == null) {

                    forwardFailureToParent();
                }

                PositioningMessage pm = new PositioningMessage(self.getRobotId(), child.getRobotId(), targetEdge, rootID, null);

                child.enqueueMessage(pm);
                break;
            default:
                break;
        }
    }

    public void confirmNextEdge() {

    }

    public void initializeEdgeMap() {
        Vertex myVertex = getCurrentVertex();
        ArrayList<LatticeEdge> edges = graph.getOutgoingEdges(myVertex);
        for(LatticeEdge edge : edges) {
            completedCycles.put(edge, false);
        }
    }

    public GeometricCycleLatticeRobot findBestNeighborForEdge(LatticeEdge targetEdge) {
        return null;
    }

    public void forwardSuccessToParent() {

    }

    public void forwardFailureToParent() {

    }

    public LatticeEdge inferNextEdge() {
        return null;
    }

    public void closeOrFailCycle() {

    }
    public Vertex getCurrentVertex() {

        if(role != CycleRole.root) {
            return assignedEdge.getTo();
        }

        if(isPrimaryRoot) {
            return graph.getPrimaryVertex();
        }

        return assignedEdge.getTo();
    }

    public void promoteToPriamaryRoot() {
        isPrimaryRoot = true;
        role = CycleRole.root;

    }



    
    
}
