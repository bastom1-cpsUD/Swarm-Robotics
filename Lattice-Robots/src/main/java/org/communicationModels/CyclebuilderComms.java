package org.communicationModels;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

import org.communicationModels.Messages.AbstractMessage;
import org.communicationModels.Messages.ChainMemberList;
import org.communicationModels.Messages.PositioningMessage;
import org.communicationModels.Messages.PromotionMessage;
import org.communicationModels.Messages.StatusMessage;
import org.communicationModels.Messages.VerificationMessage;
import org.communicationModels.Messages.VerificationResponseMessage;
import org.graphs.HexagonLattice;
import org.graphs.LatticeEdge;
import org.graphs.LatticeGraph;
import org.graphs.OrientedPoint;
import org.graphs.RigidBodyTransformation;
import org.graphs.Vertex;
import org.robots.GeometricCycleLatticeRobot;
import org.simulation.Edge;

public class CyclebuilderComms extends CommunicationSystem {
    private TrustLevel trust;
    private int stableID;
    private int pendingChildID;
    

    private HashMap<LatticeEdge, Boolean> completedCycles;
    private static final LatticeGraph graph = new HexagonLattice();

    private boolean isPrimaryRoot;

    private CycleRole role;
    private LatticeEdge originEdge;
    private LatticeEdge assignedEdge;
    private ChainMemberList chainMemberList;
    private GeometricCycleLatticeRobot self;
    private HashMap<Integer, Observation> observations;


    public CyclebuilderComms(GeometricCycleLatticeRobot self) {
        this.trust = TrustLevel.Friendly;
        this.stableID = -1;
        this.pendingChildID = -1;
        this.chainMemberList = new ChainMemberList();
        this.isPrimaryRoot = false;
        this.role = CycleRole.unassigned;
        this.completedCycles = new HashMap<>();
        this.originEdge = new LatticeEdge();
        this.assignedEdge = new LatticeEdge();
        this.self = self;
        this.observations = new HashMap<>();
    }

    public void makeObservations() {
        ArrayList<GeometricCycleLatticeRobot> neighbors = self.getNeighbors();
        observations.clear();
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
                    assignedEdge = pm.getCurrentEdge();
                    originEdge = pm.getOriginEdge();
                    chainMemberList = pm.getChainList();
                    self.addEdge(new Edge(self.getRobotId(), chainMemberList.getSenderID()));
                } else if(next instanceof VerificationMessage vm) {
                    role = CycleRole.verifying;
                    assignedEdge = vm.getCurrentEdge();
                    originEdge =  vm.getCycleOrigin();
                    chainMemberList = vm.getChainList();
                } else if(next instanceof PromotionMessage pMessage) {
                    role = CycleRole.root;
                    reset();
                    stableID = pMessage.getSenderId();
                    initializeEdgeMap();
                }
                break;
            case CycleRole.verifying:
                if(next instanceof VerificationResponseMessage vrm && vrm.isSuccessful()) {
                    pendingChildID = -1;
                    forwardPositiveVerificationUpstream();
                } else if(next instanceof VerificationResponseMessage vrm && !vrm.isSuccessful()) {
                    pendingChildID = -1;
                    forwardNegativeVerificationUpstream();
                }
                break;
            case cycleBuilder:
                if (next instanceof StatusMessage sm && sm.isSuccessful()) {
                    pendingChildID = -1;
                    forwardSuccessUpstream();
                } else if (next instanceof StatusMessage sm && !sm.isSuccessful()) {
                    pendingChildID = -1;
                    forwardFailureUpstream();
                }
                break;
            case CycleRole.root:
                if(next instanceof StatusMessage sm) {
                    if (sm.isSuccessful()) {
                       completedCycles.put(sm.getCycleOrigin(), true);
                    }
                    // Free the pending child lock whether it was success or failure
                    pendingChildID = -1;
                } else if(next instanceof VerificationResponseMessage vrm) {
                    if(vrm.isSuccessful()) {
                        // The entire cycle verification was successful
                        completedCycles.put(vrm.getCycleOrigin(), true);
                    }
                    // Free lock to try the next step or retry building
                    pendingChildID = -1; 
                }
                break;
            case CycleRole.stable:
                break;
        }
    }

    public void broadcastMessage(boolean alreadyInPosition) {
        switch (role) {
            case CycleRole.root: {
                if(pendingChildID != -1) {
                    return; // Wait until current verification or building is complete
                }

                // Find the first outgoing edge that doesn't have a completed cycle yet
                LatticeEdge targetEdge = null;
                for (Entry<LatticeEdge, Boolean> entry : completedCycles.entrySet()) {
                    if (!entry.getValue()) {
                        targetEdge = entry.getKey();
                        break;
                    }
                }
                
                if (targetEdge == null) {
                    promoteAdjacentVerticesToRoots();
                }
                /** 
                // 1. Try to verify: Is there a neighbor at the edge's target position?
                GeometricCycleLatticeRobot occupyingNeighbor = findNeighborAt(targetEdge.getToPos());

                if (occupyingNeighbor != null) {
                    // Send a VerificationMessage down the edge
                    ChainMemberList chainList = new ChainMemberList(self.getRobotId());
                    VerificationMessage vm = new VerificationMessage(self.getRobotId(), occupyingNeighbor.getRobotId(), targetEdge, targetEdge, chainList);
                    occupyingNeighbor.enqueueMessage(vm);
                    pendingChildID = occupyingNeighbor.getRobotId(); // Wait for response
                } else { */
                    // 2. Cycle does not exist. Build it!
                    GeometricCycleLatticeRobot childToBuild = findBestNeighborForEdge(targetEdge);
                    ChainMemberList chainList = new ChainMemberList(self.getRobotId());
                    if (childToBuild != null) {
                        PositioningMessage pm = new PositioningMessage(self.getRobotId(), childToBuild.getRobotId(), targetEdge, targetEdge,chainList);
                        childToBuild.enqueueMessage(pm);
                        self.addEdge(new Edge(self.getRobotId(), childToBuild.getRobotId()));
                        pendingChildID = childToBuild.getRobotId(); // Wait for status
                    }
                
                break;
            }
            case CycleRole.verifying: {
                if(pendingChildID != -1) {
                    return;
                }
                LatticeEdge targetEdge = inferNextEdge();

                OrientedPoint targetInLocalCoordinates = getTargetInLocalCoordinates(targetEdge);

                GeometricCycleLatticeRobot child = findNeighborAt(targetInLocalCoordinates);

                if(child == null) {
                    forwardNegativeVerificationUpstream();
                } else if(child.getRobotId() == chainMemberList.getRootID()) {
                    //Send verification true
                    forwardPositiveVerificationUpstream();
                } else {
                    //Send verification message
                    ChainMemberList childChainList = new ChainMemberList(chainMemberList, self.getRobotId());
                    VerificationMessage vm = new VerificationMessage(self.getRobotId(), child.getRobotId(), originEdge, targetEdge, childChainList);
                    child.enqueueMessage(vm);
                }
                break;
            }
            case CycleRole.cycleBuilder: {
                if(pendingChildID != -1) {
                    return;
                }        
                if(!alreadyInPosition) {
                    break;
                }

                LatticeEdge targetEdge = inferNextEdge();

                GeometricCycleLatticeRobot child = findBestNeighborForEdge(targetEdge);

                if(child == null) {
                    forwardFailureUpstream();
                    break;
                } else if(child.getRobotId() == chainMemberList.getRootID()) {
                    forwardSuccessUpstream();
                    break;
                }
                ChainMemberList childChainList = new ChainMemberList(chainMemberList, self.getRobotId());

                PositioningMessage pm = new PositioningMessage(self.getRobotId(), child.getRobotId(), targetEdge, originEdge, childChainList);

                child.enqueueMessage(pm);
                break;
            }
            default:
                break;
        }
    }

    

    private void promoteAdjacentVerticesToRoots() {
        Vertex myVertex = getCurrentVertex();
        ArrayList<LatticeEdge> edges = graph.getOutgoingEdges(myVertex);
        
        for(LatticeEdge edge : edges) {
            GeometricCycleLatticeRobot neighbor = findBestNeighborForEdge(edge);
            neighbor.enqueueMessage(new PromotionMessage(self.getRobotId(), neighbor.getRobotId()));
        }
    }

    private void initializeEdgeMap() {
        Vertex myVertex = getCurrentVertex();
        ArrayList<LatticeEdge> edges = graph.getOutgoingEdges(myVertex);
        for(LatticeEdge edge : edges) {
            completedCycles.put(edge, false);
        }
    }

    private GeometricCycleLatticeRobot getNeighborByID(int robotID) {
        for(GeometricCycleLatticeRobot neighbor : self.getNeighbors()) {
            if(neighbor.getRobotId() == robotID) {
                return neighbor;
            }
        }
        return null;
    }

    private GeometricCycleLatticeRobot findBestNeighborForEdge(LatticeEdge targetEdge) {
        OrientedPoint target = getTargetInLocalCoordinates(targetEdge);
        ArrayList<Observation> validObservations = new ArrayList<>();

        for(Observation obs : observations.values()) {
            int robotID = obs.getId();
            if(!chainMemberList.isInList(robotID)) {
                validObservations.add(obs);
            }
        }

        int bestNeighborID = -1;
        double smallestDistance = Double.MAX_VALUE;
        for(Observation obs : validObservations) {
            double distance = target.distance(obs.getLocalPosition());
            if(distance < smallestDistance) {
                smallestDistance = distance;
                bestNeighborID = obs.getId();
            }
        }

        return getNeighborByID(bestNeighborID);
    }

    /**
     * Helper method to find a neighbor currently sitting at the target local position.
     */
    private GeometricCycleLatticeRobot findNeighborAt(OrientedPoint targetPose) {
        for (Observation obs : observations.values()) {
            // Using equals() for exact matches. Note: If using continuous physics, 
            // you may want to change this to calculate distance < epsilon.
            if (obs.getLocalPosition().equals(targetPose)) {
                for (GeometricCycleLatticeRobot neighbor : self.getNeighbors()) {
                    if (neighbor.getRobotId() == obs.getId()) {
                        return neighbor;
                    }
                }
            }
        }
        return null;
    }

    private OrientedPoint getTargetInLocalCoordinates(LatticeEdge edge) {
        return edge.getEdgeTransformation().apply(new OrientedPoint(0,0,0));
    }

    private void forwardPositiveVerificationUpstream() {
        GeometricCycleLatticeRobot parent = getNeighborByID(chainMemberList.getSenderID());
        VerificationResponseMessage vrm = new VerificationResponseMessage(self.getRobotId(), parent.getRobotId(), chainMemberList.getRootID(), originEdge, true);
        parent.enqueueMessage(vrm);
        reset();
    }

    private void forwardNegativeVerificationUpstream() {
        GeometricCycleLatticeRobot parent = getNeighborByID(chainMemberList.getSenderID());
        VerificationResponseMessage vrm = new VerificationResponseMessage(self.getRobotId(), parent.getRobotId(), chainMemberList.getRootID(), originEdge, false);
        parent.enqueueMessage(vrm);
        reset();
    }

    private void forwardSuccessUpstream() {
        GeometricCycleLatticeRobot parent = getNeighborByID(chainMemberList.getSenderID());
        StatusMessage sm = new StatusMessage(self.getRobotId(), parent.getRobotId(), true, originEdge);
        parent.enqueueMessage(sm);
        reset();
    }

    private void forwardFailureUpstream() {
        GeometricCycleLatticeRobot parent = getNeighborByID(chainMemberList.getSenderID());
        StatusMessage sm = new StatusMessage(self.getRobotId(), parent.getRobotId(), false, originEdge);
        parent.enqueueMessage(sm);
        reset();
    }

    public LatticeEdge inferNextEdge() {
        Vertex v = getCurrentVertex();
        ArrayList<LatticeEdge> edges = graph.getOutgoingEdges(v);

        if (assignedEdge == null) return null;

        int idx = -1;

        for (int i = 0; i < edges.size(); i++) {
            if (edges.get(i).getId() == assignedEdge.getId()) {
                idx = i;
                break;
            }
        }

        if (idx == -1) return null;

        int nextIdx = (idx + 1) % edges.size();

        return edges.get(nextIdx);
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
        initializeEdgeMap();

    }

    //ACCESSORS ----------------------------------------------------
    public CycleRole   getRole()         { return role; }
    public boolean     isRoot()          { return role == CycleRole.root; }
    public boolean     isStable()        { return role == CycleRole.stable; }
    public boolean     isCycleBuilder()  { return role == CycleRole.cycleBuilder; }
    public boolean     isUnassigned()    { return role == CycleRole.unassigned; }
 
    public LatticeEdge getAssignedEdge() { return assignedEdge; }
 
    public TrustLevel  getTrustLevel()             { return trust; }
    public void        setTrustLevel(TrustLevel t) { this.trust = t; }

    public OrientedPoint getAssignedGlobalPosition() {
        if (role == CycleRole.root || role == CycleRole.stable) {
            return self.getPosition();
        }
        if (assignedEdge.isNull()) return null;
 
        GeometricCycleLatticeRobot parent = getNeighborByID(chainMemberList.getSenderID());
        if (parent == null) return null;
 
        return new RigidBodyTransformation(parent.getPosition())
                .apply(assignedEdge.getToPos());
    }

    public void reset() {
        this.stableID = -1;
        this.pendingChildID = -1;
        this.chainMemberList = new ChainMemberList();
        this.isPrimaryRoot = false;
        this.role = CycleRole.unassigned;
        this.completedCycles = new HashMap<>();
        this.originEdge = new LatticeEdge();
        this.assignedEdge = new LatticeEdge();
        this.observations = new HashMap<>();
    }
    
}
