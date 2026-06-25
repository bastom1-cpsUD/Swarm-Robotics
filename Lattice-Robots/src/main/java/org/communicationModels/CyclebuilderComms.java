package org.communicationModels;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.concurrent.ConcurrentLinkedQueue;

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
    
    private static final boolean VERBOSE = true;
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
        this.incomingMessages = new ConcurrentLinkedQueue<>();
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
            if(role == CycleRole.stable) {
                log("EMPTY");
            }
            return;
        } 

        if(self.getRobotId() == 37 && role == CycleRole.stable) {
            log("I have " + incomingMessages.size() + " to handle");
        } 

        AbstractMessage peek = incomingMessages.peek();
        // Root/stable must always accept a closing PositioningMessage — don't gate on pendingChildID
        boolean bypassPendingGate = (role == CycleRole.root || role == CycleRole.stable)
                && peek instanceof PositioningMessage;

        if(VERBOSE) {
            if(bypassPendingGate) {
                log("Bypassing pending gate as a " + role);
            } else {    
                log("Cannot bypass pending gate");
            }
        }
        if (!bypassPendingGate && pendingChildID != -1 && peek.getSenderId() != pendingChildID) {
            incomingMessages.add(incomingMessages.poll());
            return;
        }
        
        AbstractMessage next = incomingMessages.poll();
        log("Received " + next.getMessageType() + " from robot " + next.getSenderId());
        switch(role) {
            case CycleRole.unassigned:
                if(next instanceof PositioningMessage pm) {
                    role = CycleRole.cycleBuilder;
                    assignedEdge = pm.getCurrentEdge();
                    originEdge = pm.getOriginEdge();
                    chainMemberList = pm.getChainList();
                    self.addEdge(new Edge(self.getRobotId(), chainMemberList.getSenderID()));
                    log("-> became cycleBuilder: edge id=" + assignedEdge.getId()
                            + " from vertex " + assignedEdge.getFrom().getId()
                            + ", root=" + chainMemberList.getRootID()
                            + ", chain=" + chainMemberList.getIDList());
                } else if(next instanceof VerificationMessage vm) {
                    role = CycleRole.verifying;
                    assignedEdge = vm.getCurrentEdge();
                    originEdge =  vm.getCycleOrigin();
                    chainMemberList = vm.getChainList();
                    log("-> became verifying: edge id=" + assignedEdge.getId());
                } else if(next instanceof PromotionMessage pm) {
                    
                    reset();
                    role = CycleRole.root;
                    stableID = pm.getSenderId();
                    this.assignedEdge = pm.getCurrentEdge();
                    initializeEdgeMap();
                    log("-> promoted to root by robot " + stableID);
                }
                break;
            case CycleRole.verifying:
                if(next instanceof VerificationResponseMessage vrm && vrm.isSuccessful()) {
                    pendingChildID = -1;
                    log("-> downstream verification succeeded, forwarding positive upstream");
                    forwardPositiveVerificationUpstream();
                } else if(next instanceof VerificationResponseMessage vrm && !vrm.isSuccessful()) {
                    pendingChildID = -1;
                    log("-> downstream verification failed, forwarding negative upstream");
                    forwardNegativeVerificationUpstream();
                }
                break;
            case cycleBuilder:
                if (next instanceof StatusMessage sm && sm.isSuccessful()) {
                    pendingChildID = -1;
                    log("-> child reported SUCCESS, forwarding upstream");
                    forwardSuccessUpstream();
                } else if (next instanceof StatusMessage sm && !sm.isSuccessful()) {
                    pendingChildID = -1;
                    log("-> child reported FAILURE, forwarding upstream");
                    forwardFailureUpstream();
                }
                break;
            case CycleRole.root:
                if(next instanceof StatusMessage sm) {
                    if (sm.isSuccessful()) {
                    completedCycles.put(sm.getCycleOrigin(), true);
                    log("-> cycle on edge " + sm.getCycleOrigin().getId() + " COMPLETED");
                    } else {
                    log("-> cycle on edge " + sm.getCycleOrigin().getId() + " failed, will retry");
                    }
                    pendingChildID = -1;
                } else if(next instanceof VerificationResponseMessage vrm) {
                    if(vrm.isSuccessful()) {
                        completedCycles.put(vrm.getCycleOrigin(), true);
                        log("-> verification confirmed cycle on edge " + vrm.getCycleOrigin().getId());
                    }
                    pendingChildID = -1; 
                } else if(next instanceof PositioningMessage pm) {
                    log("-> reached root, reporting SUCCESS, forwarding upstream");
                    forwardSuccessUpstream(pm.getChainList().getSenderID(), pm.getOriginEdge());
                }
                break;
            case CycleRole.stable:
                if(next instanceof PositioningMessage pm) {
                    log("-> reached stable, reporting SUCCESS, forwarding upstream");
                    forwardSuccessUpstream(pm.getChainList().getSenderID(), pm.getOriginEdge());
                }
            break;
        }
    }
    

    public void broadcastMessage(boolean alreadyInPosition) {
       
        switch (role) {
            case CycleRole.root: {
                if(pendingChildID != -1) {
                    return; // Wait until current verification or building is complete
                }
                if(VERBOSE) {
                     log("Sending Message...");
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
                    promoteSelfToStable();
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
                        log("enqueuing message to robot " + childToBuild.getRobotId());
                        childToBuild.enqueueMessage(pm);
                        self.addEdge(new Edge(self.getRobotId(), childToBuild.getRobotId()));
                        pendingChildID = childToBuild.getRobotId(); // Wait for status
                    }
                    if(VERBOSE) {
                    log("Message sent to " + childToBuild.getRobotId());
                }
                
                break;
            }
            case CycleRole.verifying: {
                if(pendingChildID != -1) {
                    return;
                }
                if(VERBOSE) {
                     log("Sending Message...");
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
                    pendingChildID = child.getRobotId();
                }
                if(VERBOSE) {
                    log("Message sent!");
                }
                break;
            }
            case CycleRole.cycleBuilder: {
                if(pendingChildID != -1) {
                    return;
                }        
                if(!alreadyInPosition) {
                    log("Not in the position to broadcast...");
                    break;
                }
                if(VERBOSE) {
                     log("Sending Message...");
                }

                LatticeEdge targetEdge = inferNextEdge();
                
                GeometricCycleLatticeRobot child = findBestNeighborForEdge(targetEdge);

                if(child == null) {
                    log("-> NO candidate found, forwarding FAILURE upstream");
                    forwardFailureUpstream();
                    break;
                } else if(child.getRobotId() == chainMemberList.getRootID()) {
                    log("-> Succesffuly found root, closing chain");
                    forwardSuccessUpstream();
                    break;
                }
                log("-> assigning robot " + child.getRobotId() + " to edge " + targetEdge.getId());
                ChainMemberList childChainList = new ChainMemberList(chainMemberList, self.getRobotId());
                PositioningMessage pm = new PositioningMessage(self.getRobotId(), child.getRobotId(), targetEdge, originEdge, childChainList);
                child.enqueueMessage(pm);
                self.addEdge(new Edge(self.getRobotId(), child.getRobotId()));
                pendingChildID = child.getRobotId();
                if(VERBOSE) {
                    log("Message sent!");
                }
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
            neighbor.enqueueMessage(new PromotionMessage(self.getRobotId(), neighbor.getRobotId(), edge));
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
        makeObservations();
        OrientedPoint targetLocal  = getTargetInLocalCoordinates(targetEdge);
            log("Beginning decision process");
        // Recover global coordinates for logging — apply self's local-to-global transform
        RigidBodyTransformation localToGlobal = new RigidBodyTransformation(self.getPosition());
        OrientedPoint targetGlobal = localToGlobal.apply(targetLocal);

        ArrayList<Observation> validObservations = new ArrayList<>();
        int rootID = chainMemberList.isEmpty() ? -1 : chainMemberList.getRootID();
        for (Observation obs : observations.values()) {
            int robotID = obs.getId();
            if (robotID == rootID || !chainMemberList.isInList(robotID)) {
                validObservations.add(obs);
                OrientedPoint candidateGlobal = localToGlobal.apply(obs.getLocalPosition());
                log("Included robot " + robotID + " in consideration"
                        + " — global pos: (" + String.format("%.1f", candidateGlobal.x)
                        + ", " + String.format("%.1f", candidateGlobal.y) + ")");
            } else {
                log("Discarded robot " + robotID + " from consideration (already in chain)");
            }
        }

        int bestNeighborID = -1;
        double smallestDistance = Double.MAX_VALUE;
        for (Observation obs : validObservations) {
            double distance = targetLocal.distance(obs.getLocalPosition());
            OrientedPoint candidateGlobal = localToGlobal.apply(obs.getLocalPosition());
            if (distance < smallestDistance) {
                if (bestNeighborID != -1) {
                    log("Robot " + bestNeighborID + " displaced by robot " + obs.getId()
                            + " — robot " + obs.getId()
                            + " global pos: (" + String.format("%.1f", candidateGlobal.x)
                            + ", " + String.format("%.1f", candidateGlobal.y) + ")"
                            + ", distance to target: " + String.format("%.2f", distance)
                            + " < " + String.format("%.2f", smallestDistance));
                }
                smallestDistance = distance;
                bestNeighborID = obs.getId();
                log("New best candidate: robot " + obs.getId()
                        + " — global pos: (" + String.format("%.1f", candidateGlobal.x)
                        + ", " + String.format("%.1f", candidateGlobal.y) + ")"
                        + ", distance to target global pos ("
                        + String.format("%.1f", targetGlobal.x) + ", "
                        + String.format("%.1f", targetGlobal.y) + "): "
                        + String.format("%.2f", distance));
            } else {
                log("Candidate robot " + obs.getId() + " denied"
                        + " — global pos: (" + String.format("%.1f", candidateGlobal.x)
                        + ", " + String.format("%.1f", candidateGlobal.y) + ")"
                        + ", distance to target global pos ("
                        + String.format("%.1f", targetGlobal.x) + ", "
                        + String.format("%.1f", targetGlobal.y) + "): "
                        + String.format("%.2f", distance)
                        + " (current best: robot " + bestNeighborID
                        + " at " + String.format("%.2f", smallestDistance) + ")");
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
        return edge.getToPos();
    }

    private void log(String msg) {
        if (VERBOSE) {
            System.out.println("[Robot " + self.getRobotId() + " | " + role + "] " + msg);
        }
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

    private void forwardSuccessUpstream(int parentId, LatticeEdge sentEdge) {
        GeometricCycleLatticeRobot parent = getNeighborByID(parentId);
        StatusMessage sm = new StatusMessage(self.getRobotId(), parentId, true, sentEdge);
        parent.enqueueMessage(sm);
        // Root/stable stays in role but must free the pending lock
        this.pendingChildID = -1;
    }

    private void forwardFailureUpstream() {
        GeometricCycleLatticeRobot parent = getNeighborByID(chainMemberList.getSenderID());
        StatusMessage sm = new StatusMessage(self.getRobotId(), parent.getRobotId(), false, originEdge);
        parent.enqueueMessage(sm);
        reset();
    }

    public LatticeEdge inferNextEdge() {
        if (assignedEdge == null || assignedEdge.isNull()) return null;

        Vertex currentVertex = getCurrentVertex();          // where this robot now sits
        Vertex incomingFromType = assignedEdge.getFrom();    // sublattice the assignment came from

        ArrayList<LatticeEdge> candidateEdges = graph.getOutgoingEdges(currentVertex);
        if (candidateEdges.isEmpty()) return null;

        int incomingId = assignedEdge.getId();

        // HexagonLattice: "v1" sublattice carries Vertex ID 1, "v2" carries ID 2.
        // Leaving v1 keeps the same edge id (continue the perimeter rotation);
        // leaving v2 advances to the next id, wrapping 3 -> 1.
        int nextId = (incomingFromType.getId() == 1)
                ? incomingId
                : (incomingId % candidateEdges.size()) + 1;

        for (LatticeEdge edge : candidateEdges) {
            if (edge.getId() == nextId) {
                return edge;
            }
        }

        return null;
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

    private void promoteSelfToStable() {
        role = CycleRole.stable;
        this.pendingChildID = -1;

        //Add more stuff as needed
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
        this.pendingChildID = -1;
        this.chainMemberList = new ChainMemberList();
        this.role = CycleRole.unassigned;
        this.originEdge = new LatticeEdge();
        this.assignedEdge = new LatticeEdge();
        this.observations = new HashMap<>();
    }
    
}
