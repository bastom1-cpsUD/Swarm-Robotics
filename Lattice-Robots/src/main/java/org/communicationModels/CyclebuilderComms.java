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
import org.graphs.HexagonLattice;
import org.graphs.LatticeEdge;
import org.graphs.LatticeGraph;
import org.graphs.OrientedPoint;
import org.graphs.RigidBodyTransformation;
import org.graphs.SquareLattice;
import org.graphs.Vertex;
import org.robots.GeometricCycleLatticeRobot;
import org.simulation.Edge;

public class CyclebuilderComms extends CommunicationSystem {
    private TrustLevel trust;
    
    private static final boolean VERBOSE = true;
    private HashMap<Integer, Boolean> completedCycles;
    private static final LatticeGraph graph = new HexagonLattice();

    // State-Data
    private int stableID;
    private int pendingChildID;
    private boolean hasBeenAssigned;
    private ChainMemberList chainMemberList;
    private GeometricCycleLatticeRobot self;
    private CycleRole role;

    private int assignedVertexID;
    private int assignedOutgoingEdgeID;
    private int originVertexID;
    private int originOutgoingEdgeID;

    // Time-Step Data
    private HashMap<Integer, Observation> observations;


    //MAIN ALGORITHM STEPS
    public CyclebuilderComms(GeometricCycleLatticeRobot self) {
        this.trust = TrustLevel.Friendly;
        this.stableID = -1;
        this.pendingChildID = -1;
        this.chainMemberList = new ChainMemberList();
        this.role = CycleRole.unassigned;
        this.completedCycles = new HashMap<>();
        this.self = self;
        this.observations = new HashMap<>();
        this.incomingMessages = new ConcurrentLinkedQueue<>();

        assignedVertexID = -1;
        assignedOutgoingEdgeID = -1;
        originVertexID = -1;
        originOutgoingEdgeID = -1;
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
                    setAssignedEdge(pm.getAssignedVertexID(), pm.getAssignedOutgoingEdgeID());
                    setOriginEdge(pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
                    chainMemberList = pm.getChainList();
                    self.addEdge(new Edge(self.getRobotId(), chainMemberList.getSenderID()));
                    log("-> became cycleBuilder: edge id=" + getAssignedEdge().getId()
                            + " from vertex " + getAssignedEdge().getFrom().getId()
                            + ", root=" + chainMemberList.getRootID()
                            + ", chain=" + chainMemberList.getIDList());
                } else if(next instanceof PromotionMessage pm) {
                    
                    reset();
                    role = CycleRole.root;
                    stableID = pm.getSenderId();
                    setAssignedEdge(pm.getAssignedVertexID(), pm.getAssignedOutgoingEdgeID());
                    initializeEdgeMap();
                    log("-> promoted to root by robot " + stableID);
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
                    completedCycles.put(sm.getOriginOutgoingEdgeID(), true);
                    log("-> cycle on edge " + sm.getOriginOutgoingEdgeID() + " COMPLETED");
                    } else {
                    log("-> cycle on edge " + sm.getOriginOutgoingEdgeID()+ " failed, will retry");
                    }
                    pendingChildID = -1;
                } else if(next instanceof PositioningMessage pm) {
                    log("-> reached root, reporting SUCCESS, forwarding upstream");
                    forwardSuccessUpstream(pm.getChainList().getSenderID(), pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
                }
                break;
            case CycleRole.stable:
                if(next instanceof PositioningMessage pm) {
                    log("-> reached stable, reporting SUCCESS, forwarding upstream");
                    forwardSuccessUpstream(pm.getChainList().getSenderID(), pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
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
                int targetEdgeID = -1;
                for (Entry<Integer, Boolean> entry : completedCycles.entrySet()) {
                    if (!entry.getValue()) {
                        targetEdgeID = entry.getKey();
                        break;
                    }
                }
                
                if (targetEdgeID == -1) {
                    promoteAdjacentVerticesToRoots();
                    promoteSelfToStable();
                    return;
                }

                LatticeEdge targetEdge = graph.getOutgoingEdgeByID(getCurrentVertex().getId(), targetEdgeID);

                // 2. Cycle does not exist. Build it!
                GeometricCycleLatticeRobot childToBuild = findBestNeighborForEdge(targetEdge);
                ChainMemberList chainList = new ChainMemberList(self.getRobotId());
                if (childToBuild != null) {
                    PositioningMessage pm = new PositioningMessage(self.getRobotId(), childToBuild.getRobotId(), getVertexIDof(targetEdge), getEdgeIDof(targetEdge), getVertexIDof(targetEdge), getEdgeIDof(targetEdge), chainList);                        
                    log("Enqueuing message to robot " + childToBuild.getRobotId());
                    childToBuild.enqueueMessage(pm);
                   self.addEdge(new Edge(self.getRobotId(), childToBuild.getRobotId()));
                    pendingChildID = childToBuild.getRobotId(); // Wait for status
                 }
                if(VERBOSE) {
                    log("Message sent to " + childToBuild.getRobotId());
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
                PositioningMessage pm = new PositioningMessage(self.getRobotId(), child.getRobotId(), getVertexIDof(targetEdge), getEdgeIDof(targetEdge), originVertexID, originOutgoingEdgeID, childChainList);
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

    //MESSAGE-PROCESSING UTIL
    private void forwardSuccessUpstream() {
        GeometricCycleLatticeRobot parent = getNeighborByID(chainMemberList.getSenderID());
        StatusMessage sm = new StatusMessage(self.getRobotId(), parent.getRobotId(), true, originVertexID, originOutgoingEdgeID);
        parent.enqueueMessage(sm);
        reset();
    }

    private void forwardSuccessUpstream(int parentId, int originVertexID, int originEdgeID) {
        GeometricCycleLatticeRobot parent = getNeighborByID(parentId);
        StatusMessage sm = new StatusMessage(self.getRobotId(), parentId, true, originVertexID, originEdgeID);
        parent.enqueueMessage(sm);
        // Root/stable stays in role but must free the pending lock
        this.pendingChildID = -1;
    }

    private void forwardFailureUpstream() {
        GeometricCycleLatticeRobot parent = getNeighborByID(chainMemberList.getSenderID());
        StatusMessage sm = new StatusMessage(self.getRobotId(), parent.getRobotId(), false, originVertexID, originOutgoingEdgeID);
        parent.enqueueMessage(sm);
        reset();
    }

    //ROOT-RELATED UTIL
    private void initializeEdgeMap() {
        Vertex myVertex = getCurrentVertex();
        ArrayList<LatticeEdge> edges = graph.getOutgoingEdges(myVertex);
        for(LatticeEdge edge : edges) {
            completedCycles.put(getEdgeIDof(edge), false);
        }
    }

    private void promoteAdjacentVerticesToRoots() {
        Vertex myVertex = getCurrentVertex();
        ArrayList<LatticeEdge> edges = graph.getOutgoingEdges(myVertex);
        
        for(LatticeEdge edge : edges) {
            GeometricCycleLatticeRobot neighbor = findBestNeighborForEdge(edge);
            neighbor.enqueueMessage(new PromotionMessage(self.getRobotId(), neighbor.getRobotId(), getVertexIDof(edge), getEdgeIDof(edge)));
        }
    }

    //ASSIGNMENT-RELATED UTIL

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
    
        ArrayList<Observation> validObservations = new ArrayList<>();
        int rootID = chainMemberList.isEmpty() ? -1 : chainMemberList.getRootID();
        for (Observation obs : observations.values()) {
            int robotID = obs.getId();
            if (robotID == rootID || !chainMemberList.isInList(robotID)) {
                validObservations.add(obs);
            }
        }

        int bestNeighborID = -1;
        double smallestDistance = Double.MAX_VALUE;
        for (Observation obs : validObservations) {
            double distance = targetLocal.distance(obs.getLocalPosition());
            if (distance < smallestDistance) {
                
                smallestDistance = distance;
                bestNeighborID = obs.getId();
                
            }
        }

        return getNeighborByID(bestNeighborID);
    }

    private OrientedPoint getTargetInLocalCoordinates(LatticeEdge edge) {
        return edge.getToPos();
    }

    /**
     * EDGE AND VERTEX UTIL
     */
    private LatticeEdge inferNextEdge() {
        LatticeEdge assignedEdge = getAssignedEdge();
        if (assignedEdge.isNull()) return null;

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

    private Vertex getCurrentVertex() {
        LatticeEdge assignedEdge = getAssignedEdge();

        if(assignedEdge.isNull()) {
            return graph.getPrimaryVertex();
        }

        return assignedEdge.getTo();
    }

    private LatticeEdge getAssignedEdge() {
        if(assignedVertexID == -1 || assignedOutgoingEdgeID == -1) {
            return new LatticeEdge();
        }
        return graph.getOutgoingEdgeByID(assignedVertexID, assignedOutgoingEdgeID);
    }

    public LatticeEdge getOriginEdge() {
        if(originVertexID == -1 || originOutgoingEdgeID == -1) {
            return new LatticeEdge();
        }

        return graph.getOutgoingEdgeByID(originVertexID, originOutgoingEdgeID);
    }

    public void setAssignedEdge(int vertexID, int edgeID) {
        assignedVertexID = vertexID;
        assignedOutgoingEdgeID = edgeID;
    }

    public void setOriginEdge(int vertexID, int edgeID) {
        originVertexID = vertexID;
        originOutgoingEdgeID = edgeID;
    }

    public int getVertexIDof(LatticeEdge e) {
        return e.getFrom().getId();
    }

    public int getEdgeIDof(LatticeEdge e) {
        return e.getId();
    }

    //STATE CHANGE UTIL

    public void promoteToPriamaryRoot() {
        role = CycleRole.root;
        initializeEdgeMap();

    }

    private void promoteSelfToStable() {
        role = CycleRole.stable;
        this.pendingChildID = -1;

        //Add more stuff as needed
    }

    //ACCESSORS, MUTATORS, AND UTIL ----------------------------------------------------
    public CycleRole   getRole()         { return role; }
    public boolean     isRoot()          { return role == CycleRole.root; }
    public boolean     isStable()        { return role == CycleRole.stable; }
    public boolean     isCycleBuilder()  { return role == CycleRole.cycleBuilder; }
    public boolean     isUnassigned()    { return role == CycleRole.unassigned; }

    public TrustLevel  getTrustLevel()             { return trust; }
    public void        setTrustLevel(TrustLevel t) { this.trust = t; }

    public OrientedPoint getAssignedGlobalPosition() {
        if (role == CycleRole.root || role == CycleRole.stable) {
            return self.getPosition();
        }

        LatticeEdge assignedEdge = getAssignedEdge();

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
        this.observations = new HashMap<>();

        this.assignedVertexID = -1;
        this.assignedOutgoingEdgeID = -1;

        this.originVertexID = -1;
        this.originOutgoingEdgeID = -1;
    }

    private void log(String msg) {
        if (VERBOSE) {
            System.out.println("[Robot " + self.getRobotId() + " | " + role + "] " + msg);
        }
    }
    
}
