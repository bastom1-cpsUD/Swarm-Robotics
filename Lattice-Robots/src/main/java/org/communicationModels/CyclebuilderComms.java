package org.communicationModels;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.PriorityBlockingQueue;

import org.communicationModels.Messages.AbstractMessage;
import org.communicationModels.Messages.ChainMemberList;
import org.communicationModels.Messages.PositioningMessage;
import org.communicationModels.Messages.PromotionMessage;
import org.communicationModels.Messages.RejectAssignmentMessage;
import org.communicationModels.Messages.StatusMessage;
import org.graphs.HexagonLattice;
import org.graphs.LatticeEdge;
import org.graphs.LatticeGraph;
import org.graphs.OrientedPoint;
import org.graphs.RigidBodyTransformation;
import org.graphs.Vertex;
import org.robots.GeometricCycleLatticeRobot;
import org.simulation.Edge;
import org.utils.MathUtils;
import org.utils.logging.CommsSnapshot;
import org.utils.logging.OutgoingMessageRecord;

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
    private ArrayList<Integer> unableToDoAssignmentIDs;

    private int assignedVertexID;
    private int assignedOutgoingEdgeID;
    private int originVertexID;
    private int originOutgoingEdgeID;

    private boolean hasFailed;

    // Time-Step Data
    private HashMap<Integer, Observation> observations;

    // Logging / instrumentation support (see CommsSnapshot, org.logging package)
    private ArrayList<OutgoingMessageRecord> sentThisTick;

    //MAIN ALGORITHM STEPS
    public CyclebuilderComms(GeometricCycleLatticeRobot self) {
        this.trust = TrustLevel.Friendly;
        this.stableID = -1;
        this.pendingChildID = -1;
        this.hasBeenAssigned = false;
        this.chainMemberList = new ChainMemberList();
        this.role = CycleRole.unassigned;
        this.completedCycles = new HashMap<>();
        this.self = self;
        this.observations = new HashMap<>();
        this.incomingMessages = new PriorityBlockingQueue<>();
        this.unableToDoAssignmentIDs = new ArrayList<>();
        this.hasFailed = false;
        this.sentThisTick = new ArrayList<>();

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
        processMessages(-1);
    }

    /**
     * Canonical message-processing implementation. Pops (at most) one message
     * from the priority queue, applies the role-specific state transition,
     * and returns a human-readable description of what happened — used both
     * for the VERBOSE console log and for HtmlLog-based tick logging.
     *
     * <p>{@code tick} is accepted purely for description text / caller
     * context; it does not affect behavior. Pass {@code -1} when no tick
     * context is available.</p>
     */
    public String processMessages(int tick) {
        if(incomingMessages.isEmpty()) {
            return "N/A (EMPTY)";
        }

        AbstractMessage peek = incomingMessages.peek();
        // Root/stable must always accept a closing PositioningMessage — don't gate on pendingChildID
        boolean bypassPendingGate = (role == CycleRole.root || role == CycleRole.stable)
                && peek instanceof PositioningMessage;

        if (!bypassPendingGate && pendingChildID != -1 && peek.getSenderId() != pendingChildID) {
            incomingMessages.add(incomingMessages.poll());
            return "N/A (Waiting for pending child " + pendingChildID + ")";
        }

        AbstractMessage next = incomingMessages.poll();
        log("Received " + next.getMessageType() + " from robot " + next.getSenderId());

        switch(role) {
            case CycleRole.unassigned:
                if(next instanceof PositioningMessage pm) {
                    if(hasBeenAssigned) {
                        log("Checking for formation-breaking assignment...");
                        if(!checkAssignmentForCurrentPosition(pm)) {
                            forwardRejectionUpstream(pm);
                            log("-> assignment REJECTED by " + pm.getSenderId());
                            return "Positioning Message from " + pm.getSenderId() + "(REJECTED)";
                        }
                    }

                    role = CycleRole.cycleBuilder;
                    hasBeenAssigned = true;
                    setAssignedEdge(pm.getAssignedVertexID(), pm.getAssignedOutgoingEdgeID());
                    setOriginEdge(pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
                    unableToDoAssignmentIDs.add(pm.getSenderId());
                    chainMemberList = pm.getChainList();
                    self.addEdge(new Edge(self.getRobotId(), chainMemberList.getSenderID()));
                    log("-> became cycleBuilder: edge id=" + getAssignedEdge().getId()
                            + " from vertex " + getAssignedEdge().getFrom().getId()
                            + ", root=" + chainMemberList.getRootID()
                            + ", chain=" + chainMemberList.getIDList());
                    return "Positioning Message from " + pm.getSenderId() + "(ACCEPTED)";
                } else if(next instanceof PromotionMessage pm) {
                    reset();
                    role = CycleRole.root;
                    stableID = pm.getSenderId();
                    setAssignedEdge(pm.getAssignedVertexID(), pm.getAssignedOutgoingEdgeID());
                    initializeEdgeMap();
                    log("-> promoted to root by robot " + stableID);
                    return "Promotion Message from " + pm.getSenderId() + "(ACCEPTED)";
                } else {
                    log("-> unassigned robot received unexpected message type: " + next.getMessageType());
                    return "N/A (Unhandled message type: " + next.getMessageType() + ")";
                }
            case cycleBuilder:
                if (next instanceof StatusMessage sm && sm.isSuccessful()) {
                    pendingChildID = -1;
                    log("-> child reported SUCCESS, forwarding upstream");
                    forwardSuccessUpstream();
                    return "Status Message from " + sm.getSenderId() + "(SUCCESS)";
                } else if (next instanceof StatusMessage sm && !sm.isSuccessful()) {
                    pendingChildID = -1;
                    log("-> child reported FAILURE, forwarding upstream");
                    forwardFailureUpstream();
                    return "Status Message from " + sm.getSenderId() + "(FAILURE)";
                } else if(next instanceof RejectAssignmentMessage rm) {
                    pendingChildID = -1;
                    log("-> assignment REJECTED by " + rm.getSenderId());
                    unableToDoAssignmentIDs.add(rm.getSenderId());
                    return "Reject Assignment Message from " + rm.getSenderId() + "(REJECTED)";
                } else if(next instanceof PromotionMessage pm) {
                    forwardSuccessUpstream();
                    reset();
                    role = CycleRole.root;
                    stableID = pm.getSenderId();
                    setAssignedEdge(pm.getAssignedVertexID(), pm.getAssignedOutgoingEdgeID());
                    initializeEdgeMap();
                    log("-> promoted to root by robot " + stableID);
                    return "Promotion Message from " + pm.getSenderId() + "(ACCEPTED, re-promoted mid-build)";
                } else if(next instanceof PositioningMessage pm) {
                    // If the sender is our pending child, we can accept the message and process it normally.
                    if(pm.getSenderId() == pendingChildID) {
                        
                    }
                    // Already a cycleBuilder — a second parent is racing to assign us.
                    // Don't accept, don't reject: just defer. Re-queue untouched and
                    // let it be reconsidered on a future tick (e.g. once we free up
                    // via forwardSuccessUpstream/forwardFailureUpstream -> reset()).
                    incomingMessages.add(pm);
                    log("-> already assigned, deferring Positioning Message from " + pm.getSenderId());
                    return "Positioning Message from " + pm.getSenderId() + " (DEFERRED - already cycleBuilder)";
                } else {
                    log("-> cycleBuilder received unexpected message type: " + next.getMessageType());
                    return "N/A (Unhandled message type: " + next.getMessageType() + ")";
                }
            case CycleRole.root:
                if(next instanceof StatusMessage sm) {
                    pendingChildID = -1;
                    if (sm.isSuccessful()) {
                        completedCycles.put(sm.getOriginOutgoingEdgeID(), true);
                        log("-> cycle on edge " + sm.getOriginOutgoingEdgeID() + " COMPLETED");
                        return "Status Message from " + sm.getSenderId() + "(SUCCESS)";
                    } else {
                        hasFailed = true;
                        log("-> cycle on edge " + sm.getOriginOutgoingEdgeID() + " FAILED, ceasing operations");
                        return "Status Message from " + sm.getSenderId() + "(FAILURE)";
                    }
                } else if(next instanceof RejectAssignmentMessage rm) {
                    pendingChildID = -1;
                    unableToDoAssignmentIDs.add(rm.getSenderId());
                    log("-> assignment REJECTED by " + rm.getSenderId());
                    return "Reject Assignment Message from " + rm.getSenderId() + "(REJECTED)";
                } else if(next instanceof PositioningMessage pm) {
                    log("-> reached root, reporting SUCCESS, forwarding upstream");
                    forwardSuccessUpstream(pm.getChainList().getSenderID(), pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
                    return "Positioning Message from " + pm.getSenderId() + "(ACCEPTED)";
                } else {
                    log("-> root received unexpected message type: " + next.getMessageType());
                    return "N/A (Unhandled message type: " + next.getMessageType() + ")";
                }
            case CycleRole.stable:
                if(next instanceof PositioningMessage pm) {
                    log("-> reached stable, reporting SUCCESS, forwarding upstream");
                    forwardSuccessUpstream(pm.getChainList().getSenderID(), pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
                    return "Positioning Message from " + pm.getSenderId() + "(ACCEPTED)";
                } else {
                    log("-> stable received unexpected message type: " + next.getMessageType());
                    return "N/A (Unhandled message type: " + next.getMessageType() + ")";
                }
        }

        return "N/A (Unhandled message type)";
    }

    public void broadcastMessage(boolean alreadyInPosition) {
        broadcastMessage(alreadyInPosition, -1);
    }

    /**
     * Canonical broadcast/assignment implementation. Decides, based on role,
     * whether to assign a child, forward success/failure, promote neighbors,
     * or do nothing this tick — and returns a human-readable description of
     * the resulting action.
     *
     * <p>{@code tick} is accepted purely for description text / caller
     * context; it does not affect behavior. Pass {@code -1} when no tick
     * context is available.</p>
     */
    public String broadcastMessage(boolean alreadyInPosition, int tick) {
        switch (role) {
            case CycleRole.root: {
                if(pendingChildID != -1) {
                    return "N/A (Waiting for Status Update to complete)";
                } else if(hasFailed) {
                    return "N/A (Ceased operations due to failure)";
                }
                log("Sending Message...");

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
                    return "Done (All cycles completed, promoted to stable)";
                }

                LatticeEdge targetEdge = retrieveEdgeFromGraph(getCurrentVertex().getId(), targetEdgeID);

                // 2. Cycle does not exist. Build it!
                GeometricCycleLatticeRobot childToBuild = findBestNeighborForEdge(targetEdge);
                ChainMemberList chainList = new ChainMemberList(self.getRobotId());
                if (childToBuild != null) {
                    PositioningMessage pm = new PositioningMessage(self.getRobotId(), childToBuild.getRobotId(), getVertexIDof(targetEdge), getEdgeIDof(targetEdge), getVertexIDof(targetEdge), getEdgeIDof(targetEdge), chainList);
                    send(childToBuild, pm);
                    self.addEdge(new Edge(self.getRobotId(), childToBuild.getRobotId()));
                    pendingChildID = childToBuild.getRobotId(); // Wait for status
                } else {
                    log("Ran out of options for cycles... Ceasing operations...");
                    hasFailed = true;
                    return "Failed (No valid neighbors for cycle, ceasing operations)";
                }
                if(VERBOSE) {
                    log("Message sent to " + childToBuild.getRobotId());
                }
                return "Assigned position to robot " + childToBuild.getRobotId()
                        + " for edge " + targetEdge.getId() + " of vertex " + targetEdge.getFrom().getId();
            }
            case CycleRole.cycleBuilder: {
                if(pendingChildID != -1) {
                    return "N/A (Waiting for Status Update to complete)";
                }
                if(!alreadyInPosition) {
                    return "N/A (Not in position to broadcast)";
                }
                if(VERBOSE) {
                    log("Sending Message...");
                }

                LatticeEdge targetEdge = inferNextEdge();

                GeometricCycleLatticeRobot child = findBestNeighborForEdge(targetEdge);

                if(child == null) {
                    log("-> NO candidate found, forwarding FAILURE upstream");
                    forwardFailureUpstream();
                    return "Reporting Failure (No candidate found)";
                } else if(child.getRobotId() == chainMemberList.getRootID()) {
                    log("-> Succesffuly found root, closing chain");
                    forwardSuccessUpstream();
                    return "Reporting Success (Found root, closing chain)";
                }
                log("-> assigning robot " + child.getRobotId() + " to edge " + targetEdge.getId());
                ChainMemberList childChainList = new ChainMemberList(chainMemberList, self.getRobotId());
                PositioningMessage pm = new PositioningMessage(self.getRobotId(), child.getRobotId(), getVertexIDof(targetEdge), getEdgeIDof(targetEdge), originVertexID, originOutgoingEdgeID, childChainList);
                send(child, pm);
                self.addEdge(new Edge(self.getRobotId(), child.getRobotId()));
                pendingChildID = child.getRobotId();
                return "Assigned position to robot " + child.getRobotId() + " for edge " + targetEdge.getId() + " of vertex " + targetEdge.getFrom().getId();
            }
            case CycleRole.unassigned:
                return "N/A (Unassigned robot do not broadcast)";
            default:
                return "N/A (Unhandled)";
        }
    }

    //MESSAGE-PROCESSING UTIL
    private void forwardSuccessUpstream() {
        GeometricCycleLatticeRobot parent = getNeighborByID(chainMemberList.getSenderID());
        StatusMessage sm = new StatusMessage(self.getRobotId(), parent.getRobotId(), true, originVertexID, originOutgoingEdgeID);
        send(parent, sm);
        reset();
    }

    private void forwardSuccessUpstream(int parentId, int originVertexID, int originEdgeID) {
        GeometricCycleLatticeRobot parent = getNeighborByID(parentId);
        StatusMessage sm = new StatusMessage(self.getRobotId(), parentId, true, originVertexID, originEdgeID);
        send(parent, sm);
    }

    private void forwardFailureUpstream() {
        GeometricCycleLatticeRobot parent = getNeighborByID(chainMemberList.getSenderID());
        StatusMessage sm = new StatusMessage(self.getRobotId(), parent.getRobotId(), false, originVertexID, originOutgoingEdgeID);
        send(parent, sm);
        reset();
    }

    private void forwardRejectionUpstream(PositioningMessage pm) {
        RejectAssignmentMessage rm = new RejectAssignmentMessage(pm.getRecipient(), pm.getSenderId(), pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
        GeometricCycleLatticeRobot robot = getNeighborByID(pm.getSenderId());
        send(robot, rm);
    }

    private boolean checkAssignmentForCurrentPosition(PositioningMessage pm) {
        OrientedPoint localAssignment = getAssignedLocalPosition(pm);
        log("Assigned local position is: " + localAssignment);

        OrientedPoint positionInLocal = new RigidBodyTransformation(self.getPosition()).inverse().apply(self.getPosition());

        return MathUtils.approxEquals(localAssignment.x, positionInLocal.getX(), MathUtils.POSITION_EPSILON)
                && MathUtils.approxEquals(localAssignment.y, positionInLocal.getY(), MathUtils.POSITION_EPSILON)
                && MathUtils.isZero(MathUtils.angleDifference(localAssignment.getOrientation(), positionInLocal.getOrientation()));
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
            PromotionMessage pm = new PromotionMessage(self.getRobotId(), neighbor.getRobotId(), getVertexIDof(edge), getEdgeIDof(edge));
            send(neighbor, pm);
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
        OrientedPoint targetLocal  = getTargetInLocalCoordinates(targetEdge);
            log("Beginning decision process");

        int rootID = chainMemberList.isEmpty() ? -1 : chainMemberList.getRootID();

        // Root is only a candidate if it's not banned/parent, and check it separately from the rest
        Observation rootObs = observations.get(rootID);
        if (rootObs != null && !unableToDoAssignmentIDs.contains(rootID)) {
            double rootDistance = targetLocal.distance(rootObs.getLocalPosition());
            if (MathUtils.isZero(rootDistance, MathUtils.POSITION_EPSILON)) {
                log("-> root is exactly at target position, closing cycle");
                return getNeighborByID(rootID);
            }
        }

        // Root excluded from here on — never lets a near-but-not-exact root suppress a real candidate
        ArrayList<Observation> validObservations = new ArrayList<>();
        for (Observation obs : observations.values()) {
            int robotID = obs.getId();
            if (robotID != rootID && !chainMemberList.isInList(robotID) && !unableToDoAssignmentIDs.contains(robotID)) {
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

        return getNeighborByID(bestNeighborID); // null if truly none — genuine dead end
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

    public LatticeEdge getAssignedEdge() {
        if(assignedVertexID == -1 || assignedOutgoingEdgeID == -1) {
            return new LatticeEdge();
        }
        return retrieveEdgeFromGraph(assignedVertexID, assignedOutgoingEdgeID);
    }

    public LatticeEdge getOriginEdge() {
        if(originVertexID == -1 || originOutgoingEdgeID == -1) {
            return new LatticeEdge();
        }

        return retrieveEdgeFromGraph(originVertexID, originOutgoingEdgeID);
    }

    private LatticeEdge retrieveEdgeFromGraph(int vertexID, int edgeID) {
        return graph.getOutgoingEdgeByID(vertexID, edgeID);
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

    public boolean hasFailed() { return hasFailed; }

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

    private OrientedPoint getAssignedLocalPosition(PositioningMessage pm) {
        LatticeEdge assignedEdge = retrieveEdgeFromGraph(pm.getAssignedVertexID(), pm.getAssignedOutgoingEdgeID());

        if(assignedEdge.isNull()) return null;

        GeometricCycleLatticeRobot parent = getNeighborByID(pm.getSenderId());

        OrientedPoint assignedGlobalPosition = new RigidBodyTransformation(parent.getPosition()).apply(assignedEdge.getToPos());

        return new RigidBodyTransformation(self.getPosition()).inverse().apply(assignedGlobalPosition);
    }

    public void reset() {
        this.pendingChildID = -1;
        this.chainMemberList = new ChainMemberList();
        this.role = CycleRole.unassigned;
        this.observations = new HashMap<>();
        unableToDoAssignmentIDs.clear();

        this.assignedVertexID = -1;
        this.assignedOutgoingEdgeID = -1;

        this.originVertexID = -1;
        this.originOutgoingEdgeID = -1;
    }

    //LOGGING / SNAPSHOT SUPPORT ---------------------------------------------

    /**
     * Clears the record of messages sent since the last call. Must be called
     * once at the very start of each robot activation (before processMessages
     * / broadcastMessage run for that tick) so that {@link #sentThisTick()}
     * reflects only messages sent during the current tick.
     */
    public void beginTick() {
        sentThisTick.clear();
    }

    /**
     * Returns the messages this comms system has enqueued to other robots
     * since the last {@link #beginTick()} call.
     */
    public List<OutgoingMessageRecord> sentThisTick() {
        return List.copyOf(sentThisTick);
    }

    /**
     * Captures a read-only, defensively-copied snapshot of this comms
     * system's current state. Cheap enough to call twice per tick (once
     * before dispatch, once after) for before/after diffing.
     */
    public CommsSnapshot snapshot() {
        return new CommsSnapshot(
                role,
                trust,
                hasFailed,
                pendingChildID,
                stableID,
                chainMemberList,
                getAssignedEdge(),
                getOriginEdge(),
                Map.copyOf(completedCycles),
                snapshotQueueInOrder(),
                Map.copyOf(observations),
                List.copyOf(unableToDoAssignmentIDs)
        );
    }

    private List<AbstractMessage> snapshotQueueInOrder() {
        // PriorityBlockingQueue's own iterator makes no ordering guarantee,
        // so copy out and sort a snapshot instead of trusting iteration order.
        List<AbstractMessage> copy = new ArrayList<>(incomingMessages);
        Collections.sort(copy);
        return copy;
    }

    /**
     * Enqueues a message to a recipient and records it for logging. Every
     * outgoing send in this class should go through here rather than calling
     * {@code recipient.enqueueMessage(...)} directly, so
     * {@link #sentThisTick()} stays a complete and accurate audit trail.
     */
    private void send(GeometricCycleLatticeRobot recipient, AbstractMessage msg) {
        recipient.enqueueMessage(msg);
        sentThisTick.add(new OutgoingMessageRecord(
                recipient.getRobotId(), msg.getMessageType(), String.valueOf(msg)));
    }

    private void log(String msg) {
        if (VERBOSE) {
            System.out.println("[Robot " + self.getRobotId() + " | " + role + "] " + msg);
        }
    }

}