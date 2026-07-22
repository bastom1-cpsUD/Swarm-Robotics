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
import org.graphs.util.OrientedPoint;
import org.graphs.util.RigidBodyTransformation;
import org.graphs.voltage.HalfEdge;
import org.graphs.voltage.HexagonVoltageGraph;
import org.graphs.voltage.OctagonSquareVoltageGraph;
import org.graphs.voltage.Role;
import org.graphs.voltage.VoltageGraph;
import org.robots.GeometricCycleLatticeRobot;
import org.simulation.Edge;
import org.utils.MathUtils;
import org.utils.logging.CommsSnapshot;
import org.utils.logging.OutgoingMessageRecord;

public class CyclebuilderComms extends CommunicationSystem {
    private TrustLevel trust;

    private static final boolean VERBOSE = true;
    private HashMap<Integer, Boolean> completedCycles;
    private final VoltageGraph graph;

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

    public CyclebuilderComms(GeometricCycleLatticeRobot self, VoltageGraph graph) {
        this.graph = graph;
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
        boolean childHasLeft = true;
        for(GeometricCycleLatticeRobot neighbor : neighbors) {
            if(neighbor.getRobotId() == pendingChildID) {
                childHasLeft = false;
            }
            Observation obs = new Observation(neighbor, globalToLocal);
            observations.put(neighbor.getRobotId(), obs);
        }

        if(childHasLeft && pendingChildID != -1) {
            log("-> child " + pendingChildID + " has left, clearing pendingChildID");
            removeEdgeToChild(pendingChildID);
            pendingChildID = -1;
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
                && (peek instanceof PositioningMessage || peek instanceof StatusMessage || peek instanceof RejectAssignmentMessage);

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
                            forwardRejectionUpstream(pm, false);
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
                            + " from vertex " + getAssignedEdge().getOrigin().getId()
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
                    removeEdgeToChild(rm.getSenderId());
                    log("-> assignment REJECTED by " + rm.getSenderId());
                    if(rm.isRetryable()) {
                        forwardRejectionToParent(true);
                        reset();
                        log("-> assignment is retryable, will attempt to reassign");
                    } else {
                        unableToDoAssignmentIDs.add(rm.getSenderId());
                        log("-> assignment is NOT retryable, will not attempt to reassign");
                    }
                    return "Reject Assignment Message from " + rm.getSenderId() + "(REJECTED, " + (rm.isRetryable() ? "RETRYABLE)" : " NOT RETRYABLE)");
                } else if(next instanceof PromotionMessage pm) {
                    incomingMessages.add(pm);
                    return "Promotion Message from " + pm.getSenderId() + "(DEFERRED - already cycleBuilder)";
                } else if(next instanceof PositioningMessage pm) {
                    // If the sender is our pending child, we can accept the message and process it normally.
                    if(pm.getSenderId() == pendingChildID) {
                        //Check if assignment is to current location
                        if(!checkAssignmentForCurrentPosition(pm)) {
                            forwardRejectionUpstream(pm, false);
                            log("-> assignment REJECTED by " + pm.getSenderId() + "(WILL BREAK FORMATION)");
                            return "Positioning Message from " + pm.getSenderId() + "(REJECTED)";
                        }

                        //Check chain list to see who's list is larger, and if the incoming message has a larger list, we should accept it and send rejection to current parent with retryable
                        if(pm.getChainList().size() > chainMemberList.size()) {
                            forwardRejectionToParent(true);
                            pendingChildID = -1;
                            setAssignedEdge(pm.getAssignedVertexID(), pm.getAssignedOutgoingEdgeID());
                            setOriginEdge(pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
                            unableToDoAssignmentIDs.clear();
                            unableToDoAssignmentIDs.add(pm.getSenderId());
                            chainMemberList = pm.getChainList();
                            log("-> Positioning Message from " + pm.getSenderId() + "(ACCEPTED, incoming chain list is larger)");
                            return "Positioning Message from " + pm.getSenderId() + "(ACCEPTED, incoming chain list is larger)";

                        //If incoming list is smaller, we should wait for other to complete our task as it will override
                        } else if(pm.getChainList().size() < chainMemberList.size()) {
                            //If a root sent the message, we should tell the root that we are rejecting the assignment and that it is retryable so that the root can reassign us
                            if(pm.getChainList().getRootID() == pm.getSenderId()) {
                                forwardRejectionUpstream(pm, true);
                            }
                            log("-> Positioning Message from " + pm.getSenderId() + "(REJECTED, incoming chain list is smaller)");
                            return "Positioning Message from " + pm.getSenderId() + "(REJECTED, incoming chain list is smaller)";
                        } else {
                            //Accept message if root is smaller
                            if(pm.getChainList().getRootID() < chainMemberList.getRootID()) {
                                forwardRejectionToParent(true);
                                pendingChildID = -1;
                                setAssignedEdge(pm.getAssignedVertexID(), pm.getAssignedOutgoingEdgeID());
                                setOriginEdge(pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
                                unableToDoAssignmentIDs.add(pm.getSenderId());
                                chainMemberList = pm.getChainList();
                                log("-> Positioning Message from " + pm.getSenderId() + "(ACCEPTED, incoming root ID is smaller");
                                return "Positioning Message from " + pm.getSenderId() + "(ACCEPTED, incoming root ID is smaller)";
                            
                            //Send rejection retryable if root is larger
                            } else {
                                forwardRejectionUpstream(pm, true);
                                return "Positioning Message from " + pm.getSenderId() + "(REJECTED, incoming root ID is larger)";
                            }
                        }
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
                    removeEdgeToChild(rm.getSenderId());
                    log("-> assignment REJECTED by " + rm.getSenderId());
                    if(rm.isRetryable()) {
                        log("-> assignment is retryable, will attempt to reassign");
                    } else {
                        unableToDoAssignmentIDs.add(rm.getSenderId());
                        log("-> assignment is NOT retryable, will not attempt to reassign");
                    }
                    return "Reject Assignment Message from " + rm.getSenderId() + "(REJECTED, " + (rm.isRetryable() ? "RETRYABLE)" : " NOT RETRYABLE)");
                } else if(next instanceof PositioningMessage pm) {
                    if(!checkAssignmentForCurrentPosition(pm)) {
                        forwardRejectionUpstream(pm, false);
                        log("-> assignment REJECTED by " + pm.getSenderId() + "(WILL BREAK FORMATION)");
                        return "Positioning Message from " + pm.getSenderId() + "(REJECTED)";

                        //If a root sent the message, check if the next edge's cycle is complete
                    } else if(pm.getChainList().getRootID() == pm.getSenderId()){
                        HalfEdge nextEdge = inferNextEdge(getAssignedEdgeFromMessage(pm));

                        if(completedCycles.get(nextEdge.getId())) {
                            forwardSuccessUpstream(pm.getChainList().getSenderID(), pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
                            log("-> root received message from another root, but next edge's cycle is complete, forwarding SUCCESS upstream");
                            return "Positioning Message from " + pm.getSenderId() + "(ACCEPTED)";
                        } else if(!hasFailed()) {
                            log("-> root received message from another root, but next edge's cycle is NOT complete, forwarding REJECTION upstream (RETRYABLE)");
                            forwardRejectionUpstream(pm, true);
                            return "Positioning Message from " + pm.getSenderId() + "(REJECTED)";

                        } else {
                            forwardFailureUpstream(pm);
                            log("-> root received message from another root, but next edge's cycle is NOT complete, forwarding REJECTION upstream (NOT RETRYABLE)");
                            return "Positioning Message from " + pm.getSenderId() + "(REJECTED)";
                        }

                    } else {
                        log("-> reached root, reporting SUCCESS, forwarding upstream");
                    forwardSuccessUpstream(pm.getChainList().getSenderID(), pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
                    return "Positioning Message from " + pm.getSenderId() + "(ACCEPTED)";
                    }
                } else if(next instanceof PromotionMessage pm) {
                    //Neighbor promoted to stable, try again for cycle building
                    hasFailed = false;
                    log("-> root received Promotion Message from " + pm.getSenderId() + ", clearing hasFailed and will attempt to reassign");
                    return "Promotion Message from " + pm.getSenderId() + "(WILL ATTEMPT TO COMPLETE CYCLES)";
                } else {
                    log("-> root received unexpected message type: " + next.getMessageType());
                    return "N/A (Unhandled message type: " + next.getMessageType() + ")";
                }
            case CycleRole.stable:
                if(next instanceof PositioningMessage pm) {
                    if(!checkAssignmentForCurrentPosition(pm)) {
                        forwardRejectionUpstream(pm, false);
                        log("-> assignment REJECTED by " + pm.getSenderId() + "(WILL BREAK FORMATION)");
                        return "Positioning Message from " + pm.getSenderId() + "(REJECTED)";
                    } else {
                        log("-> reached stable, reporting SUCCESS, forwarding upstream");
                        forwardSuccessUpstream(pm.getChainList().getSenderID(), pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
                        return "Positioning Message from " + pm.getSenderId() + "(ACCEPTED)";
                    }
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

                HalfEdge targetEdge = retrieveEdgeFromGraph(targetEdgeID);

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
                        + " for edge " + targetEdge.getId() + " of vertex " + targetEdge.getOrigin().getId();
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

                HalfEdge targetEdge = inferNextEdge();

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
                return "Assigned position to robot " + child.getRobotId() + " for edge " + targetEdge.getId() + " of vertex " + targetEdge.getOrigin().getId();
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

    private void forwardFailureUpstream(PositioningMessage pm) {
        GeometricCycleLatticeRobot parent = getNeighborByID(pm.getSenderId());
        int originVertexID = pm.getOriginVertexID();
        int originOutgoingEdgeID = pm.getOriginOutgoingEdgeID();

        StatusMessage sm = new StatusMessage(self.getRobotId(), parent.getRobotId(), false, originVertexID, originOutgoingEdgeID);
        send(parent, sm);
    }


    private void forwardRejectionUpstream(PositioningMessage pm, boolean isRetryable) {
        RejectAssignmentMessage rm = new RejectAssignmentMessage(pm.getRecipient(), pm.getSenderId(), pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID(), isRetryable);
        GeometricCycleLatticeRobot robot = getNeighborByID(pm.getSenderId());
        send(robot, rm);
    }

    private void forwardRejectionToParent(boolean isRetryable) {
        GeometricCycleLatticeRobot parent = getNeighborByID(chainMemberList.getSenderID());
        RejectAssignmentMessage rm = new RejectAssignmentMessage(self.getRobotId(), parent.getRobotId(), originVertexID, originOutgoingEdgeID, isRetryable);
        send(parent, rm);
    }

    private boolean checkAssignmentForCurrentPosition(PositioningMessage pm) {
        OrientedPoint localAssignment = getAssignedLocalPosition(pm);
        log("Assigned local position is: " + localAssignment);

        // Self, expressed in self's own frame, is always exactly the origin --
        // not just approximately so for zero-rotation edges, as computing it
        // via RigidBodyTransformation(self).inverse().apply(self) would give
        // (RigidBodyTransformation.apply() discards the input point's own
        // orientation, so that formula silently returned (0, 0, -selfOrientation)
        // instead of (0, 0, 0) whenever self's orientation was nonzero).
        OrientedPoint positionInLocal = new OrientedPoint(0, 0, 0);

        return MathUtils.approxEquals(localAssignment.x, positionInLocal.getX(), MathUtils.POSITION_EPSILON)
                && MathUtils.approxEquals(localAssignment.y, positionInLocal.getY(), MathUtils.POSITION_EPSILON)
                && MathUtils.isZero(MathUtils.angleDifference(localAssignment.getOrientation(), positionInLocal.getOrientation()));
    }

    //ROOT-RELATED UTIL
    private void initializeEdgeMap() {
        Role myRole = getCurrentRole();
        List<HalfEdge> edges = graph.getOutgoingHalfEdges(myRole);
        for(HalfEdge edge : edges) {
            completedCycles.put(getEdgeIDof(edge), false);
        }
    }

    private void promoteAdjacentVerticesToRoots() {
        Role myRole = getCurrentRole();
        List<HalfEdge> edges = graph.getOutgoingHalfEdges(myRole);

        for(HalfEdge edge : edges) {
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

    private GeometricCycleLatticeRobot findBestNeighborForEdge(HalfEdge targetEdge) {
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

    private OrientedPoint getTargetInLocalCoordinates(HalfEdge edge) {
        return edge.getVoltage().apply(new OrientedPoint(0, 0, 0));
    }

    /**
     * EDGE AND ROLE UTIL
     */
    private HalfEdge inferNextEdge() {
        return inferNextEdge(getAssignedEdge());
    }

    private HalfEdge inferNextEdge(HalfEdge assignedEdge) {
        if (assignedEdge == null) return null;

        // next(h) is already "cross to twin(h), then rotate" -- Edmonds' rule
        // -- resolved once by VoltageGraphBuilder.build() and stored on the
        // half-edge itself. See DCEL-Implementation-Plan.md sec 2.2 / 4.
        return graph.getNext(assignedEdge);
    }

    private Role getCurrentRole() {
        HalfEdge assignedEdge = getAssignedEdge();

        if(assignedEdge == null) {
            return graph.getPrimaryRole();
        }

        return assignedEdge.getTarget();
    }

    public HalfEdge getAssignedEdge() {
        if(assignedVertexID == -1 || assignedOutgoingEdgeID == -1) {
            return null;
        }
        return retrieveEdgeFromGraph(assignedOutgoingEdgeID);
    }

    public HalfEdge getAssignedEdgeFromMessage(PositioningMessage pm) {
        if(pm.getAssignedVertexID() == -1 || pm.getAssignedOutgoingEdgeID() == -1) {
            return null;
        }
        return retrieveEdgeFromGraph(pm.getAssignedOutgoingEdgeID());
    }

    public HalfEdge getOriginEdge() {
        if(originVertexID == -1 || originOutgoingEdgeID == -1) {
            return null;
        }

        return retrieveEdgeFromGraph(originOutgoingEdgeID);
    }

    // vertexID is still stored (see setAssignedEdge/setOriginEdge) so the
    // wire format shared with PositioningMessage/PromotionMessage/etc. stays
    // unchanged, but it's no longer needed for lookup: HalfEdge ids are
    // globally unique (unlike the old per-vertex-local LatticeEdge ids), so
    // edgeID alone identifies the edge.
    private HalfEdge retrieveEdgeFromGraph(int edgeID) {
        return graph.getHalfEdgeById(edgeID);
    }

    public void setAssignedEdge(int vertexID, int edgeID) {
        assignedVertexID = vertexID;
        assignedOutgoingEdgeID = edgeID;
    }

    public void setOriginEdge(int vertexID, int edgeID) {
        originVertexID = vertexID;
        originOutgoingEdgeID = edgeID;
    }

    public int getVertexIDof(HalfEdge e) {
        return e.getOrigin().getId();
    }

    public int getEdgeIDof(HalfEdge e) {
        return e.getId();
    }

    private void removeEdgeToChild(int childID) {
        if (childID == -1) return;
        self.getEdges().removeIf(edge -> edge.getToId() == childID);
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

        HalfEdge assignedEdge = getAssignedEdge();

        if (assignedEdge == null) return null;

        GeometricCycleLatticeRobot parent = getNeighborByID(chainMemberList.getSenderID());
        if (parent == null) return null;

        return globalTransformOf(parent.getPosition(), assignedEdge.getVoltage())
                .apply(new OrientedPoint(0, 0, 0));
    }

    private OrientedPoint getAssignedLocalPosition(PositioningMessage pm) {
        HalfEdge assignedEdge = retrieveEdgeFromGraph(pm.getAssignedOutgoingEdgeID());

        if(assignedEdge == null) return null;

        GeometricCycleLatticeRobot parent = getNeighborByID(pm.getSenderId());

        RigidBodyTransformation assignedGlobalTransform = globalTransformOf(parent.getPosition(), assignedEdge.getVoltage());

        return new RigidBodyTransformation(self.getPosition()).inverse()
                .compose(assignedGlobalTransform)
                .apply(new OrientedPoint(0, 0, 0));
    }

    /**
     * Composes a base pose with a transform relative to it (e.g. an edge's
     * voltage) into the resulting global transform, correctly accumulating
     * rotation. RigidBodyTransformation.apply() discards the orientation of
     * whatever point it's given, so chaining via
     * {@code new RigidBodyTransformation(basePose).apply(relative.apply(origin))}
     * silently drops any rotation `relative` carries -- invisible for
     * square/hex, where every edge's rotation is zero, but wrong for
     * octagon-square, where it usually isn't.
     */
    private static RigidBodyTransformation globalTransformOf(OrientedPoint basePose, RigidBodyTransformation relative) {
        return new RigidBodyTransformation(basePose).compose(relative);
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